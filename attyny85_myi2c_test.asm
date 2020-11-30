.DEVICE attiny85
  .equ fRC       = 8000000
  .equ fCLKPR    = 1
  .equ pTC0      = 256      ; (1<<CS02)|(0<<CS01)|(0<<CS00)
  .equ i4ms      = 125      ; 4 ms takt for OCRA
  .equ i05s      = 125      ; halbe sekunde
  .equ i200ms    = 70
  .equ iPause    = 3       

  .def rSREG    = r15
  .def rTemp    = r16
  ;.def rITmp1   = r17
  .def rCnt     = r18
  ;.def rEep     = r19
  ;.def rITmp0   = r20
  .def rTmp0    = r21
  .def rPause   = r22
  .def rDelay   = r23
  .def rAddr    = r24
  .def rData    = r25
  .def rErr     = r26
  .def rCurBit  = r27

  .equ  LED    = PB4
.MACRO mIntPort
  ldi rTemp, @1
  out @0, rTemp
  .ENDM
.MACRO mLED_AN
  cbi PORTB, LED  ; LED an
  .ENDM
.MACRO mLED_AUS
  sbi PORTB, LED  ; LED aus
  .ENDM
.MACRO mWarte
  ldi rCnt, @0
  warte1:
  tst rCnt
  brne warte1
  .ENDM
.cseg
.org 0; ######## Vektortabelle  ############################
 rjmp RESET  ; 1 - RESET External Pin, Power-on Reset,
 reti        ; 2 - INT0 External Interrupt Request 0
 reti        ; 3 - PCINT0 Pin Change Interrupt Request 0
 reti        ; 4 - TIMER1_COMPA Timer/Counter1 Compare Match A
 reti        ; 5 - TIMER1_OVF Timer/Counter1 Overflow
 reti        ; 6 - TIMER0_OVF Timer/Counter0 Overflow
 reti        ; 7 - EE_RDY EEPROM Ready
 reti        ; 8 - ANA_COMP Analog Comparator
 reti        ; 9 - ADC ADC Conversion Complete
 reti        ;10 - TIMER1_COMPB Timer/Counter1 Compare Match B
 rjmp TIMER0_COMPA;11 - Compare Match A
 reti        ;12 - TIMER0_COMPB Timer/Counter0 Compare Match B
 reti        ;13 - WDT Watchdog Time-out
 reti        ;14 - USI_START USI START
 reti        ;15 - USI_OVF USI Overflow
TIMER0_COMPA:; ######## Timer Compare  ############################
  in    rSREG,SREG
  tst   rCnt
  breq  isr11_e
  dec   rCnt
 isr11_e:
  out SREG,rSREG
 reti
RESET:
  mIntPort SPL, LOW(RAMEND)
  mIntPort SPH, HIGH(RAMEND)
  mIntPort MCUCR, 1<<SE
  ; Ausgänge
  mIntPort DDRB,  (1<<LED)  ; Ein/Ausgänge
  ; Timer 0
  mIntPort TIMSK, (1<<OCIE0A)    ; erlaube OCR0A interrupt
  mIntPort TCCR0A, (0<<WGM02)|(1<<WGM01)|(0<<WGM00) ; CTC
  mIntPort OCR0A,  i4ms-1
  mIntPort TCCR0B, (1<<CS02)|(0<<CS01)|(0<<CS00)
  sei
  cbi   PORTB, LED
START:
 a0:
  ldi   rAddr, 0x70 ;0x70 Slave Adresse from PCF8574
  andi  rAddr, 0xFE ; Slave Adresse in rAddr, LSB 0 weil Write
  rcall i2c_start
  rcall i2c_send
  rcall i2c_stop
  tst   rErr
  breq  a1
  mWarte 50
  sbi   PINB,LED
  rjmp a0
  a1:
LOOP:
  cbi   PORTB, LED
  ldi   rAddr, 0x70 ;0x70 Slave Adresse from PCF8574
  ldi   rData, 0xFE ; 
  rcall i2c_write    ;
  tst   rErr
  brne  a0
LOOP2:
  rcall i2c_read
  sbrs  rData,7
  rjmp  LOOP
  sbi   PORTB, LED
  ldi   rData, 0xFF ; 
  rcall i2c_write    ;
  tst   rErr
  brne  a0

  sleep
  nop
  rjmp  LOOP2
.include "../lib/myi2c_new.asm"
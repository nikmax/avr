.DEVICE attiny85
  .equ fRC       = 8000000
  .equ fCLKPR    = 1
  .equ pTC0      = 256      ; (1<<CS02)|(0<<CS01)|(0<<CS00)
  .equ i4ms      = 125      ; 4 ms takt for OCRA
  .equ i05s      = 125      ; halbe sekunde
  .equ i200ms    = 70
  .equ iPause    = 3     
  .equ i1s       = 250 

  .def rTime    = r1
  .def rT60     = r2
  .def rT24     = r3
  .def rSec     = r4
  .def rMin     = r5
  .def rStd     = r6
  .def rTC      = r7
  .def rSREG    = r15
  .def rTemp    = r16
  .def rITmp1   = r17
  .def rCnt     = r18
  .def rEep     = r19
  .def rITmp0   = r20
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
.MACRO mTogle_LED
  ldi rTmp0, 1<<LED
  in  rTemp, PORTB
  eor rTemp, rTmp0
  out PORTB, rTemp
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
  rjmp  isr11_e
  isr11_e:
  inc rTC
  cp rTC, rTime
  brne t_ende
  clr rTC
  inc rSec
  cp rSec, rT60
  brne t_ende
  clr rSec
  inc rMin
  cp rMin, rT60
  brne t_ende
  clr rMin
  inc rStd
  cp rStd, rT24
  brne t_ende
  clr rStd
  ; hiert kommt tag
 t_ende:
  out SREG,rSREG
 reti
RESET:
  ; Zeit einstellen
  ldi rTemp, i1s
  mov rTime, rTemp
  ldi rTemp, 60
  mov rT60,  rTemp
  ldi rTemp, 24
  mov rT24, rTemp
  clr rTC
  clr rSec
  clr rMin
  clr rStd

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
START:
  mLED_AN
 a0:
  ldi   rAddr, 0x4C ; Slave Adresse  4C/2zeilig - 4E/4zeilig
  rcall lcd_init
  cpi   rErr, 0xFF
  brne  a1
  mWarte i05s
  mTogle_LED
  rjmp a0
 a1:  
  rcall lcd_clear
  rcall lcd_row_1
  ldi rData, "O"
  ldi r31, HIGH(MyStr*2)
  ldi r30, LOW(MyStr*2)
  rcall lcd_string
  mov rTemp,rSec
LOOP:
  ldi rCnt, i05s
  l1:
  tst rCnt
  brne l1
  rcall lcd_row_2
  mov  rData, rStd
  rcall lcd_dec
  ldi rData,":"
  rcall lcd_data
  mov  rData, rMin
  rcall lcd_dec
  ldi rData,":"
  rcall lcd_data
  mov  rData, rSec
  rcall lcd_dec
  rjmp  LOOP
MyStr:
  ;
  .DB "Hallo, Welt!",0,0
sOKp:
  ;
  .DB "OK+",0
sOKm:
  ;
  .DB "OK-",0
;**************************************
.include "mylcd.asm"
.include "myi2c.asm"

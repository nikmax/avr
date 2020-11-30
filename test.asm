.DEVICE attiny13
  .equ fRC    = 9600000
  .equ fCLKPR = 8
  .equ pTC0   = 8      ; (0<<CS02)(1<<CS01)(0<<CS00)
  .equ millis = 1000

  .def rTemp    = r16


  .equ LED2 = 1
  .equ LED1 = 0
  .equ TASTER = 4
  .def rFlag = r25

.MACRO mInitPort
  ldi rTemp, @1
  out @0, rTemp
  .ENDM

  .cseg
.org 0; ######## Vektortabelle  ############################
  rjmp RESET    ; 0 - RESET
  reti			; 1 - INT0
  reti          ; 2 - PCINT
  reti			; 3 - TIM0_OVF
  reti			; 4 - EE_RDY
  reti			; 5 - ANA_COMP
  reti          ; 6 - TIM0_COMPA
  reti			; 7 - TIM0_COMPB
  reti			; 8 - WDT
  reti			; 9 - ADC


RESET:
  mInitPort SPH,    HIGH(RAMEND)
  mInitPort SPL,    LOW(RAMEND)
  mInitPort MCUCR,  (1<<SE)                            ; schlafen erlaubt

  sei
LOOP:
  sbi DDRB, LED1  ; als Ausgang für LED
  sbi DDRB, LED2
  sbi PORTB,TASTER   ; PULL-UP für Taster

    ldi rFlag, 0

abfrage:
    cpi rFlag, 0
    brne an
    sbi PORTB, LED1
    rjmp w
    an: 
    cbi PORTB, LED1

    w:
    cbi  PORTB, LED2  ; LED2 leuchet, solange Taste losgelassen
    sbic PINB, TASTER ; wenn Taster an (gnd an PB4) überspringe
    rjmp abfrage

    cpi rFlag,0
    brne led_aus
    ldi rFlag,1
    rjmp tasteloslassen
    led_aus:
    ldi rFlag,0 
    tasteloslassen:     ; warte bvis Tasste losgelassen, und
      rcall warte
      sbi PORTB,LED2     ; solange LED2 aus
      sbis PINB, TASTER
      rjmp tasteloslassen
    rjmp abfrage



warte:
  ldi yh, 100
  ldi yl, 0
w2:
  sbiw yl, 1
  brne w2
  ret


  sleep
  nop
  rjmp LOOP
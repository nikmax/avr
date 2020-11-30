.DEVICE attiny13
  ; an LED wird Wert aus rErr ausgeblinkt
  ; rCnt, rEep muss lsl beim laden, rPause, rITmp0, rITmp1
  .equ fRC       = 9600000
  .equ fCLKPR    = 8
  .equ pTC0      = 256      ; (1<<CS02)|(0<<CS01)|(0<<CS00)
  .equ i4ms      = 250      ; 4 ms takt for OCRA
  .equ i05s      = 125      ; halbe sekunde
  .equ i200ms    = 70
  .equ iPause    = 8       

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
  .equ eeADR = 0
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
  breq  tpause
  dec   rCnt
  rjmp  isr11_e
 tpause:
  ldi   rCnt, i200ms
  tst   rPause
  breq  no_pause
  dec   rPause
  rjmp  isr11_e
 no_pause:
  tst rEep
  breq pause
  ldi rITmp1, 1<<LED  ; und lase LED blinken
  in  rITmp0, PORTB
  eor rITmp0, rITmp1
  out PORTB, rITmp0   ; toggle end
  dec rEep
  rjmp isr11_e
 pause:
  ldi rPause, iPause
  mov  rEep, rErr
  lsl  rEep          ; doppelt weil toggle
 isr11_e:
  out SREG,rSREG
 reti
RESET:
  ;mIntPort SPL, LOW(RAMEND)
  ;mIntPort SPH, HIGH(RAMEND)
  mIntPort MCUCR, 1<<SE
  ; Ausgänge
  mIntPort DDRB,  (1<<LED)  ; Ein/Ausgänge
  ; Timer 0
  mIntPort TIMSK0, (1<<OCIE0A)    ; erlaube OCR0A interrupt
  mIntPort TCCR0A, (0<<WGM02)|(1<<WGM01)|(0<<WGM00) ; CTC
  mIntPort OCR0A,  i4ms-1
  mIntPort TCCR0B, (0<<CS02)|(1<<CS01)|(0<<CS00)
  ;rcall USI_TWI_Master_Initialise
  mLED_AUS
  ldi rPause, 0
  ldi rCnt, 0
  ldi rErr, 3
  mov rEep, rErr
  lsl rEep
  sei

 ww0:
  ldi rData, 0x10  ; 010 bis 0xEF
ww:
  rcall i2c_write
  cpi rErr, 2
  breq LOOP0
  ;ldi  rErr, 6
  inc   rData
  cpi   rData, 0xF0
  brne  ww
  rjmp  ww0
  LOOP0:
  mov rEep, rData
  ;rcall fROM_Write

LOOP:
  sleep
  nop
  rjmp LOOP
; Bit and byte definitions
  .equ TWI_READ_BIT  = 0       ; Bit position for R/W bit in "address byte".
  .equ TWI_ADR_BITS  = 1       ; Bit position for LSB of the slave address bits in the init byte.
  .equ TWI_NACK_BIT  = 0       ; Bit position for (N)ACK bit.
; Defines error code generating
  .equ USI_TWI_NO_DATA           = 0x08  ;// Transmission buffer is empty
  .equ USI_TWI_DATA_OUT_OF_BOUND = 0x09  ;// Transmission buffer is outside SRAM space
  .equ USI_TWI_UE_START_CON      = 0x07  ;// Unexpected Start Condition
  .equ USI_TWI_UE_STOP_CON       = 0x06  ;// Unexpected Stop Condition
  .equ USI_TWI_UE_DATA_COL       = 0x05  ;// Unexpected Data Collision (arbitration)
  .equ USI_TWI_NO_ACK_ON_DATA    = 0x02  ;// The slave did not acknowledge  all data
  .equ USI_TWI_NO_ACK_ON_ADDRESS = 0x01  ;// The slave did not acknowledge  the address
  .equ USI_TWI_MISSING_START_CON = 0x03  ;// Generated Start Condition not detected on bus
  .equ USI_TWI_MISSING_STOP_CON  = 0x04  ;// Generated Stop Condition not detected on bus
  .equ USI_TWI_BAD_MEM_READ      = 0x0A  ;// Error during external memory read
; Register defines
  .equ DDR_USI  =  DDRB
  .equ PORT_USI =  PORTB
  .equ PIN_USI  =  PINB
  .equ PIN_SDA  =  PB0
  .equ PIN_SCL  =  PB2
delay5us: ; T2_TWI
  push rDelay     ; 2
  ldi  rDelay, 2 ; 1
  delay_5m:
  dec rDelay      ; rDelqy * 1
  brne delay_5m    ; rDelay * 2 + 1
  pop  rDelay     ; 2
  ret             ; 4 + 3 rcall   => 12 + 3*(rDelay-1)+2 =>  11 + 3rDelay
delay4us: ; T4_TWI
  push rDelay     ; 2
  ldi  rDelay, 1  ; 1
  delay_4m:
  dec rDelay      ; rDelqy * 1
  brne delay_4m    ; rDelay * 2 + 1
  pop  rDelay     ; 2
  ret             ; 4 + 3 rcall   => 12 + 3*(rDelay-1)+2 =>  11 + 3rDelay
.macro SDA_AUSgang
  ; SDA als Ausgang
  sbi   DDR_USI,  PIN_SDA
  .endm 
.macro SDA_EINgang
  ; SDA als Ausgang
  cbi   DDR_USI,  PIN_SDA
  .endm
i2c_write:
  ;ldi   rErr, 5
  ; 0*** Zunächst befinden sich alle Bausteine im Ruhezustand. 
  ; Er ist durch SDA = 1 und SCL= 1 gekennzeichnet.
  sbi   PORT_USI, PIN_SDA  ;
  sbi   PORT_USI, PIN_SCL
  sbi   DDR_USI,  PIN_SDA
  sbi   DDR_USI,  PIN_SCL
  ; 1*** Nun sendet der Master ein Start-Signal, indem er durch
  ; Schließen der beiden Schalter zuerst SDA und dann SCL auf 0 setzt.
  ; Dadurch werden alle Slaves des Bus-Systems in Bereitschaft versetzt:
  ; Sie warten jetzt auf das Adressbyte (vgl. Schritt 2).
  wait_scl_1:
  sbis  PIN_USI,  PIN_SCL  ; Verify that SCL becomes high.
  rjmp wait_scl_1
  ;
  sbi   DDR_USI,  PIN_SDA
  sbi   DDR_USI,  PIN_SCL
  rcall delay5us
  cbi   PORT_USI, PIN_SDA   ;  zuerst SDA
  rcall delay4us
  cbi   PORT_USI, PIN_SCL   ; und dann SCL 
  ; 2*** Der Master sendet nun bitweise die Adresse des Slaves.
  ; Dabei beginnt er bei dem höchstwertigen Bit (Bit 7),
  ; das am weitesten links steht.
  ldi   rCurBit, 8  ; 8 Bits beginne mit Bit 7
  mov   rAddr, rData ; Slave Adresse
  ;ori   rAddr, 0x01 ; => LSB 1 weil read
  ; 2.1 *** Der Master legt Bit 7 an SDA; Kurz darauf öffnet
  ; der Master den SCL-Schalter, so dass die SCL-Leitung
  ; vom Zustand 0 in den Zustand 1 übergeht. Dies ist für
  ; die Slaves das Signal, das Bit 7 von der Datenleitung SDA
  ; entgegenzunehmen und in einem Puffer zwischenzuspeichern.
  ; Kurze Zeit später setzt der Master die Clockleitung SCL wieder auf 0.


 next_bit:
  cbi   PORT_USI, PIN_SDA
  sbrc  rAddr, 7          ; MSB
  sbi   PORT_USI, PIN_SDA ; Der Master legt Bit  an SDA
  rcall delay4us
  sbi   PORT_USI, PIN_SCL ; Kurz darauf öffnet der Master den SCL
  rcall delay4us
  ; 1 Bit raus, prüfe ob Slave noch Zeit braucht
  cbi   DDR_USI,  PIN_SCL ; als Eingang
  wait_scl_2:
   sbis  PIN_USI,  PIN_SCL; Verify that SCL becomes high.
   rjmp wait_scl_2
  sbi   DDR_USI,  PIN_SCL ; als Ausgang
  ; geprüft ...
  cbi   PORT_USI, PIN_SCL ; die Clockleitung SCL wieder auf 0
  ; 2.3-2.8 *** Bit 6 bis Bit 0 werden auf gleiche Weise übertragen.
  dec   rCurBit    ; sind alle bitts übertragen?
  breq  TesteACK  ; ja - weiter
  lsl   rAddr     ; sonst schiebe für das nächste Bit auf MSB pos.
  rjmp  next_bit
 TesteACK:
  rcall delay5us
  ;2.9 *** Alle Slaves vergleichen nach dem Empfang des Bit 0, 
  ; ob das empfangene und zwischengespeicherte Byte mit ihrer
  ; eigenen Adresse übereinstimmt. Derjenige Baustein, welcher
  ; Übereinstimmung feststellt, legt nun SDA auf 0; dieses Signal
  ; wird Acknowledge-Signal (ACK) genannt. Der Master belässt/schaltet
  ; derweil SDA auf 1 und legt SCL auf 1. Er prüft nun, ob 
  ; die SDA-Leitung auf 0 oder 1 liegt. Liegt sie auf 1, geht der Master
  ; davon aus, dass sich kein Slave mit der gesendeten Adresse
  ; im Bus-System befindet. Liegt auf SDA aber ein 0, zeigt dies
  ; dem Master, dass der gewünschte Slave gefunden wurde. Der Master
  ; setzt nun seinerseits SCL wieder auf 0 (ACK_yes) und zeigt damit dem Slave,
  ; dass er dessen Acknowledge-Signal erhalten hat. Daraufhin öffnet
  ; der Slave wieder seinen SDA-Schalter (SDA wird dadurch auf 1 gesetzt,
  ; so dass der Master im Folgenden die Datenleitung wieder benutzen kann);
  ; der Slave ist jetzt zum Empfang eines weiteren Bytes (Datenbyte) bereit.
  ; Alle anderen Slaves gehen wieder in den Ruhezustand über, 
  ; bis wieder ein Startsignal erfolgt.
  sbi   PORT_USI, PIN_SDA ; auf 1 
  cbi   DDR_USI,  PIN_SDA ; als Eingang
  sbi   PORT_USI, PIN_SCL ; Kurz darauf öffnet der Master den SCL
  rcall delay4us
  ; prüfe ob Slave noch Zeit braucht
  cbi   DDR_USI,  PIN_SCL ; als Eingang
  wait_scl_3:
   sbis  PIN_USI,  PIN_SCL; Verify that SCL becomes high.
   rjmp wait_scl_3
  sbi   DDR_USI,  PIN_SCL ; als Ausgang
  ; geprüft ...lese ACK ...

  sbis  PIN_USI,  PIN_SDA
  rjmp  ACK_yes
  ldi   rErr, USI_TWI_NO_ACK_ON_ADDRESS
  sbi   DDR_USI, PIN_SDA ; wieder als ausgang und ...
  ret                     ; .. tschüss  SDA = 1 SCL = 1 =>  ruhe
 ACK_yes:
  cbi   PORT_USI, PIN_SCL ; die Clockleitung SCL wieder auf 0
  sbi   DDR_USI, PIN_SDA ; wieder als ausgang
  rcall delay5us
  ; alls ok ... byte senden  ... wie ab   1 ***

  ; STOP condition
  cbi   PORT_USI, PIN_SDA
  sbi   PORT_USI, PIN_SCL
  cbi   DDR_USI,  PIN_SCL ; als Eingang
  wait_scl_6:
   sbis PIN_USI,  PIN_SCL; Verify that SCL becomes high.
   rjmp wait_scl_6
  sbi   PORT_USI, PIN_SDA  ; STOP COND
  sbi   DDR_USI,  PIN_SCL ; wieder Ausgang
  rcall delay5us  
  ret ; i2c_write
;.include "../attiny85/eeprom.inc"
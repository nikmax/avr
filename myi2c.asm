
; Register müssen im Hauptprogramm definiert werden
;    .equ DDR_USI  =  DDRB
;    .equ PORT_USI =  PORTB
;    .equ PIN_USI  =  PINB
;    .equ PIN_SDA  =  PB0
;    .equ PIN_SCL  =  PB1
;    .def rAddr    =  r24
;    .def rData    =  r23
Warte4us:
    .equ c4us     = cXTAL/1000000
    push r16     ; 2
    in  r16,sreg
    push r16
    ldi  r16, c4us  ;
    _delay_4m:
    dec r16      ; r16 * 1
    brne _delay_4m    ; r16 * 2 + 1
    pop  r16     ; 2
    out sreg, r16
    pop r16
    ret             ; 4 + 3 rcall   => 12 + 3*(r16-1)+2 =>  11 + 3r16
i2c_wait_scl:
    rcall   Warte4us
    sbi     PORT_USI, PIN_SCL ; Kurz darauf öffnet der Master den SCL
    rcall   Warte4us
    i2c_wait_scl1:
    sbis    PIN_USI,  PIN_SCL  ; Warte bis der Bus frei ist => SCL = High
    rjmp    i2c_wait_scl1
    rcall Warte4us
    ret
i2c_init:  ; es muss immer send oder recive folgen
    sbi     DDR_USI,  PIN_SDA
    sbi     DDR_USI,  PIN_SCL
    sbi     PORT_USI, PIN_SCL
    sbi     PORT_USI, PIN_SDA
    ret
i2c_start:
    rcall   i2c_wait_scl
    cbi     PORT_USI, PIN_SDA   ; START COND:  zuerst SDA auf LOW
    rcall   Warte4us            ; oder Warte5us ?
    cbi     PORT_USI, PIN_SCL   ;          ..und dann SCL auf LOW
    rcall   Warte4us            ; oder Warte5us ?
    ret     ; 
i2c_stop:   ; STOP condition
    cbi     PORT_USI, PIN_SDA
    rcall   i2c_wait_scl
    sbi     PORT_USI, PIN_SDA  ; STOP COND: SDA geht hoch wärend SCL schon hoch ist
    ret
i2c_send:   ; rAddr, Fehler = Carry set
    push    rAddr
    push    r16
    ldi     r16, 8  ; 8 Bits beginne mit Bit 7
    next_bit:
    cbi     PORT_USI, PIN_SDA ; vorladen als 0
    sbrc    rAddr, 7          ; und prüfen MSB
    sbi     PORT_USI, PIN_SDA ;  ...doch als 1
    rcall   i2c_wait_scl      ; Der Master legt Bit  an SDA
    cbi     PORT_USI, PIN_SCL ; die Clockleitung SCL wieder auf 0
    rcall    Warte4us
    lsl     rAddr             ; schiebe für das nächste Bit auf MSB pos.
    dec     r16               ; sind alle bitts übertragen?
    brne    next_bit
    getACK:
    sbi     PORT_USI, PIN_SDA ; PULL-UP und
    cbi     DDR_USI,  PIN_SDA ; als Eingang
    rcall   i2c_wait_scl      ; prüfe ob Slave noch Zeit braucht
    clc
    sbic    PIN_USI,  PIN_SDA ; jetzt lese ACK ...also 0 wenn ok
    sec
    rjmp    i2c_ext           ; ende, wenn Fehler => Carry set
i2c_recive: ; rData <= byte
    push    rAddr
    push    r16
    sbi     PORT_USI, PIN_SDA ; pull up
    cbi     DDR_USI,  PIN_SDA ; auf empfang
    rcall   Warte4us            ; oder Warte5us ?
    clr     rData
    ldi     r16, 8            ; 8 Bits beginne mit Bit 7
    next_bit_r:
    lsl     rData             ; lsb frei machen
    rcall   i2c_wait_scl
    sbic    PIN_USI,  PIN_SDA
    ori     rData,0b00000001  ; bit als 1 empfangen
    cbi     PORT_USI, PIN_SCL ; die Clockleitung SCL wieder auf 0
    dec     r16               ; sind alle bitts übertragen?
    brne    next_bit_r
    sendACK:
    cbi     PORT_USI, PIN_SDA ; auf 0 
    sbi     DDR_USI,  PIN_SDA ; als Ausgang
    rcall   Warte4us            ; oder Warte5us ?
    rcall   i2c_wait_scl      ; sende ACK ....
    clc
i2c_ext:    ; Ausgang ; bei Fehler rErr = 0xFF
    rcall   Warte4us
    cbi     PORT_USI, PIN_SCL ; die Clockleitung SCL wieder auf 0
    sbi     DDR_USI,  PIN_SDA ; wieder als ausgang und bleibt als HIGH!!!
    rcall   Warte4us            ; oder Warte5us ?
    pop     r16
i2c_ext2:   ; Ende
    pop     rAddr
    ret
i2c_write:  ; rAddr, rData // rErr  // rCurBit, r16
    push    rAddr
    andi    rAddr, 0xFE       ; Slave Adresse in rAddr, LSB 0 weil Write
    rcall   i2c_start
    rcall   i2c_send; wenn set Carry => Fehler
    brcs    i2c_rw_ext
    mov     rAddr, rData ; alls ok ... byte senden 
    rcall   i2c_send  
    rjmp    i2c_rw_ext
i2c_read:   ; rAddr, rData // rErr  // rCurBit, r16
    push    rAddr
    ori     rAddr, 0x01  ; Slave Adresse in rAddr, LSB 1 weil read
    rcall   i2c_start
    rcall   i2c_send
    brcs    i2c_rw_ext
    rcall   i2c_recive
i2c_rw_ext: ; wenn set Carry => Fehler
    rcall   i2c_stop
    rjmp    i2c_ext2
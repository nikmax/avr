; .equ cXTAL   = muss Taktfrequenz haben
; .def rLcdC   = r22
; .equ cLcdAddr= 0x4C
; *** i2c ***
; .def rAddr   = r20
; .def rData   = r21
;---------------------------------------------
; Pins an PCF8574
; Bit 7 - Bit4  Daten immer obere Nibble first
;   LCD     LCD         IO PINS
;   PIN     FUNCTION    PCF8574A        Comments
;--------------------------------------------------------------------------
;   1       VSS                         GND
;   2       VCC                         +5V
;   3       VO          P3              10K POT VCC to GND  Contrast Voltage
;   4       RS          P0
;   5       RW          P1              Busy Detection
;   6       E           P2
;   7-10    DB0-3                       Not used - 4-bit mode used
;   11      DB4         P4
;   12      DB5         P5
;   13      DB6         P6
;   14      DB7         P7
;
;   Note: PCF8574A Address pins: A0=A1=A2=0
; -------- Hardware --------------------
;   PCF8574      _______
;             1 /       |16
;       GND o--|A0   VDD|--o +5V
;             2|        |15
;       GND o--|A1   SDA|--o SDA
;             3|        |14
;       GND o--|A2   SCL|--o SCL
;             4|        |13
;    LCD-RS o--|P0   INT|--o 
;             5|        |12   
;    LCD-RW o--|P1    P7|--o LCD DB7
;             6|        |11
;    LCD-E  o--|P2    P6|--o LCD-DB6
;             7|        |10
;    LCD-V0 o--|P3    P5|--o LCD-DB5
;             7|        |9
;       GND o--|VSS   P4|--o LCD-DB4
;              |________|
;
;---------------------------------------------
  .equ Bkl = 3  ;// LED An/Aus 
  .equ En  = 2  ;// Enable bit
  .equ Rw  = 1  ;// Read/Write bit
  .equ Rs  = 0  ;// Register select bit
; ------ Warteroutinen -----------------------
Warte50ms:  ; Warteroutine 50 ms
    push    r16
    in      r16, sreg
    push    r16
    ldi     r16,10
    w50ms:
    rcall   Warte5ms
    dec     r16
    brne    w50ms
    pop     r16
    out     sreg, r16
    pop     r16
    ret
Warte5ms:   ; Warteroutine 5 ms
    .equ    c5ms = 5000
    .equ    n5ms = (c5ms-16)/ 4 *  (cXTAL / 1000000)
    push    r29
    push    r28
    in      r28,sreg
    push    r28

    ldi     r29,  HIGH(n5ms)
    ldi     r28,   LOW(n5ms)
    Warte: ; Warteroutine Z Takte, Takte = 4*(n-1)+11 = 4*n + 7
    sbiw    r28,  1 ; + 2
    brne    Warte ; + 1 / 2

    pop     r28
    out     sreg, r28
    pop     r28 ; + 2
    pop     r29 ; +2
    ret ; + 4, Gesamt=4*n+18
; --------- Hauptteil ------------------------
LcdInit:   ; --------- LCD-Ansteuerung Init ----------
    ldi     rLcdC,0 ; init pseudo Control Register
    sbr     rLcdC,  1<<Bkl
    rcall Warte50ms ; Warte 50 ms bis LCD hochgefahren ist

    push rData
    push rAddr
    ldi  rAddr, cLcdAddr
    ldi   rData,0x30 ; Versetze in 8-Bit-Modus (drei Mal)
    rcall _LcdEnable 
    rcall _LcdEnable
    rcall _LcdEnable
    ldi   rData,0x20 ; Schalte in 4-Bit-Modus um
    rcall _LcdEnable
    pop  rAddr
    ; Funktionseinstellungen LCD
    ldi   rData,0x28 ; 4-Bit-Modus, 4 Zeilen, 5*7
    rcall LcdCmd
    ldi   rData,0x01 ; lösche Display
    rcall LcdCmd
    ldi   rData,0x0C ; B3 = 1, Display an (B2) , Cursor(B1), Blinken (B0)
    rcall LcdCmd
    ldi   rData,0x06        ; Autoindent
    rcall LcdCmd
    pop  rData
    ret
LcdData:  ; Datenwort-Ausgabe im 4-Bit-Modus Daten in rmp
    rcall _LcdBusy ; warte bis busy = Null
    sbr  rLcdC,1<<Rs ; setze RS-Bit
    rjmp _LcdWrite ; gib Byte in rmp aus
LcdCmd:  ; Kontrollwort-Ausgabe im 4-Bit-Modus Daten in rData
    rcall _LcdBusy ; warte bis busy = Null
    cbr   rLcdC, 1<<Rs ; loesche RS
_LcdWrite:   ; Ausgabe Byte im 4-Bit-Modus mit Busy
    push    rData
    push rAddr
    ldi  rAddr, cLcdAddr
    push    rData
    rcall   _LcdEnable
    pop     rData
    swap    rData          ; Nibble vertauschen
    rcall   _LcdEnable
    rjmp    _lcd_exit
_LcdBusy:    ; Warte bis Busy Null, Steuerung
    push rData       ; rette
    push rAddr
    ldi  rAddr, cLcdAddr
    cbr  rLcdC,1<<Rs ;  RS = 0, weil commando
    _LcdBusy1:
    rcall _LcdRead
    push  rData; Bit 7 Busybit
    rcall _LcdRead
    pop rData
    sbrc rData,7       ; ueberspringe bei Busy=0
    rjmp _LcdBusy1      ; wiederhole bis Busy=0
_lcd_exit:
    pop  rAddr
    pop  rData
    ret ; fertig set carry , wenn fehler
_LcdRead:
    sbr     rLcdC,  1<<Rw ; R/W = 1, weil lesen
    rcall   _LcdEnOn
    rcall   i2c_read     ; lese oberes Nibble
    rcall   _LcdEnOff
    cbr     rLcdC,  1<<Rw  ; R/W loeschen
    ret
_LcdEnable:
    rcall   _LcdEnOn
    rcall   Warte5ms
    rjmp    _LcdEnOff
_LcdEnOn:
  andi  rData, 0xF0  ; unteres Nibble loeschen
  or    rData, rLcdC ; mit Steuerung verknüpfen
  rcall i2c_write    ; vorbereiten
  ;rcall Warte5ms
  sbr   rData,1<<En  ; setze LCD-Enable
  rcall i2c_write    ; LCD-Enable aktivieren
  ;rcall Warte5ms
  ret
_LcdEnOff:
  andi  rData, 0xF0  ; unteres Nibble loeschen. LCD-Enable gelöscht
  or    rData, rLcdC ; mit Steuerung verknüpfen
  rcall i2c_write    ; LCD-Enable aktivieren
  ret
_LcdDone:
; -------------------------------------------
; um die Größe nicht auszudehnen, folgendes:
;
;
; ----------- Beleuchtung an ----------------
;
;  sbr    rLcdC,  1<<Bkl
;  rcall  _LcdBusy
;
; ----------- Beleuchtung aus ---------------
;
;  cbr    rLcdC,  1<<Bkl
;  rcall  _LcdBusy
;
; ----------- ASCII am Cursor ausgeben ------
;
;  ldi    rData,  'X'
;  rcall  LcdData
; 
; -------- gehe zu Zeile 1/2/3/4 ------------ 
;
; 	ldi 	rData,  0x80    ; Zeile 1 oder
; 	ldi 	rData,  0xC0    ; Zeile 2 oder
; 	ldi 	rData,  0x80+16 ; Zeile 3 oder
; 	ldi 	rData,  0xC0+16 ; Zeile 4
; 	rcall 	LcdCmd
;
; --------- Display löschen -----------------
;
; 	ldi 	rData,  0x01
;
; --------- Display off, Cursor off ---------
;      clearing display without clearing
;               DDRAM content
;
; 	ldi 	rData,  0x08
;
; --------- Display on, Cursor on  ----------
;
; 	ldi 	rData,  0x0E
;
; --------- Display on, Cursor off ----------
;
; 	ldi  	rData,  0x0C
;
; --------- Display on, Cursor blinking -----
;
; 	ldi  	rData,  0x0F
;
; --------- Cursor left ---------------------
;
; 	ldi  	rData,  0x10
;
; --------- Cursor right --------------------
;
; 	ldi  	rData,  0x14
;

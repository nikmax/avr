.MACRO mIntPort
  ldi rTemp, @1
  .IF @0 > 63
  sts @0, rTemp
  .ELSE
  out @0, rTemp
  .ENDIF
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

; ------ Warteroutinen ------------------
.equ  cMultiplier =  (cXTAL / 1000000)
; --------- Warteroutinen ------------


Warte50ms:
  rcall Warte25ms
  rjmp  Warte25ms
Warte25ms:  ; Warteroutine 25 ms
  .equ c25ms = 25000
  .equ n25ms = (c25ms-16)/ 4 * cMultiplier
  ;   rcall: + 3
  push r31 ; + 2
  push r30 ; + 2
  ldi r31,HIGH(n25ms) ; + 1
  ldi r30,LOW(n25ms) ; + 1
  rjmp Warte ; + 2, gesamt = 11
Warte5ms: ; Warteroutine 5 ms
  .equ c5ms = 5000
  .equ n5ms = (c5ms-16)/ 4 * cMultiplier 
  push r31
  push r30
  ldi r31,HIGH(n5ms)
  ldi r30,LOW(n5ms)
  rjmp Warte
Warte: ; Warteroutine Z Takte, Takte = 4*(n-1)+11 = 4*n + 7
  sbiw r30, 1 ; + 2
  brne Warte ; + 1 / 2
  pop r30 ; + 2
  pop r31 ; +2
  ret ; + 4, Gesamt=4*n+18

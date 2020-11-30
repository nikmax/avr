;
; Complete instruction set of the AVRs
;   ordered by the assembler mnemonic
;   as test file for gavrasm compiler
;
; Definitions
; Registers
.def r=R16
.def rHi=R16
.def rLo=R0
.def rD=R24
;
; Bits
.equ b = 0
;
; Ports
.equ p = 63
.equ pLo = 31
;
; The instruction set
	adc r,r
	add r,r
	adiw rd,63
	and r,r
	andi rHi,255
	asr r
	bclr b
	bld r,b
	brbc b,+2
	brbs b,+2
	brcc +2
	brcs +2
	brbc b,-2
	brbs b,-2
	brcc -2
	brcs -2
  break
	breq -2
	brge -2
	brhc -2
	brhs -2
	brid -2
	brie -2
	brlo -2
	brlt -2
	brmi -2
	brne -2
	brpl -2
	brsh -2
	brtc -2
	brts -2
	brvc -2
	brvs -2
	bset b
	bst rLo,b
	call 10
	cbi pLo,b
	cbr rHi,255
	clc
	clh
	cli
	cln
	clr r
	cls
	clt
	clv
	clz
	com r
	cp r,r
	cpc r,r
	cpi rHi,255
	cpse r,r
	dec r
	des 8
  eicall
  eijmp
	elpm
  elpm r,Z
  elpm r,Z+
	eor r,r
  fmul r,r
  fmuls r,r
  fmulsu r,r
	icall
	ijmp
	in r,p
	inc r
	jmp 10
	lac Z,r
	las Z,r
	lat Z,r
  ld r,X
	ld r,X+
	ld r,-X
	ld r,Y
	ld r,Y+
	ld r,-Y
	ld r,Z
	ld r,Z+
	ld r,-Z
	ldd r,Y+63
	ldd r,Z+63
	ldi rHi,255
	lds r,65535
	lpm
  lpm r,Z
  lpm r,Z+
	lsl r
	lsr r
	mov r,r
  movw r,r
	mul r,r
  muls r,r
	neg r
	nop
	or r,r
	ori rHi,255
	out p,r
	pop r
	push r
	rcall 10
	ret
	reti
	rjmp 10
	rol r
	ror r
	sbc r,r
	sbci rHi,255
	sbi pLo,b
	sbic pLo,b
	sbis pLo,b
	sbiw rD,63
	sbr rHi,255
	sbrc r,b
	sbrs r,b
	sec
	seh
	sei
	sen
	ser rHi
	ses
	set
	sev
	sez
	sleep
	spm
	spm Z+
	st X,r
	st X+,r
	st -X,r
	st Y,r
	st Y+,r
	st -Y,r
	st Z,r
	st Z+,r
	st -Z,r
	std Y+63,r
	std Z+63,r
	sts 65535,r
	sub r,r
	subi rHi,255
	swap r
	tst r
	wdr
	xch Z,r

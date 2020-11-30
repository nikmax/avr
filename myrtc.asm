; *************************************************
; *** 					init 					*** 
;
; 	.equ 	cRtcAddr	 = 0xD0
;
; 	*** i2c ***
; 	.def rAddr   = r20
; 	.def rData   = r21
;
; *************************************************
RtcInit:
	push 	rAddr
	push 	rData
    ldi   rAddr, cRtcAddr
    clc
    rcall i2c_read
	pop 	rData
	pop 	rAddr
	ret

;
; Attiny10thermometerTM1637display.asm
;
; Created: 04/05/2022 09:06:49
; Author : Manama
;


;pb0 is SDA
;pb2 is SCL
;pb1 is data line for temp sensor

.def temp = r16
.def data = r17
.def counter = r18
.def array_length = r19
.def array_byte = r20
.def ASCII = r21
.def temp1 = r22
.def posx = r23
.def posy = r24
.def result_low = r28
.def result_high = r29
.def result = r27

.equ fclk = 1000000
.equ displayon  = 0b10001001		; command 0x80 + display on = 8 + brightness 1 =0x89
.equ displayoff = 0b10000001
.equ address1 = 0xC2
.equ address2 = 0xC1
.equ address3 = 0xC0
.equ address4 = 0xC5
.equ address5 = 0xC4
.equ address6 = 0xC3

.equ search_rom = 0xf0
.equ read_rom = 0x33
.equ match_rom = 0x55
.equ skip_rom = 0xcc
.equ alarm_search = 0xec
.equ convert = 0x44
.equ write_scratchpad = 0x4e
.equ read_sscratchpad = 0xbe
.equ copy_scratchpad = 0x48
.equ recall_E2 = 0xb8
.equ read_powersupply = 0xb4

.macro printf						;macro to print characters on OLED
ldi temp,@0							
mov ASCII,temp
rcall print
.endm 


.macro DALLAS_COMMAND				; macro to send rom commands to sensor
ldi temp1,@0
rcall ROM_COMMANDS
.endm

.macro F_COMMANDS					; macro to send function commands to sensor
ldi temp1,@0
rcall f_command
.endm

.macro micros						; macro for micro seconds delay
ldi r26,((@0-10)/3)
rcall delayus
.endm

.macro command2
ldi data,@0
rcall spitx
.endm


.dseg
PAD:		.byte 1				; temp space for calculation
presence:	.byte 1				; stores response of ds18b20 during init
minussign:	.byte 1				; stores minus sign if result is negative else store space for positive result
firstdigit: .byte 1				; 100th position of result whole number
seconddigit:.byte 1				; 10th position of result whole number
thirddigit: .byte 1				; unit position of result whole number
firstplace: .byte 1				; stores ascii value of 1st fraction
secplace:	.byte 1				; stores ascii value of 2nd fraction
thirdplace: .byte 1				; stores ascii value of 3rd fraction
fourthplace:.byte 1				; stores ascii value of 4th fraction
SCRATCHPAD: .byte 2				; stores temprature high byte & low byte value copied from ds18b20sensor


.cseg
reset:
	ldi temp,0b00000101			; value to initialise portb and direction register for dataa and clock , pb0 , pb1
	out ddrb,temp				; initialize PB0 and PB2 as outputs (for TM1637 display)
	out portb,temp				; set both PB0 and PB2 ports high

main_loop:
	rcall measure				;call subroutine to measure temprature (read sensor,convert to decimal,convert to ASCII,store in SRAM)
	ldi r16, 'C'				;the last 3 fraction values are not printed,instead we use the space to print letter C, -100.1C
	sts secplace,r16			;the last 3 fraction values are not printed,instead we use the space to print letter C, -100.1C
printtemp:
	ldi YL,low(minussign)		; set Y pointer to location minussign in SRAM , all data to be displayed stored in SRAM consicutive locations
	ldi YH,high(minussign)
	ldi r25,6					; total 6 characters are displayed , so counter value 6
	rcall command1				; transmits command1 which tells TM1637 that fixed address mode is being used to send data that follows
	command2 address1			; transmit the 1st address of the display (here its the left most digit of the unit )
	ld ASCII,Y+					; ASCII (r21) used for address calculation of fonts
	rcall printchar				; call printchar function to calculate address of the font and transmit data to TM1637 display
	dec r25						; decrease counter , value used to print decimal at required location inside printchar routine
	command2 address2			; transmit the 2st address of the display
	ld ASCII,Y+
	rcall printchar
	dec r25
	command2 address3			; transmit the 3rd address of the display
	ld ASCII,Y+
	rcall printchar
	dec r25
	command2 address4			; transmit the 4th address of the display
	ld ASCII,Y+
	rcall printchar
	dec r25
	command2 address5			; transmit the 5th address of the display
	ld ASCII,Y+
	rcall printchar
	dec r25
	command2 address6			; transmit the 6th address of the display, last address as this is 6digit display , rightmost digit
	ld ASCII,Y+
	rcall printchar
	dec r25
	rcall command3				; transmit the display command which turns on the display
	rjmp main_loop				; loop again infinitely till power available




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;TM1637 ROUTINES     register used-                                                                 ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

command1:
	ldi data,0x44		;0x40 commannd for automatic address increment,0x44 for fixed address mode (address has to be supplied for each digit)
	rcall spitx
	rcall stop			;a stop has to be issued as per data sheet for data command setting (0x40 or 0x44 above)
	ret
command3:
	ldi data,displayon	;(load display command) 0x80 OR 0x01(brightness value bit0 to bit2,we use 1) OR 0x08(display on) = 0x89
	rcall spitx
	rcall stop
	ret

start:
	sbi portb,0			;set data pb0
	sbi portb,2			;set clk  pb2
	rcall us10
	cbi portb,0
	rcall us10
	cbi portb,2
	rcall us10
	ret
stop:
	cbi portb,0
	cbi portb,2
	rcall us10
	sbi portb,2
	rcall us10
	sbi portb,0
	rcall us10
	ret
SPI:
	ldi r18,8
tx:
	lsr data			; shift right data to move lsb to carry , TM1637 needs lsb 1st
	brcs hi				; if crry set bit shifted is 1 branch to label hi to transmit 1
	cbi portb,0			; reach here if carry not set while right shift, pull data low to transmit 0
	rcall us10			; delay 10us for settling data line (tm1637 is little slow as per data sheet)
	sbi portb,2			; make clock hi 
	rcall us10			; delay 10us
	cbi portb,2			; make clock line low ,data aquisition finished , 0 bit clocked in to TM1637
	rcall us10			; delay 10us
	dec r18				; decrease counter 
	brne tx				; check all 8 bits are transmitted else loop to label TX
	sbi portb,0			; if all 8 bits are transmitted , make data high ,  data line will be pulled down by TM1637 to transmit ACK
	sbi portb,2			; make clock high , clock in ACK (9th clock) so that TM1637 releases the data line from low
	rcall us10			; delay 10 us
	cbi portb,0			; pull data low , default state after start is issued
	cbi portb,2			; make clock low , 9th clock completed
	ret					; return to caller
hi:
	sbi portb,0			; reach here if carry  while right shift, pull data up to transmit 1
	rcall us10			; delay 10us
	sbi portb,2			; make clock hi
	rcall us10			; delay 10us
	cbi portb,2			; make clock low
	rcall us10			; delay 10us
	dec r18				; decrease counter
	brne tx				; check all 8 bits are transmitted else loop to label TX
	sbi portb,0			; if all 8 bits are transmitted , make data high ,  data line will be pulled down by TM1637 to transmit ACK
	sbi portb,2			; make clock high , clock in ACK (9th clock) so that TM1637 releases the data line from low
	rcall us10			; delay 10 us
	cbi portb,0			; pull data low , default state after start is issued		
	cbi portb,2			; make clock low , 9th clock completed
	ret					; return to caller
	
spitx:
	rcall start
	rcall SPI
	ret
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;PRINTCHAR function   
;characters to be printed has to be passed into register ASCII as ASCII values, only 0,1,2,3,4,5,6,7,8,9,-,., ,C are supported
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
printchar:	
	cpi ASCII,'-'						; minus
	breq minus1							; branch to label minus1
	cpi ASCII,'C'						; ascii C
	breq letterC						; branch to label letterC
	cpi ASCII,' '						; space
	breq space							; branch to lbel space
	rjmp print							; if none of above jump to label print which loads the first font '0'and calculates the relative position

minus1:
	ldi ZL,low((2*fontminus)+0x4000)	;attiny10 doesnt allow direct flash read,should access via SRAM address starting at 0x4000 as per data sheet
	ldi ZH,high((2*fontminus)+0x4000)	;pointer set to font array for - minus
	rjmp ASCII0

letterC:
	ldi ZL,low((2*fontC)+0x4000)		;attiny10 doesnt allow direct flash read,should access via SRAM address starting at 0x4000 as per data sheet
	ldi ZH,high((2*fontC)+0x4000)		;pointer set to font array for letter C
	rjmp ASCII0

space:
	ldi ZL,low((2*fontspace)+0x4000)	;attiny10 doesnt allow direct flash read,should access via SRAM address starting at 0x4000 as per data sheet
	ldi ZH,high((2*fontspace)+0x4000)	;pointer set to font array for space
	rjmp ASCII0


print:
	ldi ZL,low((2*font0)+0x4000)	;attiny10 doesnt allow direct flash read,should access via SRAM address starting at 0x4000 as per data sheet
	ldi ZH,high((2*font0)+0x4000)
	ldi array_byte,0x30   		;hex value of first ASCII caharacter in font aray is 0x30 .subtract start value from ASCII value returned from ASCII_CONVERTER to know the position of character in the array
	sub ASCII,array_byte		;result of subtraction will be position of first byte of the character
	breq ASCII0					;if the result is 0 go to label ASCII0 and print 6 bytes which is a space (0x20 = space in the font array,1st character)
	clr temp					;clear temp for counting
multiply:
	subi r30,low(-2)			;adding immediate not suppported. immediate extends beyond 8bit.using subi & sbci with -ve number will do & loading with hi &lo will propogate carry.
	sbci r31,high(-2)
	inc temp					;counter increased on each iteration
	cp temp,ASCII				;counter is compared to value in ASCII register(start address of the array row)
	brne multiply				;if the atart address not reached loop back through the code from label "multiply"
ASCII0:
	ld data,Z					; call routine font_write which prints font on LCD
	cpi r25,3					; this number decides where the decimal point will be placed , 3 means decimal will be placed on 4th digit from left most digit
	breq adddecimal				; if equal to 3 branch to adddecimal label to OR with 0x80 which will turn on the decimal point
	rcall spitx					; transmit data to TM1637 screen
	rcall stop					; transmit stop to screen
	ret
adddecimal:
	ori data,0x80				; OR 0x80 to the value in data to light up the decimal point, 0x80 is the decimal point segment data
	rcall spitx					; transmit data to TM1637 screen
	rcall stop					;transmit stop to screen
	ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;FONTS   fonts below 16bytes ,hence array lenth =16
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

// XGFEDCBA
font0: .db   0b00111111    // 0
font1: .db   0b00000110    // 1
font2: .db   0b01011011    // 2
font3: .db   0b01001111    // 3
font4: .db   0b01100110    // 4
font5: .db   0b01101101    // 5
font6: .db   0b01111101    // 6
font7: .db   0b00000111    // 7
font8: .db   0b01111111    // 8
font9: .db   0b01101111    // 9
fonta: .db   0b01110111    // A
fontb: .db   0b01111100    // b
fontc: .db   0b00111001    // C
fontd: .db   0b01011110    // d
fonte: .db   0b01111001    // E
fontf: .db   0b01110001    // F
fontminus: .db  0b01000000    // -
fontspace: .db  0b00000000
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


calc_fraction:
	clr r31					; clear r31 used for carry propagation in multibyte addition
	ldi r20,low(625)		; load r20 with 625 (scaled value of 0.0625 degC), we multiply with 625 and place decimal point in the LCD between fraction and main
	ldi r21,high(625)		; load r20 with 625 (scaled value of 0.0625 degC)
	mov r22,r20				; copy the 625 so that it can be added to itself , total added value in R23:r22
	mov r23,r21				; copy the 625 so that it can be added to itself , total added value in R23:r22
	cpi r19,1				; check if fraction value of the result copied to r19 in binary_decimal subroutine is equal to 1
	breq getout				; if 1 branch to label getout as no multiplication is needed and least value of 0.0625 is already in regiser
	cpi r19,0				; check if value of fraction in r19 is 0 , if 0 no multiplication needed just clear registers and exit
	breq clrnexit			; if 0 branch to label clrnexit 
	dec r19
fmultiply:
	add r22,r20				; add 625 to 625
	adc r23,r21				; add with carry
	adc r24,r31				; add with carry
	dec r19					; decrease r19 after each addition
	brne fmultiply			; if r19 not reached 0 loop back to fmultiply label till all additions have finished
getout:
	clr r20					; clear registers 625 is in r22, we reach here if r19 was 1 = 0.0625 * 1
	clr r21					; clear registers 625 is in r22, we reach here if r19 was 1 = 0.0625 * 1
	ret						; result in r24:r23:R22
clrnexit:
	clr r23					; clear register as there is no fraction , we reach here if r19 was 0 = no fraction, result whole number
	clr r22					; clear register as there is no fraction , we reach here if r19 was 0 = no fraction, result whole number
	ret						; result in r24:r23:R22

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;f_command used for sending function command to sensor, use F_COMMANDS macro
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
f_command:
	ldi r25,8					; counter value of 8 for 8 bits
	mov r24,r22					; macro F_COMMANDS loads value of the command to r22 which is now copied to r24
loop51:
	lsr r24						; shift right r24 to carry in SREG
	brcs high2					; branch to hihg2 label if carray flag set,  means shifted bit is 1 
	rcall write0				; call routine write0 to send 0 to sensor, reach here because carry flag didnt set during right shift means bit 0
	dec r25						; decrease counter		
	brne loop51					; do same for each bit until counter is 0, all bits transmitted
	ret							; return to caller
high2:
	rcall write1				; reach here if right shift set carry flag (bit is 1)
	dec r25						; decrease counter
	brne loop51					; loop back till all bits are shifted out
	ret							; return to caller
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;ROM_COMMANDS sends rom commands to sensor, use macro DALLAS_COMMAND
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
ROM_COMMANDS:
	ldi r25,8					; counter value of 8 for 8 bits
	rcall dallas_init			; call sensor init to reset sensor
	lds r16,presence			; copy sensor presense response value from SRAM stored during init routine
	cpi r16,0					; check whether it is 0 (sensor present)
	brne exit0					; any value not 0 indicates sensor missing so exit
	mov r24,r22					; macro DALLAS_COMMANDS loads value of the command to r22 which is now copied to r24
loop41:
	lsr r24						; shift right r24 to carry in SREG
	brcs high1					; branch to hihg2 label if carray flag set,  means shifted bit is 1
	rcall write0				; call routine write0 to send 0 to sensor, reach here because carry flag didnt set during right shift means bit 0
	dec r25						; decrease counter
	brne loop41					; do same for each bit until counter is 0, all bits transmitted
	ret							; return to caller
high1:
	rcall write1				; reach here if right shift set carry flag (bit is 1)
	dec r25						; decrease counter
	brne loop41					; do same for each bit until counter is 0, all bits transmitted
exit0:	ret						;return to caller
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

measure:
	rcall dallas_init				; call sensor init
	DALLAS_COMMAND skip_rom			; send rom command skip_rom with DALLAS_COMMAND macro
	F_COMMANDS convert				; send function command convert with macro F_COMMANDS

	
check1:
	rcall bit_read					; call routine bit_read to check busy status of sensor 1 = busy , 0= result ready
	sbrs r24,7						; skip next command if read value is 1
	rjmp check1						; loop to check1 if read value is 0 = busy
	rcall dallas_init				; call sensor init to reset sensor
	nop								; delay
	nop								; delay
	DALLAS_COMMAND skip_rom			; send rom command skip_rom with DALLAS_COMMAND macro
	F_COMMANDS read_sscratchpad		; send function command read_scratchpad with macro F_COMMANDS
	rcall byte_read					; call byte_read routine to read sensor scratch pad data (total 8 bytes but we are interested in 1st 2 temprature values)
	sts SCRATCHPAD + 1, r24			; store shifted in lsb to sram 
	rcall byte_read					; call byte_read routine to read sensor scratch pad data (total 8 bytes but we are interested in 1st 2 temprature values)
	sts SCRATCHPAD , r24			; store shifted in lsb to sram
	rcall dallas_init				; call sensor init to reset sensor and stop transmission
	rcall binary_decimal			; call binary_decimal routine to convert the result in binary to decimal structure
	ret								; return to caller
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
delayus:
	dec r26							; call & ret = 9cs, dec =1 ,brne =2 
	brne delayus
	ret
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; DS18b0 init/reset routine
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
dallas_init:
	sbi ddrb,ddb1					;porta 2 is output by setting direction register
	micros 480						;minimum time pull down time required for reset
	cbi ddrb,ddb1					;change PA2 to input to verify whether DS1B20 pulls the line low to indicate presence
	micros 70						;sensor should respond after 60 seconds ,so wait for the minimum time
	in r16,PINB						;copy the PA2 PIN value from PORTA_PIN register
	andi r16,0b00000010				;AND with 0x04 the value of r16 ,if 0 sensor responded , if 1 no response at the time of checking
	sts presence,r16				;copy value to memory
	micros 410						; wait for the remaining time slot of 460 us
	nop
	ret
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;write1 - routine to write 1 to DS18B20
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
write1:
	sbi ddrb,ddb1					;porta 2 is output by setting direction register
	nop								;delay of 3us data line stays low for more than minimum required time
	nop
	nop
	cbi ddrb,ddb1					;data line released by clearing direction register bit. line is pulled up by pullup voltage
	micros 60						;The delay of 60us to finish the time slot as per data sheet
	ret
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;write0 - routine to write 0 to DS18B20
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
write0:
	sbi ddrb,ddb1					;porta 2 is output by setting direction register
	micros 60						;delay 60us
	cbi ddrb,ddb1					;data line released by clearing direction register bit. line is pulled up by pullup voltage
	rcall us10
	ret
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;byte-read - routine to read a byte from DS18B20
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
byte_read:
	ldi r25,8						; counter value 8 for 8 bits of a byte
	clr r24							; byte is assembled here bit by bit
loopread:
	sbi ddrb,ddb1					;porta 2 is output by setting direction register
	nop								;delay of minimum 1 us as per data sheet
	nop
	nop
	cbi ddrb,ddb1					;clear direction register to release data line and turn the master to input
	nop
	nop
	nop
	nop
	nop
	nop
	sbic pinb,pinb1					;skip next instruction (one = reading a 1) if pinb register 1 value is 0
	rjmp one						;if 1 branch to label one
	clc								;clear carry bit in SREG
	ror r24							;rotate right through carry (0 is shifted into msb of r20 from carry,ds18B20 transmits lsb of LSBYTE firt.At the end the first bit in MSB reaches LSB
	micros 50						;wait for the remaining time slot of the read bit
	dec r25							;decrease counter
	brne loopread					;if counter not 0 branch back to loop3
	ret								;return to caller
one:
	sec								;set carry bit in SREG
	ror r24							;rotate through carry and the carry bit1 will be shifted to MSB of r20 , bit7 >>>>>>> bit0
	micros 50						;wait for the remaining time slot of the read bit
	dec r25							;decrease counter	
	brne loopread					;if counter not 0 branch back to loop3
	ret								;return to caller
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;bit_read - reads a bit from 18B20 , mainly used to check sensor is busy or not 1= ready , 0 = busy
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
bit_read:
	ldi r25,1						;load counter value 1, we read 1 bit so value 1
	clr r24							;clear r24 where the bit is shifted in
	rcall loopread					;call routine loopread which is part of byte_read to shift in the transmitted bit from the sensor
	andi r24,0x80					;and r24 with 0x80 , 0 is bit value = 0 , 80h is bit value = 1
	ret								;return to caller
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;us10 - routine to provide 10us delay
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
us10:
	ret
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;binary_decimal   converts binary to decimal
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

binary_decimal:
	lds r17,SCRATCHPAD + 1			; copy temprature lsb value from SRAM
	lds r18,SCRATCHPAD				; copy temprature msb value from sram
	sbrc r18,7						; check whether msb of high byte is cleared means positive number else negative number, if positive skip next instruction
	rjmp negativeresult				; jump to label negativeresult for negative number clculation		
	ldi r16,0x20					; load ascii space
	sts minussign,r16				; store in sram register minussign to clear the leading minus sign as result is positive
	mov r19,r17						; copy low byte of temprature to r19 to process fraction , lower nibble is fraction as per ds18b20 data sheet
	andi r19,0x0f					; isolate the lower nibble by anding with 80h
	rcall calc_fraction				; call calc_fraction subroutine, result is stored in SRAM from firstplace to fourth place in the called routine
	rjmp maindigits					; jump to routine maindigits to process the whole number part
negativeresult:
	clr r19							; clear r19
	lds r17,SCRATCHPAD + 1			; copy temprature lsb value from SRAM
	lds r18,SCRATCHPAD				; copy temprature msb value from sram
	com r18							; take complement
	com r17							; take complement (converting 2çomplement )
	subi r17,-1						; add 1 to get the original positive number represented by 2's complement
	mov r19,r17						; copy lsbyte of temprature to r19 to process fraction
	sts  SCRATCHPAD + 1,r17			; converted value fron 2's complement is stored back into SRAM for future use in maindigit processing
	sts SCRATCHPAD ,r18				; converted value fron 2's complement is stored back into SRAM for future use in maindigit processing
	andi r19,0x0f					; isolate the lower nibble by anding with 80h
	ldi r16,'-'						; load ascii minus sign in r16
	sts minussign,r16				; store minus sign in SRAM, this will print leading minus in the final OLED display
	rcall calc_fraction				; call calc_fraction subroutine, result is stored in SRAM from firstplace to fourth place in the called routine
maindigits:
	mov r25,r22						; copy fmultiply results to r26:r25 for ASCII conversion
	mov r26,r23						; copy fmultiply results to r26:r25 for ASCII conversion
	ldi ZL,low(firstplace)			; set Z pointer to SRAM firstplace where first portion of the fraction number will be stored
	ldi ZH,high(firstplace)
	ldi r20,4						; length of string 4 as we dont want space loaded at 4th position if leading zero in decimal fraction
	rcall ASCII_CONVERT				; call ASCII_CONVERT routine that will convert the fraction to ascii values
	lds r17,SCRATCHPAD + 1			; copy temprature lsb value from SRAM
	lds r18,SCRATCHPAD				; copy temprature msb value from sram
	lsr r18
	ror r17
	lsr r18
	ror r17
	lsr r18
	ror r17
	lsr r18
	ror r17							; shift 4 places right r18 & r17 (temp result high & low) to discard the frction value 
	mov r25,r17						; lsbyte of temprature is copied to r25 (ascii_convert input register)
	mov r26,r18					    ; msbyte of temprature is copied to r26	(ascii_convert input register)	
	ldi ZL,low(firstdigit)			; Z pointer set to SRAM firstdigit where the ascii value of 100th position will be stored by ascii_convert routine
	ldi ZH,high(firstdigit)
	ldi r20,3						; counter value for Z pointer , 100th, 10th and unit value
	rcall ASCII_CONVERT				; call ASCII_CONVERT routine that will convert the whole numbers to ascii values 
	ret								; return to caller

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;* Integer to ASCII converter subrountine here ,
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

ascii_convert:                         ;16 bit entry
				
                clr     r19            ;digit counter 
                inc     r19            ;---- decimal string generating 
                clr     r27            ;var1 /= 10; temp0
                ldi     r28,16         ; temp1
                lsl     r25            ;var0 (binary low)
                rol     r26            ;var1 (binary high)
                rol     r27            ;temp0
                cpi     r27,10         ;temp0
                brcs    PC+3           ;
                subi    r27,10         ;temp0
                inc     r25            ;var0
                dec     r28            ;temp1
                brne    PC-8           ;/
                subi    r27,-'0'       ;Push the remainder (a decimal digit when added with 0x30 ASCII)
                push    r27            ;/temp0
                cp      r25,r28		   ;if(var =! 0) var0,temp1
                cpc     r26,r28        ; continue digit loop;var1,temp1
                brne    PC-16          ;/
                cp      r19,r20        ;Adjust string length (this can be removed for auto-length)temp2,len
                brcc    PC+5           ;
                inc     r19            ; temp2
                ldi     r25,' '        ;var0
                push    r25            ;var0
                rjmp    PC-5           ;/
                pop     r25            ;Put decimal string var0
                rcall  xmit            ;<-- Put a char (var0) to memory, console or any display device
                dec     r19            ;temp2
                brne    PC-3           ;/
				clr r20				   ; clear length for next call
                ret					   ; return to caller
xmit:
	st Z+,r25						   ; store to SRAM value in r25 , Z pointer set inside binary_decimal routine
	ret								   ; return to caller

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;





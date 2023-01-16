;
; 804thermometerKTYPE.asm
;
; Created: 06/01/2023 16:43:04
; Author : Manama
; PA1 = MOSI
; PA2 = MISO
; PA3 = SCK
; PA4 = SS
; PB0  = I2C clock (SCK)
; PB1  = I2C data  (SDA)

.equ ins_cmd = 0x00
.equ data_cmd = 0x40
.equ OLED_address = 0x78
.equ read_data_len = 4
.equ fclk = 10000000 
.equ OLED_INIT_LEN = 15    ; 14 if screen flip is needed , 12 if normal screen
.equ font_length = 5
.equ fontsize = 16         ;

.def posx = r23
.def posy = r24
.def temp = r16
.def SLAVE_REG = r17
.def address = r18

.macro cursor
ldi posy,@0
ldi posx,@1
rcall set_cursor
.endm 

.macro printf
ldi temp,@0
mov ASCII,temp
rcall print
.endm 

.macro millis					; macro for delay in ms
ldi YL,low(@0)
ldi YH,high(@0)
rcall delayYx1mS
.endm

.macro micros					; macro for delay in us
ldi temp,@0
rcall delayTx1uS
.endm

.dseg

data1: .byte 1
data2: .byte 1
data3: .byte 1
data4: .byte 1
minussign:	.byte 1			; stores minus sign if result is negative else store space for positive result
OLdigit: .byte 1			; 100th position of result whole number
TTdigit:.byte 1				; 10th position of result whole number
Tdigit: .byte 1				; unit position of result whole number
Hdigit: .byte 1				; stores ascii value of 1st fraction
point: .byte 1				; stores a ascii decimal point to be displayed
tendigit:	.byte 1			; stores ascii value of 2nd fraction
ONE: .byte 1				; stores ascii value of 3rd fraction
SPA: .byte 1				; stores ascii space to be displayed between numeric value and C
CEE: .byte 1				; stores ascii C to be displayed






.cseg
.org 0x00

reset:
		
		ldi r16,'.'				; load decimal point in r16
		sts point,r16			; store decimal point in SRAM location point
		ldi r16,'C'				; load font "C"in r16
		sts CEE,r16				; store in SRAM for later printing on OLED
		ldi r16,0x20			; load 0x20 or space in r16
		sts SPA,r16				; store in SRAM for printing on OLED
		rcall MHZ10				; increase speed of processor
		ldi r16,0b00011010		; PA4,PA3,PA1 are outputs- SS,SCK & MOSI respectievly
		out VPORTA_DIR,r16		; enable out put and input
		sbi VPORTA_OUT,4		; SS HI ,set BIT4, PA4 =SS
		rcall TWI_INIT			; call subroutine to initialize TWI
		rcall oledinit			; call subroutine to initialize OLED SSD1302
		rcall CLEAR_OLED		; call subroutine to clear 128 x 32 OLED
		
read:
		rcall MAXREAD			; call subroutine to read MAX6657
		mov r16,r19				; copy low byte of sensor data to r16
		andi r16,0b00000100		; check bit 2 is low , if high thermocouple open
		brne sensor_error		; if bit 1 is set branch to sensor_error label	
		lsr r20					; logical shift right high register r20 msb
		ror r19					; rotate right through carry low register r19
		lsr r20					; temp value is held in d14-d3 bits
		ror r19					; we shift the r20:r19 to right 3 times 
		lsr r20					; the final 12 bit value from lsb will be temprature data
		ror r19					; the data is shifted 3 times right in both r20:19,D14–D3  becomes D11-D0
		
		rcall TEMP_CALC			; converted temprature result in r9:r8:r7
		rcall ascii_convert		; temprature values in ASCII format stored in SRAM OLdigit to ONE , 7 spots
		rcall zero_suppress		; call routine that supress all leading zeros in the OLED display
XYX:
		cursor 0,0				; set OLED cursor to coordinates 0,0
		ldi r16,9				; load r16 , 9
		mov r10,r16				; move 9 to r10. this serves as counter for SRAM location 
		ldi YL,low(OLdigit)		; load address of first digit OLdigit in SRAM
		ldi YH,high(OLdigit)
printloop:
		ld r16,Y+				; load r16 with data in address pointed by Y register
		rcall printchar			; call sub routine printchar which prints ascii chars stored in SRAM 
		dec r10					; decrease SRAM location counter
		brne printloop			; loop till counter r10 is 0
		millis 500				; 500ms delay
		rjmp read				; jump to label read to repeat


sensor_error:
		ldi r16,'-'				; load minus sign in r16
		sts OLdigit,r16			; store in sram
		sts TTdigit,r16			; store in sram
		sts Tdigit,r16			; store in sram
		sts Hdigit,r16			; store in sram
		sts tendigit,r16		; store in sram
		sts ONE,r16				; store in sram
		rjmp XYX

MHZ10:							; increases attiny 804 processor speed from default to 10MHZ
		ldi r16,0Xd8
		out CPU_CCP,r16
		ldi r16,0x01			; clk prescaler of 2, 20Mhz/2 = 10Mhz
		STS CLKCTRL_MCLKCTRLB,R16
		ret

MAXREAD:					; subroutine to read MAX6675 data
	ldi r25,16				; load r25 with 16, 16 clock cycles to spit out 16bit value
	rcall SS_LO				; call subroutine to pull SS pin lo
rloop:
	rcall SCK_HI			; call routine clock pin high
	rcall us10				; delay 10 micro seconds
	clc						; clear carry flag
	sbic VPORTA_IN,2		; if MISO pin is low skip next instruction
	sec						; set carry flag
	rol r19					; rotate through carray left r19, carry is loaded into lsb of r19
	rol r20					; rotate through carray left r20, the bit shifted out of r19 is looped through r20
	rcall SCK_LO			; call routine to pull clock pin low
	dec r25					; decrease counter
	brne rloop				; loop till r25 is 0 , 16 clocks
	rcall SS_HI				; pull SS high once 16 bits are clocked out
	ret


SCK_LO:	cbi VPORTA_OUT,3		; SCK low ,clear bit3 ,PA3 
		ret

SCK_HI:	sbi VPORTA_OUT,3		; SCK HI ,set BIT3, PA3 
		ret

SS_LO:	cbi VPORTA_OUT,4		; SS low ,clear bit4 ,PA4 =SS
		ret

SS_HI:	sbi VPORTA_OUT,4		; SS HI ,set BIT4, PA4 =SS
		ret



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;I2C ROUTINES
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

TWI_INIT:
		ldi r16,40
		sts TWI0_MBAUD,R16
		LDI R16,0b00000011			;SMEN,ENABLE
		STS TWI0_MCTRLA,R16
		LDI R16,0b00001000			;FLUSH ADDR & DATA REGISTERS
		STS TWI0_MCTRLB,R16
		LDI R16,0b00000001			;FORCE IDLE
		STS TWI0_MSTATUS,R16
		ret
		


TWI_START:
		MOV R16,SLAVE_REG			;SLAVE_REG IS R17, READ OR WRITE ADDRESS SHOULD BE LOADED HERE PRIOR TO CALL
		STS TWI0_MADDR,R16
		RCALL WAIT_WIF
		SBRC R16,4				;SKIP NEXT INSTRUCTION IF RXACK IS SET
		RCALL TWI_STOP
		RET

TWI_WRITE:

		MOV R16,SLAVE_REG
		STS TWI0_MDATA,R16
		RCALL WAIT_wIF
		SBRC R16,4
		RCALL TWI_STOP
		RET

TWI_READ:
		MOV R16,SLAVE_REG			; SLAVE_REG IS R17,  WRITE ADDRESS SHOULD BE LOADED HERE PRIOR TO CALL
		STS TWI0_MADDR,R16
		RCALL WAIT_WIF
		MOV R16,SLAVE_REG			; send instruction/READ_ADDRESS to THE SLAVE FROM WHICH DATA IS READ
		STS TWI0_MDATA,R16
		RCALL WAIT_WIF
		ldi r16,0x00				;loading 0 in ACKACT bit enables master to send ack after reading data register
		sts TWI0_MCTRLB,r16

		MOV R16,SLAVE_REG			; repeated start ; READ ADDRESS SHOULD BE LOADED HERE FOR READING DATA FROM SLAVE READ_ADDRESS GIVEN ABOVE
		STS TWI0_MADDR,R16			; THIS IS A REPEATED START
		RCALL WAIT_RIF				; once data arrives in the data register the read flag is set

		ldi r16,read_data_len			;load r16 with number of bytes to be read
		cpi r16,0x02				;is num of bytes less than or greater than 2
		brlo BYYTE				;if less than 2 branch to 1BYTE as NACK+STOP will be loaded prior to read
		dec r16					; decreace one count from the total count to get loop value,NACK should be sent before the last byte read
		mov r5,r16				; move the count -1 value to counter r5
loop_read:
		LDS R16,TWI0_MDATA		;MDATA REGISTER IS COPIED TO R16,DATA IS RECIVED INTO MDATA FROM SLAVE
		ST Z+,R16				;DATA IN R16 IS STORED IN SRAM BUFFER FOR LATER USE. 
		RCALL WAIT_RIF			;wait for read flag
		dec r5					;decrease counter after each read
		brne loop_read			;go throug loop till {count - 1} is finished
BYYTE: 
		LDI R16,0b00000111		;CLEAR ACKACT BIT BEFORE READING LAST BYTE AND ISSUE A STOP = NACK+STOP
		STS TWI0_MCTRLB,R16
		LDS R16,TWI0_MDATA		;MDATA REGISTER IS COPIED TO R16,THIS THE LAST DATA IS RECEIVED  FROM SLAVE
		ST Z+ ,R16				;DATA IN R16 IS STORED IN SRAM BUFFER FOR LATER USE. 
		RET


TWI_STOP:
		LDI R16,0b00000011                       ;STOP
		STS TWI0_MCTRLB,R16
		RET


WAIT_WIF:
		LDS R16,TWI0_MSTATUS
		SBRS R16,6				;CHECK WIF IS SET,IF SET SKIP NEXT INSTRUCTION
		RJMP WAIT_WIF
		RET


WAIT_RIF:
		LDS R16,TWI0_MSTATUS
		SBRS R16,7
		RJMP WAIT_RIF
		RET


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
OLED_INIT_BYTES:
.DB 0xA8,0x1f,0x20,0x01,0x21,0x00,0x7F,0x22,0x00,0x03,0xDA,0x02,0x8D,0x14,0xAF,0xA1,0xC8
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;	
;  0xA8, 0x1F,       // set multiplex (HEIGHT-1): 0x1F for 128x32, 0x3F for 128x64 ; 
;  0x22, 0x00, 0x03, // set min and max page									   ;
;  0x20, 0x01,       // set vertical memory addressing mode	
;  0x21, 0x00,0x7F   // start column 0 end column 127					   ;
;  0xDA, 0x02,       // set COM pins hardware configuration to sequential          ;
;  0x8D, 0x14,       // enable charge pump                                         ;
;  0xAF,             // switch on OLED                                             ;
;  0xA1, 0xC8        // flip the screen                                            ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

CLEAR_OLED:
	ldi r16,3				; number of pages , we write to all 4 pages 0,1,2,3
	mov r0,r16				; copy to r0 number of pages
	cursor 0,0				; call macro cursor with coordinates 0, 0
	rcall TX_address		; transmit OLED address
	rcall oled_data_write	; transmit data command 0x40 to OLED
cleardataOLED:				; this routine write 0 to oled memory 128 *4 times to clear screen.
	ldi r26,4				; row 4, row loop
time2:
	ldi r27,128				; 128 pixels
time1:
	ldi SLAVE_REG,0			; load 0 in slavereg to be transmitted to OLED and written in each pixel
	rcall TWI_WRITE			; transmit 0
	dec r27					; decrease pixel counter for each row in this loop
	brne time1				; loop till 128 pixels are written 0
	dec r26					; decrease row counter
	brne time2				; repeat till 128 x 4 is finished
	rcall TWI_STOP			; stop I2C transmission
	clr r0					; clear r0
	ret						; return to caller

TX_address:							;r18,r16,r17 registers are needed
	ldi	SLAVE_REG,OLED_address		; address of ssd1306
	rcall TWI_START					; send start condition on bus
	ret								; return to calller

set_cursor:
	rcall TX_address		; send OLED address
	ldi SLAVE_REG,0x00 		; control value for OLED command
	rcall TWI_WRITE			; transmit to OLED, following values will be OLED instructions	
	ldi SLAVE_REG,0x22		; Set row command
	rcall TWI_WRITE
	mov SLAVE_REG,posy		; cursor will start at row indicated by the number in this register defined as posy
	rcall TWI_WRITE
	ldi SLAVE_REG,0x3		; cursor feild upto row 4 (0,1,2,3) , 0=top row,3= last row
	rcall TWI_WRITE
	ldi SLAVE_REG, 0x21		; set column command
	rcall TWI_WRITE
	mov SLAVE_REG,posx		; cursor is placed on the selected column 0-127 passed as parameters in register defined as posX
	rcall TWI_WRITE
	ldi SLAVE_REG,0x7f		; last column pixel is 127 . 0-127 in a row
;	sbrs r0,0
;	subi posy,-3
;	or posy,r0
;	mov SLAVE_REG,posy		; end at page1/row1 (parameter)
	rcall TWI_WRITE
	rcall TWI_STOP			; stop I2C transmission
	ret						; return to caller


arrayread:					; routine to transmit an array pointed by Z to I2C
	lpm r27,Z+				; Z pointer pointing an array address prior to call this routine
	mov SLAVE_REG,r27
	rcall TWI_WRITE
	dec r22
	brne arrayread
	ret

oledinit:						; init routin for SSD3302 OLED 128x32
	rcall TX_address			;r18,r16,r17 registers are needed
	rcall oled_command_write	; send OLED command
	ldi ZL,low(2*OLED_INIT_BYTES)
	ldi ZH,high(2*OLED_INIT_BYTES)
	ldi r22,OLED_INIT_LEN		; init element counter
	rcall arrayread				; sends array element 1 by 1 to I2C
	rcall TWI_STOP				; stop transmission
	ret							; return to caller
	
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;I2C_COMMAND_WRITE  ;function to write a stream of commands to the OLED
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

oled_command_write:
	ldi SLAVE_REG,ins_cmd		;command value is 0x00
	rcall TWI_WRITE
	ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;I2C_DATA_WRITE	  ;function to write a stream of data to the OLED
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
oled_data_write:
	ldi SLAVE_REG,data_cmd		;data command is 0x40
	rcall TWI_WRITE
	ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;printchar ;;;prints characters to OLED
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

printchar:
	
	cpi r16,'.'		; decimal
	breq decimal
	cpi r16,'-'		; minus
	breq minus
	cpi r16,'C'		; C
	breq letterC
	cpi r16,' '		; space
	breq space
	rjmp printnum
decimal:
	ldi ZL,low(2*fontdecimal)
	ldi ZH,high(2*fontdecimal)
	rjmp load0
minus:
	ldi ZL,low(2*fontminus)
	ldi ZH,high(2*fontminus)
	rjmp load0

letterC:
	ldi ZL,low(2*fontC)
	ldi ZH,high(2*fontC)
	rjmp load0

space:
	ldi ZL,low(2*fontspace)
	ldi ZH,high(2*fontspace)
	rjmp load0

printnum:
	ldi ZL,low(2*font0)	; font0 address is loaded in Z register
	ldi ZH,high(2*font0)
	ldi r26,0x30   		; load 0x30 in r26, ascii value of 0. 1st font array is 0 with 16 elements
	sub r16,r26			; compare value in r16 to r26
	breq load0			; if subtracted is 0 value in r16 is ascii0 and we have already address of font0 pointed by Z
	clr r26				; if result of subtraction is not 0, we have an ascii value greater than 0 and have to find by multiply
multiply:
	subi r30,low(-16)	; add 16 to Z, which will make Z point to start of next font array
	sbci r31,high(-16)	; add 16 with carry to Z ,
	inc r26				; increase r26 from 0 upwards on each iteration
	cp r26,r16			; compare r26 to r16 , if r26 is 1 and compared equal to r26 then we have ascii 1
	brne multiply		; do above procedure untilwe have a match, when matched we have the Z pointer pointing to correct array
load0:
	rcall font_write	; call routine which takes each array element and calls double to stretch and print on OLED
	ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;font_write    routine loads font from font array and writes it to OLED
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	
font_write:						; R22,R27,r18,r16,r17 registers are needed
	rcall TX_address
	ldi SLAVE_REG,data_cmd		; control to send data to OLED
	rcall TWI_WRITE
	ldi r22,fontsize			; font size = 16 each font has 16 bytes
fontarrayread:
	lpm r27,Z+					; load from flash each array element to r27 and increment Z address
	rcall double				; call subroutine which takes a byte and stretches to word, 11110000 becomes 1111111100000000 and stored in r1 & r2, each byte is stretched to 16bits
	mov SLAVE_REG,r1			; load first stretched byte of the font array
	rcall TWI_WRITE				; I2C write to OLED
	mov SLAVE_REG,r2			; load high stretched byte of the font array, stretched high & low bytes make each font 32bits vertical (height) from 16 bit high font in the array
	rcall TWI_WRITE
;	mov SLAVE_REG,r5			;r5 contains 0x00 for vertical filling, with these lines the text is stretched horizontally 16 pixels , if this lines deleted horizontally 8 pixels
;	rcall TWI_WRITE
;	mov SLAVE_REG,r5			;r5 contains 0x00 for vertical filling, with these lines the text is stretched horizontally 16 pixels , if this lines deleted horizontally 8 pixels
;	rcall TWI_WRITE
;	mov SLAVE_REG,r1			;with these lines the text is stretched horizontally 16 pixels , if this lines deleted horizontally 8 pixels
;	rcall TWI_WRITE
;	mov SLAVE_REG,r2			;with these lines the text is stretched horizontally 16 pixels , if this lines deleted horizontally 8 pixels
;	rcall TWI_WRITE
	dec r22						;decrease array counter (total 16 bytes per letter/character)
	brne fontarrayread			; loop to fontarrayread label until all elements in the array are stretched and written to OLED
	rcall TWI_STOP				; stop I2C transmission for the current font/char
	ret							; return to caller


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;double  routine takes byte in r16 and stretches it to 16 bit 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


double:
	push r25			; save r25 to stack
	clr r1				; clear r1
	clr r2				; clear r2
	ldi r25,8			; counter value 8 loaded for a byte
	mov r16,r27			; copy byte from r27 which was loaded from font array by Z pointer
dloop:
	lsl r16				; shift left r16 , bit moved to carry
	brcs bit1			; if carry flag set branch to bit1 label which writes 2 ones
	rol r1				; if carry flag not set rotate left r1
	rol r2				; rotate left through carry r2
	rol r1				; rotate left through carry r1 we add a new 0 to the previous 0 as we are doubling/stretching the bit
	rol r2				; rotate left through carry r2 we add a new 0 to the previous 0 as we are doubling/stretching the bit, r2 have now 2 zeros
	dec r25				; decrease bit counter
	brne dloop			; loop back until all bits are done
	pop r25				; restore r25
	ret					; return to caller
bit1:
	rol r1				;reach here if carry flag set/bit is 1, rotate left through carry r1, the 1 in carry ends up in r1 lsb
	rol r2				; rotate through carry r2 for 16 bit 
	sec					; set carry bit to shift in one more 1 as we are stretching
	rol r1				; rotate left through carry r1 
	rol r2				; rotate left through carry r2
	dec r25				; decrease bit counter
	brne dloop			; loop back until all bits are done
	pop r25				; restore r25
	ret					; return to caller

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;FONTS   fonts below 5bytes ,assembler will add one byte of padding with 0. hence array lenth =6
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

font0:
.db 0x00,0x00, 0xE0,0x0f, 0x10,0x10, 0x08,0x20, 0x08,0x20, 0x10,0x10, 0xE0,0x0f, 0x00,0x00 //0
font1:
.DB 0x00,0x00, 0x10,0x20, 0x10,0x20, 0xF8,0x3F, 0x00,0x20, 0x00,0x20, 0x00,0x00, 0x00,0x00// 1 // 1
font2:
.db 0x00,0X00, 0x70,0X30, 0x08,0X28, 0x08,0X24, 0x08,0X22, 0x88,0X21, 0x70,0X30, 0x00,0X0 // 2
font3:
.DB 0x00,0X00, 0x30,0X18, 0x08,0X20, 0x88,0X20, 0x88,0X20, 0x48,0X11, 0x30,0X0E, 0x00,0X00 // 3
font4:
.DB 0x00,0X00, 0x00,0X07, 0xC0,0X04, 0x20,0X24, 0x10,0X24, 0xF8,0X3F, 0x00,0X24, 0x00,0X00 //0
font5:
.DB 0x00,0x00, 0xF8,0x19, 0x08,0x21, 0x88,0x20, 0x88,0x20, 0x08,0x11, 0x08,0x0e, 0x00,0x00 // 5
font6:
.DB 0x00,0x00, 0xE0,0x0f, 0x10,0x11, 0x88,0x20, 0x88,0x20, 0x18,0x11, 0x00,0x0e, 0x00,0x00 // 6 22
font7:
.DB 0x00,0x00, 0x38,0x00, 0x08,0x00, 0x08,0x3f, 0xC8,0x00, 0x38,0x00, 0x08,0x00, 0x00,0x00 // 7 23
font8:
.DB 0x00,0x00, 0x70,0x1c, 0x88,0x22, 0x08,0x21, 0x08,0x21, 0x88,0x22, 0x70,0x1c, 0x00,0x00//8 24
font9:
.DB 0x00,0x00, 0xE0,0x00, 0x10,0x31, 0x08,0x22, 0x08,0x22, 0x10,0x11, 0xE0,0x0f, 0x00,0x00 // 9 25
fontdecimal:
.db 0x00,0x00, 0x00,0x30, 0x00,0x30, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00// .
fontspace:
.db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
fontC:
.db 0xC0,0x07, 0x30,0x18, 0x08,0x20, 0x08,0x20, 0x08,0x20, 0x08,0x10, 0x38,0x08, 0x00,0x00// C
fontminus:
.db 0x00,0x00, 0x00,0x01, 0x00,0x01, 0x00,0x01, 0x00,0x01, 0x00,0x01, 0x00,0x01, 0x00,0x01// -                    
                    




	; ============================== Time Delay Subroutines =====================
; Name:     delayYx1mS
; Purpose:  provide a delay of (YH:YL) x 1 mS
; Entry:    (YH:YL) = delay data
; Exit:     no parameters
; Notes:    the 16-bit register provides for a delay of up to 65.535 Seconds
;           requires delay1mS

delayYx1mS:
    rcall    delay1mS                        ; delay for 1 mS
    sbiw    YH:YL, 1                        ; update the the delay counter
    brne    delayYx1mS                      ; counter is not zero

; arrive here when delay counter is zero (total delay period is finished)
    ret
; ---------------------------------------------------------------------------
; Name:     delayTx1mS
; Purpose:  provide a delay of (temp) x 1 mS
; Entry:    (temp) = delay data
; Exit:     no parameters
; Notes:    the 8-bit register provides for a delay of up to 255 mS
;           requires delay1mS

delayTx1mS:
    rcall    delay1mS                        ; delay for 1 mS
    dec     temp                            ; update the delay counter
    brne    delayTx1mS                      ; counter is not zero

; arrive here when delay counter is zero (total delay period is finished)
    ret

; ---------------------------------------------------------------------------
; Name:     delay1mS
; Purpose:  provide a delay of 1 mS
; Entry:    no parameters
; Exit:     no parameters
; Notes:    chews up fclk/1000 clock cycles (including the 'call')

delay1mS:
    push    YL                              ; [2] preserve registers
    push    YH                              ; [2]
    ldi     YL, low(((fclk/1000)-18)/4)     ; [1] delay counter              (((fclk/1000)-18)/4)
    ldi     YH, high(((fclk/1000)-18)/4)    ; [1]                            (((fclk/1000)-18)/4)

delay1mS_01:
    sbiw    YH:YL, 1                        ; [2] update the the delay counter
    brne    delay1mS_01                     ; [2] delay counter is not zero

; arrive here when delay counter is zero
    pop     YH                              ; [2] restore registers
    pop     YL                              ; [2]
    ret                                     ; [4]

; ---------------------------------------------------------------------------
; Name:     delayTx1uS
; Purpose:  provide a delay of (temp) x 1 uS with a 16 MHz clock frequency
; Entry:    (temp) = delay data
; Exit:     no parameters
; Notes:    the 8-bit register provides for a delay of up to 255 uS
;           requires delay1uS

delayTx1uS:
    rcall    delay10uS                        ; delay for 1 uS
    dec     temp                            ; decrement the delay counter
    brne    delayTx1uS                      ; counter is not zero

; arrive here when delay counter is zero (total delay period is finished)
    ret

; ---------------------------------------------------------------------------
; Name:     delay10uS
; Purpose:  provide a delay of 1 uS with a 16 MHz clock frequency ;MODIFIED TO PROVIDE 10us with 1200000cs chip by Sajeev
; Entry:    no parameters
; Exit:     no parameters
; Notes:    add another push/pop for 20 MHz clock frequency

delay10uS:
    ;push    temp                            ; [2] these instructions do nothing except consume clock cycles
    ;pop     temp                            ; [2]
    ;push    temp                            ; [2]
    ;pop     temp                            ; [2]
    ;ret                                     ; [4]
     nop
     nop
     nop
     ret

; ============================== End of Time Delay Subroutines ==============

us50:
	micros 5
	ret
	
us10:
	micros 1
	ret




TEMP_CALC:								;temprature value in r20:r19 , min 0 , max 0xfff (4095)  , example 0xfff
	ldi r16,255							;loadmax 8 bit value in r16
	mov r14,r16							;copy 255 to r14 for  subtraction later
	clr r15								;clear r15 for carry propagation while subtraction
	clr r13								;clear r13 to be used as counter= number of 255 in the result
	cpi r20,0							; if r20 is 0 the temprature reading is either 255 or less , here 0xfff
	breq direct							; direct multiplication
	clc									;clear carry flag in sreg
XX:	sub r19,r14							;subtract 255 from the result of 4095-temp val 
	sbc r20,r15							;subtract with carry
div_count:inc r13						;increase counter , holds value of how many 255 is there in the 12 bit number, example 4095/255 =16
	LDI R18,HIGH(255)					;16bit comparison
	CLC									;clear carry
	CPI R19,255							;16bit comparison with 255 for low byte
	CPC R20,R18							;16bit comparison with 255 for high byte
	BRGE XX								;after comparison if result is greater than 255 loop back to label XX till lessthan 255

direct:
	ldi r16,25							;each bit of MAX6675 is 0.25 degreeC, fixed point math , multiply with 25
	mul r19,r16							;multiply remainder of result/255 with 25  , example 15 is the remainder of 4095/255 =16
	mov r6,r1							;result copied from r1:r0 to R6:R5  , example  15 x25 = 777
	mov r5,r0
	ldi r17,255							;load r17 255 (0xff)
	mov r18,r13							; load r18 with number of 0xff in MAX6675 output (was in r13 above)
	mul r17,r18							; multiply 255 x number of 255 , example 255x16 = 4080
	mov r18,r1							; result of mul in r1:r0 copied to r18:r17 , example 4080
	mov r17,r0
	ldi r16,25							; load 25 for (12bit val) x  (25) , example 4080 x 25 = 102000
	mul r17,r16							; multiply lsb with 25 (16 bit wit 8 bit hardware multiplication)
	mov r7,r0							; result 0 , exmaple 102000 in r8:R7
	mov r8,r1							; result 1 , exmaple 102000 in r8:R7
	mul r18,r16							; mul high byte with 25(16 bit wit 8 bit hardware multiplication)
	mov r9,r1							; MSB of result copied to r9
	add r8,r0							; LSB of result is added to r8 , byte 2 of final result
	brcc NOINC							; if carry not set branch to NOINC
	inc r9								; if carry set increase r9 by 1
NOINC:
	add r7,r5							; add previous remainder converted to temprature to r7 
	adc r8,r6							; add with carry to r8
	ret

ascii_convert:
	ldi r16,0x30				; ascii 0
	sts OLdigit,r16				; store in SRAM
	sts TTdigit,r16				; store in SRAM
	sts Tdigit,r16				; store in SRAM
	sts Hdigit,r16				; store in SRAM
	sts tendigit,r16			; store in SRAM
	sts ONE,r16					; store in SRAM
ascii_loop:
	ldi r16,low(100000)			; compare value 100000
	mov r2,r16					; low byte of 100000 in r2
	ldi r16,byte2(100000)
	mov r3,r16					; byte2 of 100000 in r3
	ldi r16,byte3(100000)
	mov r4,r16					; byte3 of 100000 in r4
	cp  r7,r2					; compare result with low byte
	cpc r8,r3					; compare with carry with byte2
	cpc r9,r4					; compare with carry with byte3
	brge Dby100000				; if greater or equal than 100000 branch to divide by 100000
	ldi r16,low(10000)			; compare value 10000
	mov r2,r16					; low byte of 10000 in r2
	ldi r16,byte2(10000)
	mov r3,r16					; high byte of 10000 in r3
	cp  r7,r2					; compare result with 10000
	cpc r8,r3					; compare result with 10000
	brge Dby10000				; if greater or equal to 10000 branch to divide by 10000
	ldi r16,low(1000)			; load compare value 1000
	mov r2,r16					; low compare value 1000 in r2
	ldi r16,byte2(1000)
	mov r3,r16					; high of compare value 1000 in r3
	cp  r7,r2					; compare low byte of result with low of 1000
	cpc r8,r3					; compare with carry to high byte of 1000
	brge Dby1000				; if greater or equal to 1000 branch to divide by thousand
	ldi r16,low(100)			; load compare value low byte of 100
	mov r2,r16					; copy to r2
	ldi r16,byte2(100)			; load compare value high byte of 100
	mov r3,r16					; copy to r3
	cp  r7,r2					; compare low byte of result to low of 100
	cpc r8,r3					; compare with carry the result to high byte of 100
	brge Dby100					; if compared result is greater than or equal to 100 branch to divide by 100
	ldi r16,low(10)				; load ccompare value 10
	mov r2,r16					; load compare value in r2
	ldi r16,byte2(10)			; load high byte of 10
	mov r3,r16					; load compare value high byte
	cp  r7,r2					; compare result to 10
	cpc r8,r3					; compare wit carry if any
	brge Dby10					; if compared result is greater than or equal to 10 branch to divide by 100
	ldi r16,0x30				;  load ascii 0 in r16
	add r16,r7					; add ascii0 with remaining value in r7
	sts ONE,r16					; store the single digit ascii value in SRAM register ONE
	ret

Dby100000:
	clr r16
	sub r7,r2
	sbc r8,r3
	sbc r9,r4
	lds r16,OLdigit
	inc r16
	sts OLdigit,r16
	rjmp ascii_loop
Dby10000:
	sub r7,r2
	sbc r8,r3
	sbc r9,r4
	lds r16,TTdigit
	inc r16
	sts TTdigit,r16
	rjmp ascii_loop
Dby1000:
	sub r7,r2
	sbc r8,r3
	sbc r9,r4
	lds r16,Tdigit
	inc r16
	sts Tdigit,r16
	rjmp ascii_loop
Dby100:
	sub r7,r2
	sbc r8,r3
	sbc r9,r4
	lds r16,Hdigit
	inc r16
	sts Hdigit,r16
	rjmp ascii_loop
Dby10:
	sub r7,r2
	sbc r8,r3
	sbc r9,r4
	lds r16,tendigit
	inc r16
	sts tendigit,r16
	rjmp ascii_loop



zero_suppress:
	lds r16,OLdigit
	cpi r16,0x30
	breq blank
	ret
blank:
	ldi r16,' '
	sts OLdigit,r16
	lds r16,TTdigit
	cpi r16,0x30
	brne exit1
	ldi r16,' '
	sts TTdigit,r16
	lds r16,Tdigit
	cpi r16,0x30
	brne exit1
	ldi r16,' '
	sts Tdigit,r16
	
exit1:
	ret
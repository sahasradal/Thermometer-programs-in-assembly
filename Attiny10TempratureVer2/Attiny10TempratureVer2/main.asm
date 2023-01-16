;
; Attiny10TempratureVer2.asm
;
; Created: 20/04/2022 12:33:11
; Author : Manama
; TESTED and found ok , displays result in 32x8 font
; Display ssd1306 128x32 OLED 0.91" I2C enabled display from Aliexpress
;Segment   Begin    End      Code   Data   Used    Size   Use%
;		---------------------------------------------------------------
;		[.cseg] 0x000000 0x0003f4    772    240   1012    1024  98.8%
;		[.dseg] 0x000040 0x000060      0     13     13      32  40.6%


;pb0 is SDA , use pull-up 5k
;pb2 is SCL , use pull-up 5k
;pb1 is data line for temp sensor,  use pull-up 5k


.equ fclk = 1000000
.equ address = 0x78
.equ command = 0x00
.equ data_cmd = 0x40
.equ OLED_INIT_LEN = 15    
.equ font_length = 16
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

.macro printf
ldi temp,@0
mov ASCII,temp
rcall print
.endm 

.macro cursor
ldi posy,@0
ldi posx,@1
rcall set_cursor
.endm 

.macro DALLAS_COMMAND
ldi temp1,@0
rcall ROM_COMMANDS
.endm

.macro F_COMMANDS
ldi temp1,@0
rcall f_command
.endm

.macro micros
ldi r26,((@0-10)/3)
rcall delayus
.endm

.dseg
PAD: .byte 1
presence: .byte 1
minussign: .byte 1
firstdigit: .byte 1
seconddigit: .byte 1
thirddigit: .byte 1
decimal: .byte 1
firstplace: .byte 1
secplace: .byte 1
thirdplace: .byte 1
fourthplace: .byte 1
SCRATCHPAD: .byte 2


.cseg
reset:
	rcall i2c_command_write		;call 12c write subriutine
	rcall OLED_INIT				;call OLED initialization routine
	ldi array_length,OLED_INIT_LEN		;length of OLED init array 12 loaded
	rcall array_read			;call array_read subroutine to send OLED init to display via 12c
	rcall i2c_stop				;call 12c stop
	rcall clear_OLED			;call OLED screen clear routine. writes 0x00 in all OLED GDRAM address 128*32
;	cursor 0, 10				;cursor macro is called .2nd line 20th slot

main_loop:
	rcall measure				;call subroutine to measure temprature (read sensor,convert to decimal,convert to ASCII,store in SRAM)
	ldi r16,' '					;the last 3 fraction values are not printed,instead we use the space to print 2 spaces and letter C, -100.1  C
	sts secplace,r16			;the last 3 fraction values are not printed,instead we use the space to print 2 spaces and letter C, -100.1  C
	sts thirdplace,r16			;the last 3 fraction values are not printed,instead we use the space to print 2 spaces and letter C, -100.1  C
	ldi r16, 'C'				;the last 3 fraction values are not printed,instead we use the space to print 2 spaces and letter C, -100.1  C
	sts fourthplace,r16			;the last 3 fraction values are not printed,instead we use the space to print 2 spaces and letter C, -100.1  C
	ldi r25,9					;length of SRAM to be copied (max is 9), 6 means result will look -100.1 , if positive leading space
	ldi YL,low(minussign)
	ldi YH,high(minussign)
	rcall ccursor				;routine sets cursor to 0,0 coordinates
printtemp:
	ld ASCII,Y+					;load ASCII register with value from address (SRAM) pointed by Y (9 bytes starting fromaddress  minussign)
	rcall printchar				;call routine to calculate address,read font array, double size vertically (32x8) and print to OLED
	dec r25						;decrease address counter
	brne printtemp				;repeat if all 9 bytes are not read
	rjmp main_loop				;jump to main_loop to repeat 


OLED_INIT:
	ldi ZL,low((2*OLED_INIT_BYTES)+0x4000)		;load low value of OLED_INIT_BYTES address to ZL
	ldi ZH,high((2*OLED_INIT_BYTES)+0x4000)		;load high value of OLED_INIT_BYTES address to ZH
	ret											;return from subroutine
OLED_INIT_BYTES:
.DB 0xA8,0x1f,0x20,0x01,0x21,0x00,0x7F,0x22,0x00,0x03,0xDA,0x02,0x8D,0x14,0xAF
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
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;I2C ROUTINES     register used- temp/data/counter = r16/r17/r18                                                                 ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
sda_low:
	sbi ddrb,ddb0			;set ddrb0 making pb0 output, as port is 0 the output will sink causing the SDA line to go low
	ret						; return

sda_high:
	cbi ddrb,ddb0			;clear data direction bit of pb0.As port and DDR is 0 the pb0 is tristated and line is pulled high by I2C resistor
	ret

scl_low:
	sbi ddrb,ddb2			;set ddrb2 making pb2 output, as port is 0 the output will sink causing the SCK line to go low
	ret

scl_high:
	cbi ddrb,ddb2			;clear data direction bit of pb2.As port and DDR is 0 the pb2 is tristated and line is pulled high by I2C resistor
	ret

i2c_init:
	ldi temp,0x00			;load 0 in ddr & port to tristate the portB
	out ddrb,temp			
	out portb,temp
	ret

i2c_start:
	rcall sda_low			;I2C start condition needs SDA low before SCL goes low
	rcall scl_low
	ldi data,address		;slave address is copied to data register to initiate communication.integrated with subroutine to reduce code repetition.
	rcall i2c_write			;write function for i2c
	ret

i2c_stop:
	rcall sda_low			;I2C needs SCL to go high and then SDA to high to indicate stop transmission.SDA is pulled low to make sure.
	rcall scl_high
	rcall sda_high
	ret

i2c_write:
	ldi counter,0x08		;counter is loaded 8 as we send 8 bits.counter is decreased after sending each bit and completes transmission at 0
loop:
	rcall sda_low			;SDA is kept low and only made high if the read bit is 1
	mov temp,data			;copy data byte to temp
	andi temp,0x80			;AND copied value with 0b10000000.If 7th bit is one the register holds the same value or register becomes 0
	sbrc temp,7				;If 7th bit is 0 skip next instruction
	rcall sda_high			;call SDA high function if AND result was a 1 in 7th bit in TEMP register
	nop						;delay
	rcall scl_high			;call SCL high to clock the data.data is transmitted when the clock is cycled high to low while SDA is held stable at 0 or 1
	lsl data				;shift data register left to check the next bit
	rcall scl_low			;call SCL low function to complete the clocking
	dec counter				;decrease the counter which indicates how many bits left
	brne loop				;loop back through the process again until counter is 0
	rcall sda_high			;release SDA line for receiving the ACK from slave
	rcall scl_high			;call SCL to send 9th pulse and wait for ACK
	nop						;delay
	nop						;delay
	rcall scl_low			;ready for continuation of data transfer unless a stop is called
	rcall sda_low
	ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;I2C_COMMAND_WRITE  ;function to write a stream of commands to the OLED
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

i2c_command_write:
	rcall i2c_init			;
	rcall i2c_start
	ldi data,command		;command value is 0x00
	rcall i2c_write
	ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;I2C_DATA_WRITE	  ;function to write a stream of data to the OLED
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
i2c_data_write:
	rcall i2c_init
	rcall i2c_start
	ldi data,data_cmd			;data command is 0x40
	rcall i2c_write
	ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;ARRAY READ FUNCTION registers-r19/r20 ;array_length = r19,array_byte = r20 
;dependency = use ldi array_length,( number of bytes ) prior to rcall array_read
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

array_read:
loop1:
	ld array_byte,Z+
	mov data,array_byte
	rcall i2c_write
	dec array_length
	brne loop1
	ret


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;PRINT function   uses registers- temp1,temp,array_byte,ASCII,array_length
;characters to be printed has to be passed into register ASCII as ASCII values
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
printchar:	
	cpi ASCII,'.'		; decimal
	breq decimal1
	cpi ASCII,'-'		; minus
	breq minus1
	cpi ASCII,'C'
	breq letterC
	cpi ASCII,' '
	breq space
	rjmp print
decimal1:
	ldi ZL,low((2*fontdecimal)+0x4000)	;attiny10 doesnt allow direct flash read,should access via SRAM address starting at 0x4000 as per data sheet
	ldi ZH,high((2*fontdecimal)+0x4000)
	rjmp ASCII0
minus1:
	ldi ZL,low((2*fontminus)+0x4000)	;attiny10 doesnt allow direct flash read,should access via SRAM address starting at 0x4000 as per data sheet
	ldi ZH,high((2*fontminus)+0x4000)
	rjmp ASCII0

letterC:
	ldi ZL,low((2*fontC)+0x4000)	;attiny10 doesnt allow direct flash read,should access via SRAM address starting at 0x4000 as per data sheet
	ldi ZH,high((2*fontC)+0x4000)
	rjmp ASCII0

space:
	ldi ZL,low((2*fontspace)+0x4000)	;attiny10 doesnt allow direct flash read,should access via SRAM address starting at 0x4000 as per data sheet
	ldi ZH,high((2*fontspace)+0x4000)
	rjmp ASCII0


print:
	ldi ZL,low((2*font0)+0x4000)	;attiny10 doesnt allow direct flash read,should access via SRAM address starting at 0x4000 as per data sheet
	ldi ZH,high((2*font0)+0x4000)
	ldi array_byte,0x30   		;hex value of first ASCII caharacter in font aray is 0x30 .subtract start value from ASCII value returned from ASCII_CONVERTER to know the position of character in the array
	sub ASCII,array_byte		;result of subtraction will be position of first byte of the character
	breq ASCII0					;if the result is 0 go to label ASCII0 and print 6 bytes which is a space (0x20 = space in the font array,1st character)
	clr temp					;clear temp for counting
multiply:
	subi r30,low(-16)			;adding immediate not suppported. immediate extends beyond 8bit.using subi & sbci with -ve number will do & loading with hi &lo will propogate carry.
	sbci r31,high(-16)
	inc temp					;counter increased on each iteration
	cp temp,ASCII				;counter is compared to value in ASCII register(start address of the array row)
	brne multiply				;if the atart address not reached loop back through the code from label "multiply"
ASCII0:
	rcall i2c_data_write		;call function to initiate data write which will transmit data write command before transfer of data array
	ldi array_length,16			;specify array length of 6 as six bytes make 1 character.5 as in the array below and as mentioned above 0 padded by assembler. if stored bytes are not even assembler will add 0 byte to align code
	rcall array_read2			;call array read function which writes the bytes 1 after another in a sequence which ahs i2c write function included
	rcall i2c_stop				;stop i2c transfer
	ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;array_read2   - routine to read fonts from arry and double the size 16x8  = 32x16
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

array_read2:
loop12:
	ld array_byte,Z+			;load array_byte register with font pointed by Z
	rcall double				;call routine double to stretch font to double size,stretched font in r23 and r24
	mov data,r23				;move r23 to data
	rcall i2c_write				;transmit first half of stretched font
	mov data,r24				;move r24 to data
	rcall i2c_write				;transmit second half of stretched font
;	ldi data,0x00				;commented out code doubles horizontally but the display can show only 4 characters, implemented only vertically stretched font
;	rcall i2c_write
;	ldi data,0x00
;	rcall i2c_write
;	mov data,r23
;	rcall i2c_write
;	mov data,r24
;	rcall i2c_write
;	clr r23
;	clr r24
	dec array_length			;decrease array_length register by 1 from 16 to 0 each iteration
	brne loop12					;repeat until all array elements are stretched and written to OLED
	ret							;return to caller
	
double:							;routine stretches the text by doubling all the bits in the byte to a word
	clr r23						;clear the working register
	clr r24						;clear the working register
	ldi r27,8					;no of bits in each byte loaded in counter
	mov r16,array_byte			;move font loaded by Z in array_byte inside array_read2 routine to r16 for stretching
dloop:
	lsl r16						;left shift the msb of r16 into carry
	brcs bit1					;if carry set branch to bit1 label to double bit 1
	rol r23						;reach here if carry not set hence bit is 0, we will stretch by adding 1 more 0, shift 0 in carry to lsb of r23
	rol r24						;rotate left r24, msb of r23 shifted into carry in rol r23 will be shifted into lsb of r24
	rol r23						; as we are doubling 0 ,shift another 0 from carry to r23
	rol r24						; shift 0 to r24
	dec r27						; decrease counter
	brne dloop					; if counter not 0 , repeat for all bits in the loaded font array byte
	ret							; return to caller
bit1:
	rol r23						;reach here if carry set, shift 1 from carry to r23
	rol r24						;shift r24 left through carry
	sec							;set carry to 1 tp double the bit
	rol r23						;shift 1 from carry to r23
	rol r24						;shift r24 left through carry
	dec r27						; decrease counter
	brne dloop					; if counter not 0 , repeat for all bits in the loaded font array byte
	ret							; return to caller

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
fontspace:
.db 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00 // space
fontdecimal:
.db 0x00,0x00, 0x00,0x30, 0x00,0x30, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00// .
fontC:
.db 0xC0,0x07, 0x30,0x18, 0x08,0x20, 0x08,0x20, 0x08,0x20, 0x08,0x10, 0x38,0x08, 0x00,0x00// C
fontminus:
.db 0x00,0x00, 0x00,0x01, 0x00,0x01, 0x00,0x01, 0x00,0x01, 0x00,0x01, 0x00,0x01, 0x00,0x01// -
                     

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;CLEAR_OLED     registers array_byte/array_length used  writes 128*4 0's to OLED
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

clear_OLED:
	rcall ccursor
	ldi array_byte,4
	rcall i2c_data_write
loop3:
	ldi array_length,128
loop4:
	ldi data,0x00
	rcall i2c_write
	dec array_length
	brne loop4
loop5:
	dec array_byte
	brne loop3
	ret
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;set_cursor   routine to set cursor at required coordinates   use MACRO  eg  cursor 0 ,10
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
set_cursor:
;;;;;;;;;;;;;;;;;
;set_row        
;;;;;;;;;;;;;;;;;
	rcall i2c_command_write
	ldi data, 0x22			;oled command to set row
	rcall i2c_write
	mov data,posy			;row value in posY supplied to macro (starting point of cursor)
	rcall i2c_write
	ldi data,3				;max travel of cursor (here max is 3rd page = 3)
	rcall i2c_write
;;;;;;;;;;;;;;;;
;set_column    
;;;;;;;;;;;;;;;;
	ldi data, 0x21			;oled command to set column
	rcall i2c_write
	mov data,posx			;column value in posx supplied to macro
	rcall i2c_write
	ldi data,0x7f			;column begins at 0 to 127 as end in each row. we will choose column end as 127=0x7f
	rcall i2c_write
	rcall i2c_stop
	ret
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; ccursor    routine to place cursor at 0,0 coordinates
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
ccursor:
	cursor 0,0
	ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; delay routines
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

delayus:
	dec r26				; call & ret = 9cs, dec =1 ,brne =2 
	brne delayus
	ret


us10:
	ret
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;calc_fraction     routine used to calculate fraction part of the result , multiplys with625
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


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

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;measure - routine to init ,convert and read data from DS18B80
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

measure:
	rcall dallas_init				; call sensor init
	DALLAS_COMMAND skip_rom			; send rom command skip_rom with DALLAS_COMMAND macro
	F_COMMANDS convert				; send function command convert with macro F_COMMANDS
	
check1:
	rcall bit_read					; call routine bit_read to check busy status of sensor 0 = busy , 1= result ready
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
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;dallas_init    routine to initialise DS18B20 sensor -resets sensor
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

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
;write1     routine to write a 1 to DS18B20
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
write1:
	sbi ddrb,ddb1					;porta 2 is output by setting direction register
	nop								;delay of 3us data line stays low for more than minimum required time
	nop
	nop
	cbi ddrb,ddb1					;data line released by clearing direction register bit. line is pulled up by pullup voltage
	micros 60						;The delay of 60us to finish the time slot as per data sheet
	ret
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;write0    routine to write a 0 to DS18B20
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
write0:
	sbi ddrb,ddb1					;porta 2 is output by setting direction register
	micros 60						;delay 60us
	cbi ddrb,ddb1					;data line released by clearing direction register bit. line is pulled up by pullup voltage
	rcall us10
	ret
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;byte_read   reads a byte from DS18B20
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
byte_read:
	ldi r25,8						; counter value 8 for 8 bits of a byte
	clr r24							; byte is assembled here bit by bit
loopread:
	sbi ddrb,ddb1					;porta 2 is output by setting direction register
	rcall us10
	cbi ddrb,ddb1					;clear direction register to release data line and turn the master to input
	rcall us10
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
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;bit_read         routine to read single bit from DS18B20  , used to read busy flag of sensor
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
bit_read:
	ldi r25,1						;load counter value 1, we read 1 bit so value 1
	clr r24							;clear r24 where the bit is shifted in
	rcall loopread					;call routine loopread which is part of byte_read to shift in the transmitted bit from the sensor
	andi r24,0x80					;and r24 with 0x80 , 0 is bit value = 0 , 80h is bit value = 1
	ret								;return to caller
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;* Integer to ASCII converter subrountine here ,r25:r26 is the input registers
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

ascii_convert:                         ;16 bit entry
				
                clr     r19            ;digit counter 
                inc     r19            ;---- decimal string generating 
                clr     r27            ;var1 /= 10; temp0
                ldi     r28,16         ; temp1
                lsl     r25            ;var0
                rol     r26            ;var1
                rol     r27            ;temp0
                cpi     r27,10         ;temp0
                brcs    PC+3            ;
                subi    r27,10         ;temp0
                inc     r25            ;var0
                dec     r28            ;temp1
                brne    PC-8           ;/
                subi    r27,-'0'       ;Push the remainder (a decimal digit when added with 0x30 ASCII)
                push    r27            ;/temp0
                cp      r25,r28        ;if(var =! 0) var0,temp1
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
                ret
xmit:
	st Z+,r25
	ret


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;binary_decimal   converts binary to decimal
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


binary_decimal:
	ldi r16,'.'						; load r16 with ascii decimal point
	sts decimal,r16					; store in sram register
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
	ret












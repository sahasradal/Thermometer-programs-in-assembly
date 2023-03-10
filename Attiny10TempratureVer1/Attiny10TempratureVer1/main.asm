;
; Attiny10TempratureVer2.asm
;
; Created: 20/04/2022 12:33:11
; Author : Manama
; tested and found working OLED ssd 1306  with 16x8 fonts



;pb0 is SDA
;pb2 is SCL
;pb1 is data line for temp sensor



.equ fclk = 1000000
.equ address = 0x78
.equ command = 0x00
.equ data_cmd = 0x40
.equ OLED_INIT_LEN = 15    ; 14 if screen flip is needed , 12 if normal screen
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

.macro printf						;macro to print characters on OLED
ldi temp,@0							
mov ASCII,temp
rcall print
.endm 

.macro cursor
ldi posy,@0
ldi posx,@1
rcall set_cursor
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

.dseg
PAD:		.byte 1				; temp space for calculation
presence:	.byte 1				; stores response of ds18b20 during init
minussign:	.byte 1				; stores minus sign if result is negative else store space for positive result
firstdigit: .byte 1				; 100th position of result whole number
seconddigit:.byte 1				; 10th position of result whole number
thirddigit: .byte 1				; unit position of result whole number
decimal:	.byte 1				; stores ascii value of decimal point
firstplace: .byte 1				; stores ascii value of 1st fraction
secplace:	.byte 1				; stores ascii value of 2nd fraction
thirdplace: .byte 1				; stores ascii value of 3rd fraction
fourthplace:.byte 1				; stores ascii value of 4th fraction
SCRATCHPAD: .byte 2				; stores temprature high byte & low byte value copied from ds18b20sensor


.cseg
reset:
	rcall i2c_command_write		;call 12c write subriutine
	rcall OLED_INIT				;call OLED initialization routine
	ldi array_length,OLED_INIT_LEN		;length of OLED init array 12 loaded
	rcall array_read			;call array_read subroutine to send OLED init to display via 12c
	rcall i2c_stop				;call 12c stop
	rcall clear_OLED			;call OLED screen clear routine. writes 0x00 in all OLED GDRAM address 128*32
	cursor 0, 10				;cursor macro is called .2nd line 20th slot

main_loop:
	rcall measure				;call subroutine to measure temprature (read sensor,convert to decimal,convert to ASCII,store in SRAM)
	ldi r16,' '					;the last 3 fraction values are not printed,instead we use the space to print 2 spaces and letter C, -100.1  C
	sts secplace,r16			;the last 3 fraction values are not printed,instead we use the space to print 2 spaces and letter C, -100.1  C
	sts thirdplace,r16			;the last 3 fraction values are not printed,instead we use the space to print 2 spaces and letter C, -100.1  C
	ldi r16, 'C'				;the last 3 fraction values are not printed,instead we use the space to print 2 spaces and letter C, -100.1  C
	sts fourthplace,r16			;the last 3 fraction values are not printed,instead we use the space to print 2 spaces and letter C, -100.1  C
	ldi r25,9					;length of SRAM to be copied (max is 9), 6 means result will look -100.1 , if positive leading space else leading -
	ldi YL,low(minussign)		;set pointer Y to SRAM address 'minussign'
	ldi YH,high(minussign)		;set pointer Y to SRAM address 'minussign'
	cursor 0, 10				;set OLED cursor to 1st line 10th step
printtemp:
	ld ASCII,Y+
	rcall printchar
	dec r25
	brne printtemp
	rjmp main_loop


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
	nop						; delay
	nop						;delay
	rcall scl_low			;ready for continuation of data transfer unless a stop is called
	rcall sda_low
	ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;I2C_COMMAND_WRITE  ;function to write a stream of commands to the OLED
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
i2c_command_write:
	rcall i2c_init			;
	rcall i2c_start
	ldi data,command		;command value is 0x00
	rcall i2c_write
	ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;I2C_DATA_WRITE	  ;function to write a stream of data to the OLED
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
i2c_data_write:
	rcall i2c_init
	rcall i2c_start
	ldi data,data_cmd			;data command is 0x40
	rcall i2c_write
	ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;ARRAY READ FUNCTION registers-r19/r20 ;array_length = r19,array_byte = r20 
;dependency = use ldi array_length,( number of bytes ) prior to rcall array_read
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
array_read:
loop1:
	ld array_byte,Z+			;load register array_byte data from address pointed by Z pointer (array_read should be called after setting up Z to required array)
	mov data,array_byte			;copy array_byte to I2C data register to transmit
	rcall i2c_write				;I2C write
	dec array_length			;decrease array element count(array element count should be initialized in array_length register before calling array_read)
	brne loop1					;loop gain until all array elements are transmitted
	ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;PRINTCHAR function   
;characters to be printed has to be passed into register ASCII as ASCII values, only 0,1,2,3,4,5,6,7,8,9,-,., ,C are supported
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
printchar:	
	cpi ASCII,'.'						; decimal
	breq decimal1						; branch to label decimal1
	cpi ASCII,'-'						; minus
	breq minus1							; branch to label minus1
	cpi ASCII,'C'						; ascii C
	breq letterC						; branch to label letterC
	cpi ASCII,' '						; space
	breq space							; branch to lbel space
	rjmp print							; if none of above jump to label print which loads the first font '0'and calculates the relative position
decimal1:
	ldi ZL,low((2*fontdecimal)+0x4000)	;attiny10 doesnt allow direct flash read,should access via SRAM address starting at 0x4000 as per data sheet
	ldi ZH,high((2*fontdecimal)+0x4000)	;pointer set to font array for . decimal
	rjmp ASCII0
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
	subi r30,low(-16)			;adding immediate not suppported. immediate extends beyond 8bit.using subi & sbci with -ve number will do & loading with hi &lo will propogate carry.
	sbci r31,high(-16)
	inc temp					;counter increased on each iteration
	cp temp,ASCII				;counter is compared to value in ASCII register(start address of the array row)
	brne multiply				;if the atart address not reached loop back through the code from label "multiply"
ASCII0:
	rcall i2c_data_write		;call function to initiate data write which will transmit data write command before transfer of data array
	ldi array_length,16			;specify array length of 6 as six bytes make 1 character.5 as in the array below and as mentioned above 0 padded by assembler. if stored bytes are not even assembler will add 0 byte to align code
	rcall array_read			;call array read function which writes the bytes 1 after another in a sequence which ahs i2c write function included
	rcall i2c_stop			    ;stop i2c transfer
	ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;FONTS   fonts below 16bytes ,hence array lenth =16
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

font0:
.db 0x00,0x00, 0xE0,0x0f, 0x10,0x10, 0x08,0x20, 0x08,0x20, 0x10,0x10, 0xE0,0x0f, 0x00,0x00 //0
font1:
.DB 0x00,0x00, 0x10,0x20, 0x10,0x20, 0xF8,0x3F, 0x00,0x20, 0x00,0x20, 0x00,0x00, 0x00,0x00 //1 
font2:
.db 0x00,0X00, 0x70,0X30, 0x08,0X28, 0x08,0X24, 0x08,0X22, 0x88,0X21, 0x70,0X30, 0x00,0X0 // 2
font3:
.DB 0x00,0X00, 0x30,0X18, 0x08,0X20, 0x88,0X20, 0x88,0X20, 0x48,0X11, 0x30,0X0E, 0x00,0X00 //3
font4:
.DB 0x00,0X00, 0x00,0X07, 0xC0,0X04, 0x20,0X24, 0x10,0X24, 0xF8,0X3F, 0x00,0X24, 0x00,0X00 //4
font5:
.DB 0x00,0x00, 0xF8,0x19, 0x08,0x21, 0x88,0x20, 0x88,0x20, 0x08,0x11, 0x08,0x0e, 0x00,0x00 //5
font6:
.DB 0x00,0x00, 0xE0,0x0f, 0x10,0x11, 0x88,0x20, 0x88,0x20, 0x18,0x11, 0x00,0x0e, 0x00,0x00 //6
font7:
.DB 0x00,0x00, 0x38,0x00, 0x08,0x00, 0x08,0x3f, 0xC8,0x00, 0x38,0x00, 0x08,0x00, 0x00,0x00 //7
font8:
.DB 0x00,0x00, 0x70,0x1c, 0x88,0x22, 0x08,0x21, 0x08,0x21, 0x88,0x22, 0x70,0x1c, 0x00,0x00 //8 
font9:
.DB 0x00,0x00, 0xE0,0x00, 0x10,0x31, 0x08,0x22, 0x08,0x22, 0x10,0x11, 0xE0,0x0f, 0x00,0x00 //9
fontspace:
.db 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00 //space
fontdecimal:
.db 0x00,0x00, 0x00,0x30, 0x00,0x30, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00 //.
fontC:
.db 0xC0,0x07, 0x30,0x18, 0x08,0x20, 0x08,0x20, 0x08,0x20, 0x08,0x10, 0x38,0x08, 0x00,0x00 //C
fontminus:
.db 0x00,0x00, 0x00,0x01, 0x00,0x01, 0x00,0x01, 0x00,0x01, 0x00,0x01, 0x00,0x01, 0x00,0x01 //-
                     

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;CLEAR_OLED     registers array_byte/array_length used  writes 128*4 0's to OLED
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

clear_OLED:
	ldi array_byte,3				;this code uses vertical write with 2 page high characters,so change page height to 4 (0-3) so cursor runs entire screen
	sts PAD,array_byte				;if page height is left at 0-1 or 2-3 range the clear screen doesnt clear the whole screen but only the range specified by cursor macro
	cursor 0, 0						;cursor is placed at top left corner (as page range is set to 0-3 the subroutine will fill all columns of all pages with 0x00)
	ldi array_byte,4				;couter loaded with 4 ( 1 count per page)
	rcall i2c_data_write			;write I2C
loop3:
	ldi array_length,128			;load counter 128 (each count for each column in 1 page ,total 128 columns)
loop4:
	ldi data,0x00					;laod I2C data register with 0x00 to be written to each OLED RAM
	rcall i2c_write					;I2C write 1 byte
	dec array_length				;decrease column count from 128 to 0 in each iteration
	brne loop4						;loop till column count reaches 0
loop5:
	dec array_byte					;decrease page count from 3-0 (4 steps)
	brne loop3						;loop back till a 4 pages are written
	clr array_byte					;clear array_byte register,previously 3 at the start of clear_OLED routine
	sts PAD,array_byte				;store array byte value in SRAM address PAD. This value is later compared by cursor routine to determine call is char print or clear screen
	ret
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;SET_ROW   uses register y/x /temp   r24/r23/r16
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
set_cursor:
;;;;;;;;;;;;;;;;;
;set_row        
;;;;;;;;;;;;;;;;;
	rcall i2c_command_write	;call routine to transmit OLED command 
	ldi data, 0x22			;oled command to set row
	rcall i2c_write			;I2C write set row command
	mov data,posy			;row value in posY supplied to macro
	rcall i2c_write			;I2C write page start value
	lds r16,PAD				;copy value stored in sram space PAD
	sbrs r16,0				;if bit 0 of the value is set skip next instruction, values 3 and 0 expected ,if value 3 clear screen and if 0 character write
	subi posy,-1			;if r16 was 0 we add 1 to posy register to limit the cursor movement to 2 pages. if posy is 0 (start) this makes page1 (0+1 end),if posy is 2(start) makes (2+1=3 end)
	or posy,r16				;or posy with value in r16 to set the page end value( 1 or 3)
	mov data,posy			;row end - can be 1 ,2,3. we choose last page 3
	rcall i2c_write			;I2C write the page end value
;;;;;;;;;;;;;;;;
;set_column    
;;;;;;;;;;;;;;;;
	ldi data, 0x21			;oled command to set column
	rcall i2c_write			;I2C write OLED set column command
	mov data,posx			;column value in posx supplied to macro
	rcall i2c_write			;I2C write the column start value
	ldi data,0x7f			;column begins at 0 to 127 as end in each row. we will choose column end as 127=0x7f
	rcall i2c_write			;I2C write the column end value
	rcall i2c_stop			;stop I2C transmission
	ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


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

delayus:
	dec r26							; call & ret = 9cs, dec =1 ,brne =2 
	brne delayus
	ret

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

write1:
	sbi ddrb,ddb1					;porta 2 is output by setting direction register
	nop								;delay of 3us data line stays low for more than minimum required time
	nop
	nop
	cbi ddrb,ddb1					;data line released by clearing direction register bit. line is pulled up by pullup voltage
	micros 60						;The delay of 60us to finish the time slot as per data sheet
	ret

write0:
	sbi ddrb,ddb1					;porta 2 is output by setting direction register
	micros 60						;delay 60us
	cbi ddrb,ddb1					;data line released by clearing direction register bit. line is pulled up by pullup voltage
	rcall us10
	ret

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

bit_read:
	ldi r25,1						;load counter value 1, we read 1 bit so value 1
	clr r24							;clear r24 where the bit is shifted in
	rcall loopread					;call routine loopread which is part of byte_read to shift in the transmitted bit from the sensor
	andi r24,0x80					;and r24 with 0x80 , 0 is bit value = 0 , 80h is bit value = 1
	ret								;return to caller

us10:
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
	com r17							; take complement (converting 2?omplement )
	subi r17,-1						; add 1 to get the original positive number represented by 2's complement
;	sbci r18,0
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

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;* Integer to ASCII converter subrountine here ,
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
                brne    PC-8            ;/
                subi    r27,-'0'       ;Push the remainder (a decimal digit when added with 0x30 ASCII)
                push    r27            ;/temp0
                cp      r25,r28       ;if(var =! 0) var0,temp1
                cpc     r26,r28       ; continue digit loop;var1,temp1
                brne    PC-16           ;/
                cp      r19,r20        ;Adjust string length (this can be removed for auto-length)temp2,len
                brcc    PC+5            ;
                inc     r19            ; temp2
                ldi     r25,' '        ;var0
                push    r25            ;var0
                rjmp    PC-5            ;/
                pop     r25            ;Put decimal string var0
                rcall  xmit            ;<-- Put a char (var0) to memory, console or any display device
                dec     r19            ;temp2
                brne    PC-3            ;/
				clr r20					; clear length for next call
                ret
xmit:
	st Z+,r25
	ret





;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;





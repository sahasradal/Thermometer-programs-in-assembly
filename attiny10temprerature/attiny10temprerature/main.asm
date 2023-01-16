;
; attiny10temprerature.asm
; tested and working
; Created: 06/04/2022 18:51:56
; Author : Manama
; chipset = attiny10
; Dallas DS18B20 temprature sensor
; single sensor is used with 5k pullup and direct power supply. SkipROM command is used as only 1 sensor in the bus connected to PB1 of attiny10
; CRC and ROM is not checked, Only SKIPROM, CONVERT,READSCRATCHPAD commands used. Scratchpad reading terminated after 2 tempraturs registers are read.
; result dispalyed in ssd1306 OLED with 5x8 font which is very small
; tested OK	
;
;pb0 is SDA
;pb2 is SCL
;pb1 is data line for temp sensor



.equ fclk = 1000000
.equ address = 0x78
.equ command = 0x00
.equ data_cmd = 0x40
.equ OLED_INIT_LEN = 12    ; 14 if screen flip is needed , 12 if normal screen
.equ font_length = 5
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
PAD: .BYTE 1
PAD1: .byte 1
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
	cursor 0, 20				;cursor macro is called .2nd line 20th slot
	ldi ZL,low((2*T)+0x4000)	;attiny10 doesnt allow direct flash read,should access via SRAM address starting at 0x4000 as per data sheet
	ldi ZH,high((2*T)+0x4000)
	rcall ASCII0
	ldi ZL,low((2*E)+0x4000)	;attiny10 doesnt allow direct flash read,should access via SRAM address starting at 0x4000 as per data sheet
	ldi ZH,high((2*E)+0x4000)
	rcall ASCII0
	ldi ZL,low((2*M)+0x4000)	;attiny10 doesnt allow direct flash read,should access via SRAM address starting at 0x4000 as per data sheet
	ldi ZH,high((2*M)+0x4000)
	rcall ASCII0
	ldi ZL,low((2*P)+0x4000)	;attiny10 doesnt allow direct flash read,should access via SRAM address starting at 0x4000 as per data sheet
	ldi ZH,high((2*P)+0x4000)
	rcall ASCII0
	cursor 2, 100
	ldi ZL,low((2*C)+0x4000)	;attiny10 doesnt allow direct flash read,should access via SRAM address starting at 0x4000 as per data sheet
	ldi ZH,high((2*C)+0x4000)
	rcall ASCII0
	
main_loop:
	rcall measure				; subroutine to read ,convert to decimal and ASCII from DS18B20
	ldi r25,6					; r25 used as counter for printing ASCII characters stored in SRAM registers begining with minussign, number in r25 decides no of characters max = 9, here loaded 6 to display only 1 decimal position eg 10.1
	ldi YL,low(minussign)		; load Y pointer with SRAM register (minussign)of stored ASCII characters after reading sensor
	ldi YH,high(minussign)
	cursor 2, 20				; set ssd1602 cursor to 3rd line (0,1,2 are lines) 20th position to start printing temprature value
printtemp:
	ld ASCII,Y+					; load register ASCII with value stored in address pointed by Y , starts from minussign in SRAM
	rcall print					; call subroutine print to calculate and write the corresponding font from the font table
	dec r25						; decrease counter
	brne printtemp				; if counter not exhausted loop to label printtemp which will load from next register in SRAM
	rjmp main_loop				; repeat this main loop for ever


OLED_INIT:
	ldi ZL,low((2*OLED_INIT_BYTES)+0x4000)		;load low value of OLED_INIT_BYTES address to ZL
	ldi ZH,high((2*OLED_INIT_BYTES)+0x4000)		;load high value of OLED_INIT_BYTES address to ZH
	ret											;return from subroutine
OLED_INIT_BYTES:
.DB 0xA8,0x1F,0x22,0x00,0x03,0x20,0x00,0xDA,0x02,0x8D,0x14,0xAF,0xA1,0xC8
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;	
;  0xA8, 0x1F,       // set multiplex (HEIGHT-1): 0x1F for 128x32, 0x3F for 128x64 ; 
;  0x22, 0x00, 0x03, // set min and max page					   ;
;  0x20, 0x00,       // set horizontal memory addressing mode			   ;
;  0xDA, 0x02,       // set COM pins hardware configuration to sequential          ;
;  0x8D, 0x14,       // enable charge pump                                         ;
;  0xAF,             // switch on OLED                                             ;
;  0xA1, 0xC8        // flip the screen                                            ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;I2C ROUTINES     register used- temp/data/counter = r16/r17/r18                                                                 ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
sda_low:
	sbi ddrb,ddb0			;set ddrb0 making pb0 output, as port is 0 the output will sink causing the SDA line to go low
	nop						;mini delay
	ret						; return

sda_high:
	cbi ddrb,ddb0			;clear data direction bit of pb0.As port and DDR is 0 the pb0 is tristated and line is pulled high by I2C resistor
	nop						;delay
	ret

scl_low:
	sbi ddrb,ddb2			;set ddrb2 making pb2 output, as port is 0 the output will sink causing the SCK line to go low
	nop
	ret

scl_high:
	cbi ddrb,ddb2			;clear data direction bit of pb2.As port and DDR is 0 the pb2 is tristated and line is pulled high by I2C resistor
	nop
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
print:
	ldi ZL,low((2*fonts)+0x4000)	;attiny10 doesnt allow direct flash read,should access via SRAM address starting at 0x4000 as per data sheet
	ldi ZH,high((2*fonts)+0x4000)
	ldi array_byte,0x20   			;hex value of first ASCII caharacter in font aray is 0x20 .subtract start value from ASCII value returned from ASCII_CONVERTER to know the position of character in the array
	sub ASCII,array_byte			;result of subtraction will be position of first byte of the character
	breq ASCII0						;if the result is 0 go to label ASCII0 and print 6 bytes which is a space (0x20 = space in the font array,1st character)
	clr temp						;clear temp for counting
;ldi temp1,0x00						;clear temp1 for subtraction (ithink this was for 0 carry propogation)
multiply:
	subi r30,low(-6)				;adding immediate not suppported. immediate extends beyond 8bit.using subi & sbci with -ve number will do & loading with hi &lo will propogate carry.
	sbci r31,high(-6)
	inc temp						;counter increased on each iteration
	cp temp,ASCII					;counter is compared to value in ASCII register(start address of the array row)
	brne multiply					;if the atart address not reached loop back through the code from label "multiply"
ASCII0:
	rcall i2c_data_write			;call function to initiate data write which will transmit data write command before transfer of data array
	ldi array_length,6				;specify array length of 6 as six bytes make 1 character.5 as in the array below and as mentioned above 0 padded by assembler. if stored bytes are not even assembler will add 0 byte to align code
	rcall array_read				;call array read function which writes the bytes 1 after another in a sequence which ahs i2c write function included
	rcall i2c_stop					;stop i2c transfer
	ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;FONTS   fonts below 5bytes ,assembler will add one byte of padding with 0. hence array lenth =6
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

fonts:
.db 0x00, 0x00, 0x00, 0x00, 0x00 //sp 0  0x20
.DB 0x00, 0x00, 0x2f, 0x00, 0x00 // ! 1  0x21
.DB 0x00, 0x07, 0x00, 0x07, 0x00 // " 2  0x22
.DB 0x14, 0x7f, 0x14, 0x7f, 0x14 // # 3  0x23
.DB 0x24, 0x2a, 0x7f, 0x2a, 0x12 // $ 4  0x24
.DB 0x62, 0x64, 0x08, 0x13, 0x23 // % 5  0x25
.DB 0x36, 0x49, 0x55, 0x22, 0x50 // & 6  0x26
.DB 0x00, 0x05, 0x03, 0x00, 0x00 // ' 7  0x27
.DB 0x00, 0x1c, 0x22, 0x41, 0x00 // ( 8  0x28
.DB 0x00, 0x41, 0x22, 0x1c, 0x00 // ) 9  0x29
.DB 0x14, 0x08, 0x3E, 0x08, 0x14 // * 10 0x2A
.DB 0x08, 0x08, 0x3E, 0x08, 0x08 // + 11 0x2B
.DB 0x00, 0x00, 0xA0, 0x60, 0x00 // , 12 0x2C
.DB 0x08, 0x08, 0x08, 0x08, 0x08 // - 13 0x2D
.DB 0x00, 0x60, 0x60, 0x00, 0x00 // . 14 0x2E
.DB 0x20, 0x10, 0x08, 0x04, 0x02 // / 15 0x2F
.DB 0x3E, 0x51, 0x49, 0x45, 0x3E // 0 16 0x30
.DB 0x00, 0x42, 0x7F, 0x40, 0x00 // 1 17 0x31
.DB 0x42, 0x61, 0x51, 0x49, 0x46 // 2 18 0x32
.DB 0x21, 0x41, 0x45, 0x4B, 0x31 // 3 19 0x33
.DB 0x18, 0x14, 0x12, 0x7F, 0x10 // 4 20 0x34
.DB 0x27, 0x45, 0x45, 0x45, 0x39 // 5 21 0x35
.DB 0x3C, 0x4A, 0x49, 0x49, 0x30 // 6 22 0x36
.DB 0x01, 0x71, 0x09, 0x05, 0x03 // 7 23 0x37
.DB 0x36, 0x49, 0x49, 0x49, 0x36 // 8 24 0x38
.DB 0x06, 0x49, 0x49, 0x29, 0x1E // 9 25 0x39
/*
.DB 0x00, 0x36, 0x36, 0x00, 0x00 // : 26 0x3A
.DB 0x00, 0x56, 0x36, 0x00, 0x00 // ; 27 0x3B
.DB 0x08, 0x14, 0x22, 0x41, 0x00 // < 28 0X3C
.DB 0x14, 0x14, 0x14, 0x14, 0x14 // = 29 0X3D
.DB 0x00, 0x41, 0x22, 0x14, 0x08 // > 30 0X3E
.DB 0x02, 0x01, 0x51, 0x09, 0x06 // ? 31 0X3F
.DB 0x32, 0x49, 0x59, 0x51, 0x3E // @ 32 0X40
.DB 0x7C, 0x12, 0x11, 0x12, 0x7C // A 33 0X41
.DB 0x7F, 0x49, 0x49, 0x49, 0x36 // B 34 0X42
*/
C:
.DB 0x3E, 0x41, 0x41, 0x41, 0x22 // C 35 0X43
;.DB 0x7F, 0x41, 0x41, 0x22, 0x1C // D 36 0X44
E:
.DB 0x7F, 0x49, 0x49, 0x49, 0x41 // E 37 0X45
/*
.DB 0x7F, 0x09, 0x09, 0x09, 0x01 // F 38 0X46
.DB 0x3E, 0x41, 0x49, 0x49, 0x7A // G 39 0X47
.DB 0x7F, 0x08, 0x08, 0x08, 0x7F // H 40 0X48
.DB 0x00, 0x41, 0x7F, 0x41, 0x00 // I 41 0X49
.DB 0x20, 0x40, 0x41, 0x3F, 0x01 // J 42 0X4A
.DB 0x7F, 0x08, 0x14, 0x22, 0x41 // K 43 0X4B
.DB 0x7F, 0x40, 0x40, 0x40, 0x40 // L 44 0X4C
*/
M:
.DB 0x7F, 0x02, 0x0C, 0x02, 0x7F // M 45 0X4D
;.DB 0x7F, 0x04, 0x08, 0x10, 0x7F // N 46 0X4E
;.DB 0x3E, 0x41, 0x41, 0x41, 0x3E // O 47 0X4F
P:
.DB 0x7F, 0x09, 0x09, 0x09, 0x06 // P 48 0X50
/*
.DB 0x3E, 0x41, 0x51, 0x21, 0x5E // Q 49 0X51
.DB 0x7F, 0x09, 0x19, 0x29, 0x46 // R 50 0X52
.DB 0x46, 0x49, 0x49, 0x49, 0x31 // S 51 0X53
*/
T:
.DB 0x01, 0x01, 0x7F, 0x01, 0x01 // T 52 0X54
/*
.DB 0x3F, 0x40, 0x40, 0x40, 0x3F // U 53 0X55
.DB 0x1F, 0x20, 0x40, 0x20, 0x1F // V 54 0X56
.DB 0x3F, 0x40, 0x38, 0x40, 0x3F // W 55 0X57
.DB 0x63, 0x14, 0x08, 0x14, 0x63 // X 56 0X58
.DB 0x07, 0x08, 0x70, 0x08, 0x07 // Y 57 0X59
.DB 0x61, 0x51, 0x49, 0x45, 0x43 // Z 58 0X5A
.DB 0x00, 0x7F, 0x41, 0x41, 0x00 // [ 59 0X5B
.DB 0x02, 0x04, 0x08, 0x10, 0x20 // \ 60 0X5C
.DB 0x00, 0x41, 0x41, 0x7F, 0x00 // ] 61 0X5D
.DB 0x04, 0x02, 0x01, 0x02, 0x04 // ^ 62 0X5E
.DB 0x40, 0x40, 0x40, 0x40, 0x40  // _ 63 0X5F
*/
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;CLEAR_OLED     registers array_byte/array_length used  writes 128*4 0's to OLED, subroutine to clear whole screen ,write 0
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

clear_OLED:
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
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;SET_ROW   uses register y/x /temp   r24/r23/r16
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
set_cursor:
;;;;;;;;;;;;;;;;;
;set_row        
;;;;;;;;;;;;;;;;;
	rcall i2c_command_write
	ldi data, 0x22			;oled command to set row
	rcall i2c_write
	mov data,posy			;row value in posY supplied to macro
	rcall i2c_write
	ldi data,0x03			;row end - can be 1 ,2,3. we choose last page 3
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



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;* Integer to ASCII converter subrountine here ,r26:r25    r17:r16 is the input registers
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Universal decimal string conversion (8/16 bit unsigned)
;
; Register Variables:
;  Call:   r26:r25  = 16 bit value to be converted r26:r25
;          r20     = final String length r20
;          r28:r27  = <Don't care> (high register must be assigned)r28:r27 temp registers
;          r19    = <Don't care>r19
;
;  Result: r26:r25  = <Unknown>
;          len     = <Not changed>
;          r28:r27  = <Unknown>
;          r19    = 0
;
; ; Stack: 10 bytes max (+output routine)
;
; Examples:   var1    len    output
;              100      0    "100"
;             1234      0    "1234"
;                0      7    "      0"
;              100      5    "  100"
;              100      2    "100"
;mk_decu8:       clr     var1            ;8 bit entry
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




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;calc_fraction multiplies the LSB nibble with scaled value of 625 to get decimal value of fraction, each bit of DS18B20 fraction is weighted 0.0625 degree C
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

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
	dec r19					; if r19 not 1 or 0 we reach here, decrease 1 from r19 as 625 is already loaded, we need to multiply 1 time less
fmultiply:
	add r22,r20				; add 625 to 625
	adc r23,r21				; add with carry
	adc r24,r31				; add with carry
	dec r19					; deecrease r19 after each addition
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
;f_command    sends function commands to ds18b20 cll with macro , ROM_COMMANDS  sends rom commands to ds18b20 call with macro
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

f_command:
	ldi r25,8				; load counter value 8 (byte )
	mov r24,r22				; copy command value from macro register r22
loop51:
	lsr r24					; shift right r24 ,lsb now in carry, maxim protocol transmits lsbit first
	brcs high2				; branch to high2 label if carry flag set , if bit is 1 carry flag is set else carry will 0
	rcall write0			; if bit was 0 in previous step call routine to write 0 to DS18b20 
	dec r25					; decrease counter
	brne loop51				; if counter not reach 0 loop back to label loop51 till all bits are transmitted
	ret
high2:
	rcall write1			; reach here if carry is set , call subroutine to write 1 to ds18b20
	dec r25					; decrease counter
	brne loop51				; if counter not reach 0 loop back to label loop51 till all bits are transmitted
	ret

ROM_COMMANDS:
	ldi r25,8				; load counter value 8 (byte )
	rcall dallas_init		; call ds18b20 initialisation routine
	lds r16,presence		; check response value stored in sram to find out sensor responded or not
	cpi r16,0				; if sensor not responded exit
	brne exit0				; if above condition true exit routine (sensor faulty)
	mov r24,r22				; copy command value from macro register r22
loop41:
	lsr r24					; shift right r24 ,lsb now in carry, maxim protocol transmits lsbit first
	brcs high1				; branch to high2 label if carry flag set , if bit is 1 carry flag is set else carry will 0
	rcall write0			; if bit was 0 in previous step call routine to write 0 to DS18b20
	dec r25					; decrease counter
	brne loop41				; if counter not reach 0 loop back to label loop41 till all bits are transmitted
	ret
high1:
	rcall write1			; reach here if carry is set , call subroutine to write 1 to ds18b20
	dec r25					; decrease counter
	brne loop41				; if counter not reach 0 loop back to label loop51 till all bits are transmitted
exit0:	ret
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

measure:
	cbi portb,portb1			;pin porta 2 is pulled down by clearing port register
	rcall dallas_init			; call ds18b20 initialisation
	DALLAS_COMMAND skip_rom		; call macro to send rom command skip_rom
	F_COMMANDS convert			; call macro to send convert function command
	
check1:
	rcall bit_read				; call bit read subroutine, checks/reads a single bit from 18b20 , if read 1 sensor busy , if read 0 result ready
	sbrs r24,7					; if read 1 sensor result ready , if read 1 result ready , read bit is shifted to msb of r24 so set = 1 clear = 0
	rjmp check1					; if 0 loop to label check1 till  1 is read from sensor (wit till conversion is complete)
	rcall dallas_init			; call ds18b20 initialisation
	DALLAS_COMMAND skip_rom		; call macro to send rom command skip_rom
	F_COMMANDS read_sscratchpad	; call macro to send read_sscratchpad function command
	rcall byte_read				; call byte read subroutine to reear sratch pad of the sensor , reads each byte and copies it to SRAM
	sts SCRATCHPAD + 1, r24
	rcall byte_read
	sts SCRATCHPAD , r24
	nop
	rcall dallas_init
	rcall binary_decimal
	ret
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
delayus:
	dec r26				; call & ret = 9cs, dec =1 ,brne =2 
	brne delayus
	ret
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
dallas_init:
	sbi ddrb,ddb1		;porta 2 is output by setting direction register
	micros 480			;minimum time pull down time required for reset
	cbi ddrb,ddb1		;change PA2 to input to verify whether DS1B20 pulls the line low to indicate presence
	micros 70			;sensor should respond after 60 seconds ,so wait for the minimum time
	in r16,PINB			;copy the PA2 PIN value from PORTA_PIN register
	andi r16,0b00000010	;AND with 0x04 the value of r16 ,if 0 sensor responded , if 1 no response at the time of checking
	sts presence,r16	;copy value to memory
	micros 410			; wait for the remaining time slot of 460 us
	nop
	ret

write1:
	sbi ddrb,ddb1		;porta 2 is output by setting direction register
	nop					;delay of 3us data line stays low for more than minimum required time
	nop
	nop
	cbi ddrb,ddb1		;data line released by clearing direction register bit. line is pulled up by pullup voltage
	micros 64			;The delay of 60us to finish the time slot as per data sheet
	ret

write0:
	sbi ddrb,ddb1	;porta 2 is output by setting direction register
	micros 60			;delay 60us
	cbi ddrb,ddb1		;data line released by clearing direction register bit. line is pulled up by pullup voltage
	rcall us10
	ret

byte_read:
	ldi r25,8
	clr r24
loopread:
	sbi ddrb,ddb1		;porta 2 is output by setting direction register
	nop					;delay of minimum 1 us as per data sheet
	nop
	nop
	cbi ddrb,ddb1		;clear direction register to release data line and turn the master to input
	nop
	nop
	nop
	nop
	nop
	sbic pinb,pinb1 
	rjmp one			;if not 0 branch to label one
	clc					;clear carry bit in SREG
	ror r24				;rotate right through carry (0 is shifted into msb of r20 from carry,ds18B20 transmits lsb of LSBYTE firt.At the end the first bit in MSB reaches LSB
	micros 55			;wait for the remaining time slot of the read bit
	dec r25				;decrease counter
	brne loopread			;if counter not 0 branch back to loop3
	ret
one:
	sec					;set carry bit in SREG
	ror r24				;rotate through carry and the carry bit1 will be shifted to MSB of r20 , bit7 >>>>>>> bit0
	micros 55			;wait for the remaining time slot of the read bit
	dec r25				; counter	;wait for the remaining time slot of the read bit
	brne loopread			;if counter not 0 branch back to loop3
	ret

bit_read:
	ldi r25,1
	clr r24
	rcall loopread
	andi r24,0x80
	ret

us10:
	ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

binary_decimal:
	ldi r16,'.'
	sts decimal,r16
	lds r17,SCRATCHPAD + 1
	lds r18,SCRATCHPAD 
	sbrc r18,7
	rjmp negativeresult
	ldi r16,0x20
	sts minussign,r16
	mov r19,r17
	andi r19,0x0f
	rcall calc_fraction
	rjmp maindigits
negativeresult:
	clr r19
	lds r17,SCRATCHPAD + 1
	lds r18,SCRATCHPAD 
	com r18
	com r17
	subi r17,-1
;	sbci r18,0
	mov r19,r17
	sts  SCRATCHPAD + 1,r17
	sts	 SCRATCHPAD ,r18
	andi r19,0x0f
	ldi r16,'-'
	sts minussign,r16
	rcall calc_fraction
			
maindigits:
	mov r25,r22					; copy fmultiply results to r26:r25 for ASCII conversion
	mov r26,r23					; copy fmultiply results to r26:r25 for ASCII conversion
	ldi ZL,low(firstplace)
	ldi ZH,high(firstplace)
	ldi r20,4					; length of string 4 as we dont want space loaded at 4th position if leading zero in decimal fraction
	rcall ASCII_CONVERT
	lds r17,SCRATCHPAD + 1
	lds r18,SCRATCHPAD 
	lsr r18
	ror r17
	lsr r18
	ror r17
	lsr r18
	ror r17
	lsr r18
	ror r17
	mov r25,r17
	mov r26,r18
	ldi ZL,low(firstdigit)
	ldi ZH,high(firstdigit)
	ldi r20,3
	rcall ASCII_CONVERT
	ret


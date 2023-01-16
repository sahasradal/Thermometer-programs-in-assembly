;
; ATTINY13KTYPEthermometer1.asm
;
; Created: 15/01/2023 18:49:14
; Author : Manama
;


.equ fclk = 1000000
.equ OLEDaddress = 0x78
.equ command = 0x00
.equ data_cmd = 0x40
.equ OLED_INIT_LEN = 15    
.equ font_length = 16
.def temp = r16
.def data = r17
.def address = r25  
.def counter = r18
.def array_length = r19
.def array_byte = r20
.def ASCII = r21
.def temp1 = r22
.def posx = r23
.def posy = r24



.macro millis
ldi temp,@0
rcall delayTx1mS
.endm

.macro pointer
ldi ZL,low(2*@0)		;load low value of OLED_INIT_BYTES address to ZL
ldi ZH,high(2*@0)		;load high value of OLED_INIT_BYTES address to ZH
.endm

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



.dseg
 PAD:  .byte 1	
 PAD1: .byte 1	
 PAD2: .byte 1
 PAD3: .byte 1
 OLdigit: .byte 1
 TTdigit: .byte 1
 Tdigit: .byte 1
 Hdigit: .byte 1
 point: .byte 1
 tendigit: .byte 1
 ONE: .byte 1
 gap: .byte 1
 CEE: .byte 1




.cseg
	ldi r16,'.'					; load decimal point
	sts point,r16				; store in sram point
	ldi r16,'C'					; load font "C"in r16
	sts CEE,r16					; store in SRAM for later printing on OLED
	ldi r16,0x20				; load 0x20 or space in r16
	sts gap,r16					; store in SRAM for printing on OLED
	ldi r16,0b00011000
	out ddrb,r16				; store is ddr register 
	sbi PORTB,PORTB3			; SS HI ,set BIT3,
	ldi address,OLEDaddress		; load in address register OLED display I2C adress
	rcall i2c_command_write		; call 12c write subriutine
	rcall OLED_INIT				; call OLED initialization routine
	ldi array_length,OLED_INIT_LEN		;length of OLED init array 12 loaded
	rcall array_read			; call array_read subroutine to send OLED init to display via 12c
	rcall i2c_stop				; call 12c stop
	rcall clear_OLED			; call OLED screen clear routine. writes 0x00 in all OLED GDRAM address 128*32
	rcall i2c_stop				; stop I2C transmission
read:
	rcall MAXREAD			; call routine to read MAX6675 , result in r20:r19
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
	ldi address,OLEDaddress		; load in address register OLED display I2C adress
	cursor 0,0				; set OLED cursor to coordinates 0,0
	ldi r16,9				; load r16 , 9
	mov r10,r16				; move 9 to r10. this serves as counter for SRAM location 
	ldi YL,low(OLdigit)		; load address of first digit OLdigit in SRAM
	ldi YH,high(OLdigit)
printloop:
	ld ASCII,Y+				; load ASCII with data in address pointed by Y register
	rcall printchar			; call sub routine printchar which prints ascii chars stored in SRAM 
	dec r10					; decrease SRAM location counter
	brne printloop			; loop till counter r10 is 0
	rcall ms250				; 250ms delay
	rcall ms250				; 250ms delay
	rjmp read				; jump to label read to repeat

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;I2C ROUTINES     register used- temp/data/counter = r16/r17/r18                   ;
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
	in temp,ddrb
	andi temp,0xFA
	out ddrb,temp	
	in temp,portb
	andi temp,0xFA		
	out portb,temp
	ret

i2c_start:
	rcall sda_low			;I2C start condition needs SDA low before SCL goes low
	rcall scl_low
	mov data,address	;slave address is copied to data register to initiate communication.integrated with subroutine to reduce code repetition.
	rcall i2c_write			;write function for i2c
	ret

i2c_stop:
	rcall sda_low			;I2C needs SCL to go high and then SDA to high to indicate stop transmission.SDA is pulled low to make sure.
	rcall scl_high
	rcall sda_high
	ret

i2c_ack:
	sbi ddrb,ddb0			; sda low for ack
	cbi ddrb,ddb2           ; scl high
	nop						; delay
	sbi ddrb,ddb2			; scl low
	cbi ddrb,ddb0			; sda high 
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
	rcall scl_low			;ready for continuation of data transfer unless a stop is called
	rcall sda_low
	ret


	; ============================== Time Delay Subroutines =====================

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
	ldi     YL,low(((fclk/1000)-18)/4)     ; [1] delay counter              (((fclk/1000)-18)/4)
    ldi     YH,high(((fclk/1000)-18)/4)    ; [1]                            (((fclk/1000)-18)/4)

delay1mS_01:
    subi    YL,low(1)						; [2] update the the delay counter
	sbci    YH,high(1)
    brne    delay1mS_01                     ; [2] delay counter is not zero

; arrive here when delay counter is zero
    pop     YH                              ; [2] restore registers
    pop     YL                              ; [2]
	ret                                     ; [4]

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
ms250:							; 250ms dealy routine
	millis 250
	ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

OLED_INIT:
	ldi ZL,low(2*OLED_INIT_BYTES)		;load low value of OLED_INIT_BYTES address to ZL
	ldi ZH,high(2*OLED_INIT_BYTES)		;load high value of OLED_INIT_BYTES address to ZH
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
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
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
loop2:
	lpm array_byte,Z+
	mov data,array_byte
	rcall i2c_write
	dec array_length
	brne loop2
	ret


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;PRINT function   uses registers- temp1,temp,array_byte,ASCII,array_length
;characters to be printed has to be passed into register ASCII as ASCII values
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
printchar:	
	cpi ASCII,'.'		; decimal
	breq decimal1
	cpi ASCII,'C'		; C
	breq letterC
	cpi ASCII,' '		; decimal
	breq spacee
	rjmp print
decimal1:
	ldi ZL,low(2*fontdecimal)	;attiny10 doesnt allow direct flash read,should access via SRAM address starting at 0x4000 as per data sheet
	ldi ZH,high(2*fontdecimal)
	rjmp ASCII0
spacee:
	ldi ZL,low(2*fontspace)		;attiny10 doesnt allow direct flash read,should access via SRAM address starting at 0x4000 as per data sheet
	ldi ZH,high(2*fontspace)
	rjmp ASCII0
letterC:
	ldi ZL,low(2*fontC)			;attiny10 doesnt allow direct flash read,should access via SRAM address starting at 0x4000 as per data sheet
	ldi ZH,high(2*fontC)
	rjmp ASCII0
print:
	ldi ZL,low(2*font0)			;attiny10 doesnt allow direct flash read,should access via SRAM address starting at 0x4000 as per data sheet
	ldi ZH,high(2*font0)
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
	lpm array_byte,Z+			;load array_byte register with font pointed by Z
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
fontdecimal:
.db 0x00,0x00, 0x00,0x30, 0x00,0x30, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00// .
fontspace:
.db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00                    
fontC:
.db 0xC0,0x07, 0x30,0x18, 0x08,0x20, 0x08,0x20, 0x08,0x20, 0x08,0x10, 0x38,0x08, 0x00,0x00// C


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;CLEAR_OLED     registers array_byte/array_length used  writes 128*4 0's to OLED
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

clear_OLED:
	rcall ccursor
	rcall i2c_data_write
	ldi YL,low(512)
	ldi YH,high(512)
lll:
	ldi data,0x00
	rcall i2c_write
	subi YL,1
	sbci YH,0
	brne lll
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

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
zero_suppress:
	lds r16,OLdigit			; load r16 with contents of SRAM OLdigit
	cpi r16,0x30			; check if it is ascii 0
	breq blank				; if 0 branch to label blank to fill with space character
	ret
blank:
	ldi r16,' '				; load space character in r16
	sts OLdigit,r16			; store in OLdigit sram
	lds r16,TTdigit			; load from sram TTdigit to r16
	cpi r16,0x30			; compare contents with ascii0
	brne exit1				; if not ascii 0 exit procedure
	ldi r16,' '				; load r16 with space
	sts TTdigit,r16			; store in TTdigit if it was 0
	lds r16,Tdigit			; load r16 with contents of Tdigit sram
	cpi r16,0x30			; compare with ascii 0
	brne exit1				; if not ascii 0 exit procedure
	ldi r16,' '				; load r16 with space
	sts Tdigit,r16			; store in Tdigit if it was 0
exit1:
	ret						; return to caller
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

MAXREAD:
	ldi r25,16				; load r25 with 16, 16 clock cycles to spit out 16bit value
	cbi PORTB,PORTB3		; SS low ,clear bit3
rloop:
	sbi PORTB,PORTB4		; SCK HI ,set BIT4,
	nop						; 1 cycle ,1us delay
	nop						; 1 cycle ,1us delay
	nop						; 1 cycle ,1us delay
	nop						; 1 cycle ,1us delay
	clc						; clear carry flag
	sbic PINB,PINB1 		; if MISO pin is low skip next instruction
	sec						; set carry flag
	rol r19					; rotate through carray left r19, carry is loaded into lsb of r19
	rol r20					; rotate through carray left r20, the bit shifted out of r19 is looped through r20
	cbi PORTB,PORTB4		; SCK low ,clear bit4 ,
	dec r25					; decrease counter
	brne rloop				; loop till r25 is 0 , 16 clocks
	sbi PORTB,PORTB3		; SS HI ,set BIT3,
	ret						; return to caller

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; 1) (MAXREAD result)x(25) = temprature in degree centigrade
; each least significant bit of the result equals 0.25 degree as per MAX6675 data sheet , all 0 = 0 degree , all 1's = 0xfff =1023.75
; eg (0xfff) x (25) = 102375 . we place decimal point between 100th and tenth place in OLED and make it 1023.75
; instead of multiplying with 0.25 , we do multiply with 25 and place a decimal point in OLED between 100th and tenth position 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

TEMP_CALC:
		clr r6					; clear r6
		clr r7					; clear r7 
		clr r8					; clear r8
		clr r9					; clear r9
		ldi r16,25				; load r16 with multiplier 25 (0.25 degreeC), we multiply max6675 result with  0.25 degree
calcu:
		add r7,r19				; add r7 with r19 (low byte of max6675 result)
		adc r8,r20				; add with carry r8 and r20 (high byte of MAX6675 result)
		adc r9,r6				; add with carry r9 with r6 ( r6 has 0 in it)
		dec r16					; decrease r16 , we add result with itself 25 times to effect result x 25 = final temprature
		brne calcu				; loop till r16 is 0 , 25 additions done
		ret						; return to caller

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

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
	brsh Dby100000				; if greater or equal than 100000 branch to divide by 100000
	ldi r16,low(10000)			; compare value 10000
	mov r2,r16					; low byte of 10000 in r2
	ldi r16,byte2(10000)
	mov r3,r16					; high byte of 10000 in r3
	ldi r16,byte3(10000)
	mov r4,r16
	cp  r7,r2					; compare result with 10000
	cpc r8,r3					; compare result with 10000
	brsh Dby10000				; if greater or equal to 10000 branch to divide by 10000
	ldi r16,low(1000)			; load compare value 1000
	mov r2,r16					; low compare value 1000 in r2
	ldi r16,byte2(1000)
	mov r3,r16					; high of compare value 1000 in r3
	ldi r16,byte3(1000)
	mov r4,r16
	cp  r7,r2					; compare low byte of result with low of 1000
	cpc r8,r3					; compare with carry to high byte of 1000
	brsh Dby1000				; if greater or equal to 1000 branch to divide by thousand
	ldi r16,low(100)			; load compare value low byte of 100
	mov r2,r16					; copy to r2
	ldi r16,byte2(100)			; load compare value high byte of 100
	mov r3,r16					; copy to r3
	ldi r16,byte3(100)
	mov r4,r16
	cp  r7,r2					; compare low byte of result to low of 100
	cpc r8,r3					; compare with carry the result to high byte of 100
	brsh Dby100					; if compared result is greater than or equal to 100 branch to divide by 100
	ldi r16,low(10)				; load ccompare value 10
	mov r2,r16					; load compare value in r2
	ldi r16,byte2(10)			; load high byte of 10
	mov r3,r16					; load compare value high byte
	ldi r16,byte3(10)
	mov r4,r16
	cp  r7,r2					; compare result to 10
	cpc r8,r3					; compare wit carry if any
	brsh Dby10					; if compared result is greater than or equal to 10 branch to divide by 100
	ldi r16,0x30				; load ascii 0 in r16
	add r16,r7					; add ascii0 with remaining value in r7
	sts ONE,r16					; store the single digit ascii value in SRAM register ONE
	ret

Dby100000:						; divide by 100000
	clr r16						; clear r16 counter
	sub r7,r2					; subtract lsb of 100000 from result
	sbc r8,r3					; subtract with carry 2nd byte of 100000
	sbc r9,r4					; subtract with carry 3rd byte of 100000
	lds r16,OLdigit				; load r16 with value in OLdigit, ascii 0 (0x30) was loaded in SRAM previously
	inc r16						; increase r16
	sts OLdigit,r16				; store back r16 in OLdigit
	rjmp ascii_loop				; jump back to ascii_loop to find the remaining digits
Dby10000:						; divide by 10000
	sub r7,r2					; subtract lsb of 10000 from result
	sbc r8,r3					; subtract with carry 2nd byte of 10000
	sbc r9,r4					; subtract with carry 3rd byte of 10000
	lds r16,TTdigit				; load r16 with value in TTdigit, ascii 0 (0x30) was loaded in SRAM previously
	inc r16						; increase r16
	sts TTdigit,r16				; store back r16 in OLdigit
	rjmp ascii_loop				; jump back to ascii_loop to find the remaining digits
Dby1000:						; divide by 1000
	sub r7,r2					; subtract 1000
	sbc r8,r3					; subtract 1000
	sbc r9,r4					; subtract 1000
	lds r16,Tdigit				; copy Tdigit to r16
	inc r16						; increase r16
	sts Tdigit,r16				; store back in SRAM
	rjmp ascii_loop				; loop back
Dby100:							; divide by 100
	sub r7,r2					; subtract 100
	sbc r8,r3					; subtract 100
	sbc r9,r4					; subtract 100
	lds r16,Hdigit
	inc r16
	sts Hdigit,r16
	rjmp ascii_loop
Dby10:							; divide by 10
	sub r7,r2					; subtract 10
	sbc r8,r3					; subtract 10
	sbc r9,r4					; subtract 10
	lds r16,tendigit
	inc r16
	sts tendigit,r16
	rjmp ascii_loop
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;



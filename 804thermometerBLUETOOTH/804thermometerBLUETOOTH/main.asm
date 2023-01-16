;
; 804thermometerBLUETOOTH.asm
;
; Created: 04/05/2022 15:08:32
; Author : Manama
;


;All data and commands are transmitted least significant bit first over the 1-Wire bus.
;PA3 = sensor data
;PA1 = data
;PA2 = clock

; Replace with your application code
.equ fclk = 10000000

.DEF SLAVE_REG = R17
.DEF TEMP = R16
.def data = r19
.def counter = r25
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
.equ BAUD = 9600
.equ fBAUD = ((64 * fclk) /(16 * BAUD)+0.5)
.def counter10k = r5
.def counter1k = r6
.def counter100 = r7
.def counter10 = r8
.def counter1 = r9

.equ displayon  = 0b10001001		; command 0x80 + display on = 8 + brightness 1 =0x89
.equ displayoff = 0b10000001
.equ address1 = 0xC2
.equ address2 = 0xC1
.equ address3 = 0xC0
.equ address4 = 0xC5
.equ address5 = 0xC4
.equ address6 = 0xC3



.macro millis
ldi YL,low(@0)
ldi YH,high(@0)
rcall delayYx1mS
.endm

.macro micros
ldi XL,low(@0-1)  ;1
ldi XH,high(@0-1) ;1
rcall us		  ;2
.endm

.macro DALLAS_COMMAND
ldi r22,@0
rcall ROM_COMMANDS
.endm

.macro F_COMMANDS
ldi r22,@0
rcall f_command
.endm

.macro command2
ldi data,@0
rcall spitx
;ret
.endm

.macro datatx
ldi data,@0
rcall spitx
rcall stop
.endm

.macro BLETX
ldi ZL,low(2*@0)
ldi ZH,high(2*@0)
rcall TXstring1
.endm

.dseg
PAD: .BYTE 1
PAD1: .byte 1
presence: .byte 1
ROM1: .byte 8
ROM2: .byte 8
SCRATCHPAD: .byte 9
minussign: .byte 1
firstdigit: .byte 1
seconddigit: .byte 1
thirddigit: .byte 1
;decimal: .byte 1
firstplace: .byte 1
secplace: .byte 1
thirdplace: .byte 1
fourthplace: .byte 1

.cseg
reset:

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;PROTECTED WRITE (processor speed set to 10MHZ)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;ldi r16,0b01111110  ; osclock =0 bit7  and bit 1:0 = 0x2 for 20mhz ,all reserved bits to ber witten as 1.
;out OSCCFG,r16

	ldi r16,0Xd8
	out CPU_CCP,r16
	ldi r16,0x01
	STS CLKCTRL_MCLKCTRLB,R16
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	rcall UART_SETUP			; call subroutine to setup UART engine
	rcall BLE_SETUP				; Call subroutine to setup Bluetooth low energy HM10 module
	ldi r16,0b00000110			; load r16 0x06
	sts VPORTA_DIR,r16			; set portb  direction register PA1,PA2 as outputs
	sts VPORTA_OUT,r16			; set portb  to high by sourcing , TM1637 DIO & CLK lines high to start

measure:
	rcall dallas_init			; call subroutine to initiate DS18B20
	DALLAS_COMMAND skip_rom		; call subroutine to send skip rom using DALLAS_COMMAND macro , skiprom is the parameter
	F_COMMANDS convert			; call subroutine to send function commands, F_COMMANDS macro , convert is the parameter
	rcall ms750					; sensor takes 700ms to convert temprature with 12bit resolution (default value)
check1:
	rcall bit_read				; call bit read function, if convert finished sensor pulls line low (0) else high (1)
	sbrs r24,7					; check r24 for bit status read and loaded by bit_read function
	rjmp check1					; if msb of r24 is set loop to check1 label until msb becomes 0 (1 means conversion still going on-sensor busy)
	rcall dallas_init			; call sensor init is required before and after each operation
	DALLAS_COMMAND skip_rom		; call subroutine to send skip rom using DALLAS_COMMAND macro , skiprom is the parameter, skiprom bypasses rom access
	F_COMMANDS read_sscratchpad	; send read_scratchpad command with F_COMMAND macro, this reads the earlier converted temprature from the sensor ram lsb & lsb+1 is temprature
	rcall byte_read				; call read_byte subroutine to read scratch pad ( total 9 bytes with lsbyte and lsbit first transmitted from sensor)		
	sts SCRATCHPAD + 8, r24		; store LSB available in r24 to SRAM register SCRATCHPAD
	rcall byte_read				; read LSB + 1 from sensor
	sts SCRATCHPAD + 7, r24		; store LSB available in r24 to SRAM register SCRATCHPAD
	rcall byte_read				; read LSB + 2 from sensor
	sts SCRATCHPAD + 6, r20		; store LSB available in r24 to SRAM register SCRATCHPAD
	rcall byte_read				; read LSB + 3 from sensor
	sts SCRATCHPAD + 5, r24		; store LSB available in r24 to SRAM register SCRATCHPAD
	rcall byte_read				; read LSB + 4 from sensor
	sts SCRATCHPAD + 4, r24		; store LSB available in r24 to SRAM register SCRATCHPAD
	rcall byte_read				; read LSB + 5 from sensor
	sts SCRATCHPAD + 3, r24		; store LSB available in r24 to SRAM register SCRATCHPAD
	rcall byte_read				; read LSB + 6 from sensor
	sts SCRATCHPAD + 2, r24		; store LSB available in r24 to SRAM register SCRATCHPAD
	rcall byte_read				; read LSB + 7 from sensor
	sts SCRATCHPAD + 1, r24		; store LSB available in r24 to SRAM register SCRATCHPAD
	rcall byte_read				; read MSB  from sensor
	sts SCRATCHPAD , r24		; store LSB available in r24 to SRAM register SCRATCHPAD
	rcall DALLAS_INIT			; call sensor init is required before and after each operation
	rcall binary_decimal		; call subroutine to convert binary value of SCRATCHPAD + 8 & SCRATCHPAD + 7 SRAM registers to decimal format and convert to ASCII characters
	ldi r16, 'C'				; we use the secplace in SRAM to store ascii C , this place already has fraction value we are not using , we display 1st fraction only
	sts secplace,r16			; store C in secplace SRAM
	ldi YL,low(minussign)		; set Y pointer to SRAM minussign
	ldi YH,high(minussign)		; set Y pointer to minussign
	ldi r27,6					; load 6 as the counter value, we display only 6 characters eg -100.9C if not minus 100.9C
	rcall command1				; send data command for fixed address (auto increment has a staggered sequence physically)(display address is 0xc2,0xC1,0xC0,0xC5,0xC4,0xC3 from left to right on the display)
	command2 address1			; send address to which data following will be written (address1 = 0xC2)
	ld r16,Y+					; ld from SRAM value in minussign to r16 and increase address pointer
	rcall printchar				; call printchar to calculate font address and trannsmit to screen
	dec r27						; decrease r27 (used to decide where the decimal point will light up inside printchar routine)
	command2 address2			; send address to which data following will be written (address1 = 0xC1)
	ld r16,Y+
	rcall printchar
	dec r27
	command2 address3			; send address to which data following will be written (address1 = 0xC0)
	ld r16,Y+
	rcall printchar
	dec r27
	command2 address4			; send address to which data following will be written (address1 = 0xC5)
	ld r16,Y+
	rcall printchar
	dec r27
	command2 address5			; send address to which data following will be written (address1 = 0xC4)
	ld r16,Y+
	rcall printchar
	dec r27
	command2 address6			; send address to which data following will be written (address1 = 0xC3)
	ld r16,Y+
	rcall printchar
	dec r27
	rcall command3				; sequence ends with command3 which turns on the display
	rcall TXUSART				; call subroutine to transmit the values stored as ASCII characters to UART terminal			
	rjmp measure				; infinite loop
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;DS18B20 init sequence
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

dallas_init:
	cbi VPORTA_OUT,3	;pin porta 3 is pulled down by clearing port register
	sbi VPORTA_DIR,3	;porta 3 is output by setting direction register
	micros 480			;minimum time pull down time required for reset
	cbi VPORTA_DIR,3	;change PA3 to input to verify whether DS1B20 pulls the line low to indicate presence (PA3 is tristated)
	micros 70			;sensor should respond after 60 seconds ,so wait for the minimum time
	in r16,VPORTA_IN	;copy the PA3 PIN value from PORTA_PIN register
	andi r16,0b00001000	;AND with 0x08 the value of r16 ,if 0 sensor responded , if 1 no response at the time of checking
	sts presence,r16	;copy value to memory
	micros 410			; wait for the remaining time slot of 460 us
	nop
	ret
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;writes 1 to DS18B20
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

write1:
	sbi VPORTA_DIR,3	;porta 3 is output by setting direction register
	micros 6			;delay of 3us data line stays low for more than minimum required time
	cbi VPORTA_DIR,3	;data line released by clearing direction register bit. line is pulled up by pullup voltage
	micros 64			;The delay of 60us to finish the time slot as per data sheet
	ret
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;writes 0 to DS18B20
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

write0:
	sbi VPORTA_DIR,3	;porta 3 is output by setting direction register
	micros 60			;delay 60us
	cbi VPORTA_DIR,3	;data line released by clearing direction register bit. line is pulled up by pullup voltage
	micros 10			;consume the remaining time slot
	ret
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;reads a byte from DS18b20
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
byte_read:
	ldi r25,8			;load counter r25 with 8 (8 bits to be read from the sensor)
	clr r24				;r24 is the register where byte is accumulated
loop3:
	sbi VPORTA_DIR,3	;porta 3 is output by setting direction register
	micros 6			;delay of minimum 1 us as per data sheet
	cbi VPORTA_DIR,3	;clear direction register to release data line and turn the master to input
	micros 9			;wait 10us till the DS18B20 responds by placing a bit on the data line
	in r16,VPORTA_IN	;copy PORTA pin value to r16
	andi r16,0b00001000	;AND r16 with 0x08 
	brne one			;if not 0 branch to label one
	clc					;clear carry bit in SREG
	ror r24				;rotate right through carry (0 is shifted into msb of r20 from carry,ds18B20 transmits lsb of LSBYTE firt.At the end the first bit in MSB reaches LSB
	micros 55			;wait for the remaining time slot of the read bit
	dec r25				;decrease counter
	brne loop3			;if counter not 0 branch back to loop3
	ret
one:
	sec					;set carry bit in SREG
	ror r24				;rotate through carry and the carry bit1 will be shifted to MSB of r20 , bit7 >>>>>>> bit0
	micros 55			;wait for the remaining time slot of the read bit
	dec r25				; counter	;wait for the remaining time slot of the read bit
	brne loop3			;if counter not 0 branch back to loop3
	ret
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;reads 1 bit from DS18b20
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

bit_read:
	ldi r25,1			;subroutine used to read 1 bit from sensor, so set counter to 1
	clr r24				; clear accumulator r24
	rcall loop3			; call loop3 which is part of byte_read subroutine 
	andi r24,0x80 		; and r24 with 0x80 , if bit is 1 msb stays 1 else all 0
	ret

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

; ---------------------------------------------------------------------------

us1:
	nop  ; 1 cyc (0.0000001 sec = 100 nano ; 1us = 1000ns)
	nop	 ; 1
	nop  ; 1
	nop  ; 1
	ret  ; call 2 cyc, ret 4 cyc

us:
	nop	 ;1
	nop  ;1
	nop  ;1
	nop  ;1
	nop  ;1
	nop  ;1
	sbiw XH:XL,1	;2
	brne us		;2  (1 for last iteration)
	ret		    ;4 + 2  for ret and call , (takes 13 cycles(1.3us) total + loop), delayvalue-2 is to makeup for the extra 3cycles.If 10us is needed delay value of 9 is loaded


ms1:
		millis 1
		ret
ms10:
		millis 10
		ret
ms50:
		millis 50
		ret
ms100:
		millis 100
		ret
ms500:
		millis 500
		ret
ms750:
		millis 750
		ret
ms1000:
		millis 1000
		ret

; ============================== End of Time Delay Subroutines ==============---------------------------------------------------------------------------------
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

ROM_COMMANDS:
	ldi r25,8				; load counter r25 value 8
	rcall dallas_init		; call sensor init routine to initiate 
	lds r16,presence		; copy SRAM register presence whoes value indicates whether sensor acknowledged init request in previous line
	cpi r16,0				; compare value to 0
	brne exit0				; if any value other than 0 sensor failed to acknowledge ,abort by branching to exit
	mov r24,r22				; if value is 0 , copy command in r22 loaded by ROM_COMMAND macro to r24
loop4:
	lsr r24					; right shift r24 , shifted out bit goes to carry in SREG
	brcs high1				; if carry set bit shifted out bit is 1 , branch to label high1 where a cal to write1 subroutine is made
	rcall write0			; if carry is not set than shifted out bit is 0, call routine write0 to send a zero to sensor			
	dec r25					; decrease counter r25
	brne loop4				; if r25 is not 0 bits remaining to be transmitted , loop back to loop4 to send remaining bits
	ret
high1:
	rcall write1			; call routine to send a 1 to the sensor
	dec r25					;	decrease counter r25
	brne loop4				; if r25 is not 0 bits remaining to be transmitted , loop back to loop4 to send remaining bits
exit0:	ret
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

f_command:
	ldi r25,8				; load counter r25 value 8
	mov r24,r22				; copy command in r22 loaded by F_COMMAND macro to r24
loop5:
	lsr r24					; right shift r24 , shifted out bit goes to carry in SREG
	brcs high2				; if carry set bit shifted out bit is 1 , branch to label high1 where a cal to write1 subroutine is made
	rcall write0			; if carry is not set than shifted out bit is 0, call routine write0 to send a zero to sensor
	dec r25					; decrease counter r25
	brne loop5				; if r25 is not 0 bits remaining to be transmitted , loop back to loop5 to send remaining bits
	ret
high2:
	rcall write1			; call routine to send a 1 to the sensor
	dec r25					; decrease counter r25
	brne loop5				; if r25 is not 0 bits remaining to be transmitted , loop back to loop5 to send remaining bits
	ret


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Multiply of two 16bits numbers with 16bits result.
; USAGE
; r17:r16 = r23:r22 * r21:r20
; Register usage: r0, r1 and r16 to r23 (8 registers)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
                                    
mul16x16_16:
	mul	r22, r20		; al * bl
    movw	r17:r16, r1:r0
    mul	r23, r20		; ah * bl
    add	r17, r0
    mul	r21, r22		; bh * al
    add	r17, r0
    ret
                                    
                                    
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

binary_decimal:
	lds r17,SCRATCHPAD + 8			;copy LSB of temprature value from SRAM to r17
	lds r18,SCRATCHPAD + 7			;copy MSB of temprature value from SRAM to r17
	sbrc r18,7						;check firts bit of MSB, if 0 positive value if 1 negative value
	rjmp negativeresult				; if negative value jump to label negativevalue else proceed to next instruction
	ldi r16,' '						; load r16 ASCII blank/space , do this if result is positive , by loading blank the minus sign is not displayed
	sts minussign,r16				; store blank in minus register in the SRAM
	mov r19,r17						; copy the  lsb to r19 for dealing with 4 bit fraction value
	andi r19,0x0f					; anf r19 wit 0x0f leaving only teh lower nibble
	ldi r22,low(625)				; as per data sheet each fraction bit has a weight of 0.0625 degree centigrade, we will multiply with 625 ,fixed point calculation
	ldi r23,high(625)				; we use 625 as a scaled value to multiply the nibble and place decimal point as needed (multiplicand)
	mov r20,r19						; copy the anded value to r20 which is the multiplier
	clr r21							; clear r21 , high byte of multiplier (16 bit multiply rutine will be called in the next line)
	rcall mul16x16_16				; result in r17:r16
	rcall bin2ascii					; call bin2ascii subroutine which will convert the binary value of the decimal fraction value to 5 ascii characters
	sts firstplace,counter1k		; store the 1000th position in SRAM register firstplace
	sts secplace,counter100			; store the 100th position in SRAM register secplace
	sts thirdplace,counter10		; store the 10th position in SRAM register thirdplace
	sts fourthplace,counter1		; store the 1st position in SRAM register fourthplace
	rjmp maindigits

negativeresult:
	clr r19							; clear r19 of any previous values
	lds r17,SCRATCHPAD + 8			; copy LSB of temprature value from SRAM to r17
	lds r18,SCRATCHPAD + 7			; copy MSB of temprature value from SRAM to r17
	com r18							; perform complement of 1 operation on msb (negative values are stored as 2 's complement)
	com r17							; perform complement of 1 operation on msb
	subi r17,-1						; add 1 to 1's complement to get original positive number
	sts SCRATCHPAD + 8,r17			; copy back the corrected number to the SRAM register where 2's complement was stored
	sts SCRATCHPAD + 7,r18			; copy back the corrected number to the SRAM register where 2's complement was stored
	mov r19,r17						; copy the  lsb to r19 for dealing with 4 bit fraction value
	andi r19,0x0f					; and r19 wit 0x0f leaving only teh lower nibble
	ldi r16,'-'						; load r16 with ascii - sign
	sts minussign,r16				; store - sign in SRAM minus register to indicate negative values
	ldi r22,low(625)				; as per data sheet each fraction bit has a weight of 0.0625 degree centigrade, we will multiply with 625 ,fixed point calculation
	ldi r23,high(625)				; we use 625 as a scaled value to multiply the nibble and place decimal point as needed (multiplicand)
	mov r20,r19
	clr r21
	rcall mul16x16_16				; result in r17:r16
	rcall  bin2ascii				; call bin2ascii subroutine which will convert the binary value of the decimal fraction value to 5 ascii characters
	sts firstplace,counter1k		; store the 1000th position in SRAM register firstplace
	sts secplace,counter100			; store the 100th position in SRAM register firstplace
	sts thirdplace,counter10		; store the 10th position in SRAM register firstplace
	sts fourthplace,counter1		; store the 1st position in SRAM register firstplace

maindigits:
	lds r17,SCRATCHPAD + 8			; copy the result LSB to r17
	lds r18,SCRATCHPAD + 7			; copy the result MSB to r18
	lsr r18							; right shift msb
	ror r17							; rotate right lsb
	lsr r18							; right shift msb
	ror r17							; rotate right lsb
	lsr r18							; right shift msb
	ror r17							; rotate right lsb
	lsr r18							; right shift msb
	ror r17							; rotate right lsb , now the fraction nibble is discarded and value in r17 is the whole number result between -55C to 125 C
	mov r16,r17						; copy LSB to r16 (input for ascii routine)
	mov r17,r18						; copy MSB to r17 (input for ascii routine)
	rcall bin2ascii					; call binary to ascii conversion routine for the main digits ( we converted and stored fraction earlier)
	rcall zero_suppress				; call routine to suppress the leading zeros with space/blanks
	sts firstdigit,counter100		; store 100 position didgit in SRAM firstdigit
	sts seconddigit,counter10		; store 10 th position digit in sram seconddigit
	sts thirddigit,counter1			; store 1st position digit in sram thirddigit
	ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;* Integer to ASCII converter subrountine here ,r17:r16 is the input registers
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; 16 bit binary to 5 digit ASCII conversion
; the 16 bit value is in bytes nh & nl r17:r16 ,temp r18
; each of the 5 ascii digits is stored (sent) to a separate memory located (UART xmt buffer)
;NOTE---the data storage is not yet in this code
;NOTE this code HAS BEEN tested & seems to work well---NO REFUNDS ARE POSSIBLE
; 10000 base 10=$2710, 1000 base 10 =$03E8, 100 base 10=0064, 10 base 10= $000A
;
bin2ascii:
         ldi r18, $2F ;Init ASCII conversion ('0'-1)
loop10k: inc r18      ;count up 10k "counts"
         subi r16, $10
         sbci r17, $27
         brcc loop10k  ;IF >=10000 THEN subtract again
         subi r16, $F0  ;subtract -10000 (add 10000)
         sbci r17, $D8
end10k:  mov counter10k,r18 ;do store 10k char now

         ldi r18, $2F ;Init ASCII conversion ('0'-1)
loop1k:  inc r18      ;count up 1k "counts"
         subi r16, $E8
         sbci r17, $03
         brcc loop1k   ;IF >=1000 THEN subtract again
         subi r16, $18  ;subtract -1000 (add 1000)
         sbci r17, $FC
end1k:  mov counter1k,r18;do store 1k char now

         ldi r18, $2F ;Init ASCII conversion ('0'-1)
loop100: inc r18      ;count up 100 "counts"
         subi r16, $64
         sbci r17, $00
         brcc loop100  ;IF >=100 THEN subtract again
         subi r16, $9C  ;subtract -100 (add 100)
         sbci r17, $FF
end100:  mov counter100,r18;do store 100 char now

;nh==0 so ignore nh from here

         ldi r18, $2F ;Init ASCII conversion ('0'-1)
loop10:  inc r18      ;count up 10 "counts"
         subi r16, $0A
         brcc loop10   ;IF >=10 THEN subtract again
         subi r16, $F6  ;subtract -10 (add 10)
end10:   mov counter10,r18;do store 10's digit

         subi r16, $D0  ;convert 1's digit to ASCII
         mov counter1,r16;do store 1's digit (value held by nl)
		 ret


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

zero_suppress:
	push r16
	ldi r16,0x30
	mov r4,r16
	pop r16
	cp counter10k,r4
	breq blank
	ret
blank:
	ldi r16,' '
	mov counter10k,r16
	cp counter1k,r4
	brne exit1
	ldi r16,' '
	mov counter1k,r16
	cp counter100,r4
	brne exit1
	ldi r16,' '
	mov counter100,r16
	cp counter10,r4
	brne exit1
	ldi r16,' '
	mov counter10,r16
exit1:
	ret
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

command1:
	ldi data,0x44		;0x40 commannd for automatic address increment,0x44 for fixed address mode (address has to be supplied for each digit)
	rcall spitx			;call spitx to transmit command to TM1637 DISPLAY
	rcall stop			;a stop has to be issued as per data sheet for data command setting (0x40 or 0x44 above)
	ret
command3:
	ldi data,displayon	;(load display command) 0x80 OR 0x01(brightness value bit0 to bit2,we use 1) OR 0x08(display on) = 0x89
	rcall spitx			;call spitx to transmit command to TM1637 DISPLAY
	rcall stop			;a stop has to be issued as per data sheet for address command if fixed mode is chosen
	ret

start:					;TM1637 display commuication "start" routine
	sbi VPORTA_OUT,1	;set data PA1 high
	sbi VPORTA_OUT,2	;set clock PA2 high
	rcall delay10uS		;call 10us delay as TM1637 needs slow clock
	cbi VPORTA_OUT,1	;clear data PA1 (data low before clock low)
	rcall delay10uS		;call 10us delay as TM1637 needs slow clock
	cbi VPORTA_OUT,2	;clear clock PA2 (clock low after data low) to complete start sequence
	rcall delay10us		;call 10us delay as TM1637 needs slow clock
	ret
stop:					;TM1637 display commuication "stop" routine
	cbi VPORTA_OUT,1	;clear data PA1 low
	cbi VPORTA_OUT,2	;clear clock PA2 low
	rcall delay10uS		;call 10us delay as TM1637 needs slow clock
	sbi VPORTA_OUT,2	;set clock PA1 (clock high before data high)
	rcall delay10us		;call 10us delay as TM1637 needs slow clock
	sbi VPORTA_OUT,1	;set data PA1 (data high after clock high)
	rcall delay10us		;call 10us delay as TM1637 needs slow clock
	ret

SPI:					;proc to send a byte to TM1637 7segment DISPLAY
	ldi r18,8			;bit counter , 8 = 8 bits
tx:
	lsr data			;The data register is shifted to rhs (LSB first) as TM1637 expects data LSB 1st (opposite to I2C)
	brcs hi				;branch if carry set in SREG (if shifted bit is 1)
	cbi VPORTA_OUT,1	;clear PA1 to pull data low (shifted bit is 0)
	rcall delay10us		;call 10us delay as TM1637 needs slow clock
	sbi VPORTA_OUT,2	;push clock high to clockin data
	rcall delay10us		;call 10us delay as TM1637 needs slow clock
	cbi VPORTA_OUT,2	;clear PA2 to pull clock low
	rcall delay10us		;call 10us delay as TM1637 needs slow clock
	dec r18				;decrease bit counter
	brne tx				;if bit counter not 0 loop back
	sbi VPORTA_OUT,1	;set PA1 to pull data high ,release data so that TM1637 can ACK
	sbi VPORTA_OUT,2	;push clock high to clockin ACK, TM1637 will release the data line only on receiving 9th clock
	rcall delay10us		;call 10us delay as TM1637 needs slow clock
	cbi VPORTA_OUT,1	;set data low before clock will be pulled down
	cbi VPORTA_OUT,2	;pull low clock line to complete 9th clock
	ret
hi:
	sbi VPORTA_OUT,1	;set PA1 to pull data high (bit shifted is 1)
	rcall delay10us		;call 10us delay as TM1637 needs slow clock
	sbi VPORTA_OUT,2	;push clock high to clockin data
	rcall delay10us		;call 10us delay as TM1637 needs slow clock
	cbi VPORTA_OUT,2	;clear PA2 to pull clock low
	rcall delay10us		;call 10us delay as TM1637 needs slow clock
	dec r18				;decrease bit counter
	brne tx				;if bit counter not 0 loop back
	sbi VPORTA_OUT,1	;set PA1 to pull data high ,release data so that TM1637 can ACK
	sbi VPORTA_OUT,2	;push clock high to clockin ACK, TM1637 will release the data line only on receiving 9th clock
	rcall delay10us		;call 10us delay as TM1637 needs slow clock
	cbi VPORTA_OUT,1	;set data low before clock will be pulled down
	cbi VPORTA_OUT,2	;pull low clock line to complete 9th clock
	ret
	
spitx:
	rcall start
	rcall SPI
	ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;fonts  , 7 segment display sequence
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

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
fontminus: .db  0b01000000  // -
fontspace: .db  0b00000000

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;printchar ;;;loads correct ccharacters by interpreting ASCII values and calculating the necessary 7 segment byte
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

printchar:
	cpi r16,'-'		; minus
	breq minus
	cpi r16,'C'
	breq letterC
	cpi r16,' '
	breq space
	rjmp printnum
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
	ldi ZL,low(2*font0)
	ldi ZH,high(2*font0)
	ldi r26,0x30   				; load 0x30 in r26
	sub r16,r26					; compare value in r16 to r26
	breq load0					; if equal/0 branch to load0 label which lods font data from font0 label, Z pointer set to font0 at the begining of 'printnum'
	clr r26						; if subtraction is not equal to 0 , need to calculate address, clear r26. the value in this register after calculation decides address
multiply:
	subi r30,low(-2)			; add 16 to  Z low	(-2 because assembler pads  0x00 with  each font for alignment)
	sbci r31,high(-2)			; add with carry to Z high
	inc r26						; increase r26 which indicates how many times 16 was added
	cp r26,r16					; compare the value in r26 after addition to r16 r16 holds the difference between the font 0 and required font
	brne multiply				; if value differ again add until bot r16 and r26 are same
load0:
	lpm data,Z					; call routine font_write which prints font on LCD
	cpi r27,3					; this number decides where the decimal point will be placed , 3 means decimal will be placed on 4th digit from left most digit
	breq adddecimal				; if equal to 3 branch to adddecimal label to OR with 0x80 which will turn on the decimal point
	rcall spitx					; transmit data to TM1637 screen
	rcall stop					; transmit stop to screen
	ret
adddecimal:
	ori data,0x80				; OR 0x80 to the value in data to light up the decimal point, 0x80 is the decimal point segment data
	rcall spitx					; transmit data to TM1637 screen
	rcall stop					;transmit stop to screen
	ret


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


UART_SETUP:
	lds r16,PORTB_DIR	
	ori r16,(1<<2)
	sts PORTB_DIR,r16			;set portB PB2 direction as output
	lds r16,PORTB_OUT
	ori r16,(1<<2)
	sts PORTB_OUT,r16			;set portB PB2 as 1 or +ve
	ldi r16,low(fBAUD)			;load low value of fBAUD as calculated in the formula provided above
	ldi r17,high(fBAUD)			;load high value of fBAUD as calculated in the formula provided above
	sts USART0_BAUD,r16			;store low fBAUD in BAUD register
	sts USART0_BAUD + 1,r17		;store low fBAUD in BAUD register
	ldi r16,USART_TXEN_bm	    ;0b01000000 ,(1<<6) ,bitmask value to enable USART transmit 
	sts USART0_CTRLB,r16		;store TXEN in USART_CTRLB register
;	rjmp TXUSART				;jump to TXUSART address (main routine)
	ret

sendbyte:
	lds r16,USART0_STATUS		;copy USART status register to r16
	andi r16,USART_DREIF_bm     ;(0b00100000) AND with DATA REGISTER EMPTY FLAG bitmask (position5) to check flag status 0= not empty 1= empty 
	sbrs r16,5					;skip next instruction if bit 5 is 1 (means flag set for data transmit buffer ready to receive new data )
	rjmp sendbyte				;if DREIF = 0 ,bit 5 in r16 is 0 then loop back to sendbyte until DREIF = 1
	mov r16,r24					;copy data to be transmitted from r20 to r16
	sts USART0_TXDATAL,r16		;store r16 in TXDATAL transmit data low register 
	ret


send_temp:
	ldi YL,low(minussign)		; set pointer Y to SRAM location minus which is the starting point of the converted ASCII values holding temprature
	ldi YH,high(minussign)
	ldi r25,7					; load counter value 7 , -125.1C is the format , if positive leading space. last 3 fraction values we ignore for display, C is inserted in secplace SRAM inside "measure"routine.
loop:
	cpi r25,3					; if counter value reaches 3, 4 characters have already been transmitted, we need to insert decimal point ti match our format
	breq insert
	ld r24, Y+					; load r24 with value pointed by Y pointer and increase pointer to next address
insert1:
	rcall sendbyte				; call routine to send byte to UART engine
	dec r25						; decrease counter
	breq exit					;if null/0x00 the end of string has reached
	rjmp loop					;jump to loop back through the string until a null is encountered
	rcall ms10					; wait time of 10ms
exit:ret
insert:
	ldi r24,'.'
	rjmp insert1

TXstring:
	ldi ZL,low(2*string)
	ldi ZH,high(2*string)	
loop2:
	lpm r24,Z+
	cpi r24,0x00
	breq exit2					;if null/0x00 the end of string has reached
	rcall sendbyte
	rjmp loop2					;jump to loop back through the string until a null is encountered
exit2:ret


TXstring1:
	lpm r24,Z+
	cpi r24,0x00
	breq exit21					;if null/0x00 the end of string has reached
	rcall sendbyte
	rjmp TXstring1				;jump to loop back through the string until a null is encountered
exit21:ret


TXUSART:
	rcall TXstring				;call routine to transmit a null terminated string
	rcall send_temp				; call routine send temp which reads values in SRAM and sends to UART engine
	ldi ZL,low(2*string2)		; string 2 is nothing but carriage feed and line feed for temprature values
	ldi ZH,high(2*string2) 
	rcall loop2
	ret

string: .db "TEMPERATURE", '\r', '\n' , 0
string2:.db " ", '\r', '\n' , 0
AT: .db "AT", '\n', '\r' , 0
role: .db "AT+ROLE0", '\r', '\n' , 0
service: .db "AT+UUIDFFE0", '\r', '\n' , 0
char: .db "AT+CHARFFE1", '\r', '\n' , 0
name: .db "AT+NAMESAJEEV", '\r', '\n' , 0

BLE_SETUP:
	BLETX AT
	rcall ms500
	BLETX role
	rcall ms500
	BLETX service
	rcall ms500
	BLETX char
	rcall ms500
	BLETX name
	rcall ms500
	ret















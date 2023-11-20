;
; homework2 slave.asm
;
; Created: 11/16/2023 3:03:13 PM
; Author : Duc Anh
;
	;.org 0x40
;Keypad I/O initialize
;C3-C0 connect to PA3-PA0
;R3-R0 connect to PA7-PA4
	ldi r20, 0b00001111 ; set upper 4 bits of PORTD as input with pull-up, lower 4 bits as output
	out DDRA, r20
	ldi r20, 0b11111111 ; enable pull up resistor 
	out PORTA, r20

;SPI initialize
;LSB first, Slave mode, sampling on rising edge
	ldi r16, (1<<SPE0) | (1<<DORD0)
	out SPCR0, r16	

	ldi r16, (1<<6)	| (1<<0)	; Set MISO as output, set PORTB0 as output for interrupt signal 
	out DDRB, r16	


;Initialize interrupt signal
;PORTB0 as output to send interrupt signal to master
	
	sbi PORTB, 0
main:
	
	call keypad_scan
	cbi PORTB, 0
	call SPI_Transmit
	rjmp main


;----------------------------------------------------------------------------
; ATmega324PA keypad scan function
; Scans a 4x4 keypad connected to PORTA
;C3-C0 connect to PA3-PA0
;R3-R0 connect to PA7-PA4
; Returns the key value (0-15) or 0xFF if no key is pressed
keypad_scan:
	ldi r22, 0b11110111 ; initial col mask
	ldi r23, 0 ; initial pressed row value
	ldi r24,3 ;scanning col index

keypad_scan_loop:
	out PORTA, r22 ; scan current col
	nop ;need to have 1us delay to stablize
	sbic PINA, 4 ; check row 0
	rjmp keypad_scan_check_col2
	rjmp keypad_scan_found ; row 0 is pressed

keypad_scan_check_col2:
	sbic PINA, 5 ; check row 1
	rjmp keypad_scan_check_col3
	ldi r23, 1 ; row1 is pressed
	rjmp keypad_scan_found

keypad_scan_check_col3:
	sbic PINA, 6 ; check row 2
	rjmp keypad_scan_check_col4
	ldi r23, 2 ; row 2 is pressed
	rjmp keypad_scan_found

keypad_scan_check_col4:
	sbic PINA, 7 ; check row 3
	rjmp keypad_scan_next_row
	ldi r23, 3 ; row 3 is pressed
	rjmp keypad_scan_found

keypad_scan_next_row:
 ; check if all rows have been scanned
	cpi r24,0
	breq keypad_scan_not_found
 ; shift row mask to scan next row
	ror r22
	dec r24 ;increase row index
	rjmp keypad_scan_loop

keypad_scan_found:
; combine row and column to get key value (0-15)
;key code = row*4 + col
	lsl r23 ; shift row value 4 bits to the left
	lsl r23
	add r23, r24 ; add row value to column value
	;cbi PORTB, 0	; Clear interrupt signal
	;call SPI_Transmit
	ret

keypad_scan_not_found:
	ldi r23, 0x00 ; no key pressed
	;call SPI_transmit
	ret

;---------------------------------------------------------------------
;Subroutine to transmit data in r23
SPI_Transmit:
		push r17
		;cbi PORTB, 0	; Clear interrupt signal 
		out SPDR0, r23	; Output data to register to transmit

		


wait:	
		in r17, SPSR0
		sbrs r17, SPIF0
		rjmp wait

		in r18, SPDR0

		sbi PORTB, 0	; Set interrupt signal 

		pop r17

		ret

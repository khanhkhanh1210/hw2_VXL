;
; homework2 master.asm
;
; Created: 11/16/2023 2:50:17 PM
; Author : Duc Anh
;

	.equ SS = 4
	.def LCDData = r16
	.equ LCDPORT = PORTA ; Set signal port reg to PORTA
	.equ LCDPORTDIR = DDRA ; Set signal port dir reg to PORTA
	.equ LCDPORTPIN = PINA ; Set clear signal port pin reg to PORTA
	.equ LCD_RS = PINA0
	.equ LCD_RW = PINA1
	.equ LCD_EN = PINA2
	.equ LCD_D7 = PINA7
	.equ LCD_D6 = PINA6
	.equ LCD_D5 = PINA5
	.equ LCD_D4 = PINA4
	.org 0x00
	rjmp main

	.org 0x02
	rjmp INT0_Keypressed

	.org 0x40

main:
	
	call LCD_Init	; Initialize the LCD

	;USART_Init:
	 ; Set baud rate to 9600 bps with 8 MHz clock
	 ldi r16, 103
	 sts UBRR0L, r16
	;set double speed
	ldi r16, (1 << U2X0)
	 sts UCSR0A, r16
	 ; Set frame format: 8 data bits, no parity, 1 stop bit
	 ldi r16, (1 << UCSZ01) | (1 << UCSZ00)
	 sts UCSR0C, r16
	 ; Enable transmitter and receiver
	 ldi r16, (1 << RXEN0) | (1 << TXEN0)
	 sts UCSR0B, r16
	 

;Initialize Interrupt:
	ldi r16, (1<<ISC01)			;Interrupt on falling edge
	sts EICRA, r16
	sei
	ldi r16, (1<<INT0)			;Enable INT0
	out EIMSK, r16

	ldi r16, (0<<2)	; Set INT0 pin as input 
	out DDRD, r16

;SPI initialize
	ldi r16, (1<<5) | (1<<7) | (1<<4)	;Set output for pins SS, CLK, MOSI
	out DDRB, r16
;LSB Master mode, sampling on rising edge, fclk / 8
	ldi r16, (1<<SPE0)  | (1<<MSTR0) | (1<<SPR00) | (1<<DORD0)
	out SPCR0, r16
	ldi r16, (1<<SPI2X0)
	out SPSR0, r16

	ldi r18, 0

;Set PORTA as output for LED bar 
	;ldi r16, 0xFF
	;out DDRA, r16


start:	
		;out PORTA, r18
		ldi r16, 0
		ldi r17, 0
		call LCD_Move_Cursor
		
		ldi r16, 0x30
		CPI r18, 10
		BRSH send_LCD_DATA
		add r16, r18
send_LCD_DATA:
		call  LCD_Send_Data
		jmp start

NO_KEY_PRESSED:
		ldi r16, 0x30
		call LCD_Send_Data
		jmp start

;-------------------------------------------------------------

;Interrupt
INT0_Keypressed:
		cbi	PORTB,SS
		call SPI_Transmit	
		sbi	PORTB,SS
		;lsr r18
		;mov r16, r18
		;call out_data_lcd
		reti

;--------------------------------------------------------------
;Send 1 byte in r16 using SPI, recieved data in r18
SPI_Transmit:
		push r17
		out SPDR0, r16
wait:	
		in r17, SPSR0
		sbrs r17, SPIF0
		rjmp wait

		in r18, SPDR0

		pop r17
		ret
;------------------------------------------------------------------
;send out 1 byte to UART in r16
USART_SendChar:
	push r17
	; Wait for the transmitter to be ready
USART_SendChar_Wait:
	lds r17, UCSR0A
	sbrs r17, UDRE0 ;check USART Data Register Empty bit
	rjmp USART_SendChar_Wait
	sts UDR0, r16 ;send out
	pop r17
	ret
;------------------------------------------------------------------
out_data_lcd:
		ldi r16, 0
		ldi r17, 0
		call LCD_Move_Cursor

		ldi r16, 0x30
		add r16, r18
		call USART_SendChar 
		ret

;------------------------------------------------------------------
;subroutine to initialize the LCD
LCD_Init:
		; Set up data direction register for Port A
	ldi r16, 0b11110111 ; set PA7-PA4 as outputs, PA2-PA0 as output
	out LCDPORTDIR, r16
	; Wait for LCD to power up
	call DELAY_10MS
	call DELAY_10MS

	 ; Send initialization sequence
	ldi r16, 0x02 ; Function Set: 4-bit interface
	call LCD_Send_Command
	ldi r16, 0x28 ; Function Set: enable 5x7 mode for chars
	call LCD_Send_Command
	ldi r16, 0x0E ; Display Control: Display OFF, Cursor ON
	call LCD_Send_Command
	ldi r16, 0x01 ; Clear Display
	call LCD_Send_Command
	ldi r16, 0x80 ; Clear Display
	call LCD_Send_Command
	ret

;subroutine to wait
LCD_wait_busy:
	push r16
	ldi r16, 0b00000111 ; set PA7-PA4 as input, PA2-PA0 as output
	out LCDPORTDIR, r16
	ldi r16,0b11110010 ; set RS=0, RW=1 for read the busy flag
	out LCDPORT, r16
	nop
LCD_wait_busy_loop:
	sbi LCDPORT, LCD_EN
	nop
	nop
	in r16, LCDPORTPIN
	cbi LCDPORT, LCD_EN
	nop
	sbi LCDPORT, LCD_EN
	nop
	nop
	cbi LCDPORT, LCD_EN
	nop
	andi r16,0x80
	cpi r16,0x80
	breq LCD_wait_busy_loop
	ldi r16, 0b11110111 ; set PA7-PA4 as output, PA2-PA0 as output
	out LCDPORTDIR, r16
	ldi r16,0b00000000 ; set RS=0, RW=1 for read the busy flag
	out LCDPORT, r16
	pop r16
	ret

;subroutine to send data
LCD_Send_Data:
	push r17
	call LCD_wait_busy ;check if LCD is busy
	mov r17,r16 ;save the command
	; Set RS high to select data register
	; Set RW low to write to LCD
	andi r17,0xF0
	ori r17,0x01
	; Send data to LCD
	out LCDPORT, r17
	nop
	; Pulse enable pin
	sbi LCDPORT, LCD_EN
	nop
	cbi LCDPORT, LCD_EN
	; Delay for command execution
	;send the lower nibble
	nop
	swap r16
	andi r16,0xF0
	; Set RS high to select data register
	; Set RW low to write to LCD
	andi r16,0xF0
	ori r16,0x01
	; Send command to LCD
	out LCDPORT, r16
	nop
	; Pulse enable pin
	sbi LCDPORT, LCD_EN
	nop
	cbi LCDPORT, LCD_EN
	pop r17
	ret

;subroutine to send command to LCD
LCD_Send_Command:
	push r17
	call LCD_wait_busy ; check if LCD is busy
	mov r17,r16 ;save the command
	; Set RS low to select command register
	; Set RW low to write to LCD
	andi r17,0xF0
	; Send command to LCD
	out LCDPORT, r17
	nop
	nop
	; Pulse enable pin
	sbi LCDPORT, LCD_EN
	nop
	nop
	cbi LCDPORT, LCD_EN
	swap r16
	andi r16,0xF0
	; Send command to LCD
	out LCDPORT, r16
	; Pulse enable pin
	sbi LCDPORT, LCD_EN
	nop
	nop
	cbi LCDPORT, LCD_EN
	pop r17
	ret

	;subroutine to move cursor in LCD
LCD_Move_Cursor:
	cpi r16,0 ;check if first row
	brne LCD_Move_Cursor_Second
	andi r17, 0x0F
	ori r17,0x80
	mov r16,r17
	; Send command to LCD
	call LCD_Send_Command
	ret
LCD_Move_Cursor_Second:
	cpi r16,1 ;check if second row
	brne LCD_Move_Cursor_Exit ;else exit
	andi r17, 0x0F
	ori r17,0xC0
	mov r16,r17
	; Send command to LCD
	call LCD_Send_Command
LCD_Move_Cursor_Exit:
; Return from function
	ret


;-----------------------------------------------------------------
;subroutine to delay 10ms
DELAY_10MS:
	LDI R21,80 ;1MC
	L1: LDI R20,250 ;1MC
	L2: DEC R20 ;1MC
	NOP ;1MC
	BRNE L2 ;2/1MC
	DEC R21 ;1MC
	BRNE L1 ;2/1MC
	RET ;4MC
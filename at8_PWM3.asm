.include "m8def.inc" ; ���������� ������������
.include "Massiv_bait.asm"
;.��� � �������� ������ �� Atmega8 ��� ���� ����
; � ����� �� ����� � 2-� �������� ������ (�.�. �� ���� ���� --- 2 ������)
; ����� ���� �����-�� � ��� ����� �� �������� TmrCntr1 (�.�. � ��� 2-�� �������� �� 1-��)

; ************** Interrupt Vectore Table ************************
.cseg		;directive defines the start of a Code Segment
.org	0x00				; Reset-Address (���������� ��������� �������� ��������� � ������ ������� � ������ 0)
	rjmp	Reset
.org	INT0addr			; External Interrupt Request 0
	reti
.org	INT1addr			; External Interrupt Request 1
	reti
.org	OC2addr 			; Timer/Counter2 Compare Match
;	reti
	rjmp	TIM2_CompMatch
.org	OVF2addr			; Timer/Counter2 Overflow
	reti
;	rjmp TIM2_Overflow
.org	ICP1addr			; Timer/Counter1 Capture Event 
	reti
.org	OC1Aaddr			; Timer/Counter1 Compare Match A
;	reti
	rjmp	TIM1A_CompMatch
;	rjmp	Proverka_D6		; �������� ������ �� ������ D6
.org	OC1Baddr			; Timer/Counter1 Compare Match B
;	reti
	rjmp	TIM1B_CompMatch
.org	OVF1addr			; Timer/Counter1 Overflow
	rjmp	TIM1_Overflow
.org	OVF0addr			; Timer/Counter0 Overflow
	reti
.org	SPIaddr 			; SPI Serial Transfer Complete
	reti
.org	URXCaddr			; UART, Rx Complete
	reti
.org	UDREaddr			; UART Data Register Empty
	reti
.org	UTXCaddr			; UART, Tx Complete
	reti
.org	ADCCaddr			; ADC Conversion Complete
	reti	
.org	ERDYaddr			; EEPROM Ready
	reti
.org	ACIaddr 			; Analog Comparator
	reti


init_PWM1:		
	;���������� ������ PWM � 16�� ������� �������/�������� "1" � � 8-�� ������� �������/�������� "2"
	ldi xlam,0b11110001	; ���������� TCCR1A (COM1A1 COM1A0 COM1B1 COM1B0 FOC1A FOC1B WGM11 WGM10) (xxxxxx01 =8-�� ������ ���, 
	out TCCR1A,xlam		; xxxxxx10 =9-�� ������, xxxxxx11 =10-�� ������) ; COM1A1 COM1A0 COM1B1 COM1B0=11 -- Set OC1A/OC1B on Compare Match when up-counting. Clear OC1A/OC1B on Compare Match when downcounting.
	ldi xlam,0b00000010	; ���������� TCCR1B	(00000101 = f/1024, 00000100 = f/256, 00000011 = f/64)
	out TCCR1B,xlam	; 	; 					(00000010 = f/8, 00000001 = f )
	
	ldi xlam,0b01110010	; ���������� TCCR2 (FOC2 WGM20 COM21 COM20 WGM21 CS22 CS21 CS20)	
	out TCCR2,xlam		;(Bit 6,3 � WGM21:0: Waveform Generation Mode, Bit 5:4 � COM21:0: Compare Match Output Mode ������� ������ ��� ����� ��� ���� 
				;(00000101 = f/1024, 00000100 = f/256, 00000011 = f/64, 00000010 = f/8, 00000001 = f , � ����� �������2 ���� ��� f/32 , f/128)
				; 11 = Set OC2 on Compare Match when up-counting. Clear OC2 on Compare Match when downcounting.

	ldi xlam,0b11011100	; ���������� ��������� ������ ���������� �� ����������� � �� ������������� ��������� 1 � 2
	out TIFR,xlam	; ��������� ������ ���������� �� ����������� ��������� 1 � 2
;	ldi xlam,0b11011100	;���������� ��������� ����� ���������� ��������� ���������� �� ����������� � �������������
	out TIMSK,r16	;��������� ����� ���������� ��������� ���������� �� ����������� � ������������� (���������� ����������� ��������� 1 � 2)

	ldi xlam,0b00001110		; ���������� ��������� ������ ����� �� ����� 1,2,3  ����� B (DDRB --- Data Direction Port B )
	
	out DDRB,xlam	; Set Port B as output ��������� ������ ����� �� ����� 1,2,3 ����� B
					; ���� 2 � 3 ������������ ��� � ��� �������� (SS, MOSI SPI ����������)
	out DDRC,xlam	; Set Port C as output ��������� ������ ����� �� ����� 1,2,3 ����� C
					; � ����� � �������� ������� ������ � �����������
;	cbi DDRD,PD6	; PD6 = 6 (Set PD6/ICP as input) 

;  ldi xlam,0xFF	;��������� ����� FF � ������� xlam
;  out DDRB,xlam	;�������� ������� B � ��� ������ , �� ����� ���������� ����� �� �������
;  in xlam,TCCR1B	;��������� ������� TCCR1B � xlam
;  com xlam		; ��� ������ ������ ���������� ������, � �� �����
;  out PORTB,xlam	; ����� � ������� B ����� ���������� ������ �� �������

	ldi MassivH,HIGH(SinTabl*2)	;�������� � ������� MassivH �������� ����� ���������� ������ ��� ��������� OCR1AH
;	ldi MassivH3,HIGH(SinTabl3*2)	;�������� � ������� MassivH3 �������� ����� ���������� ������ ��� ��������� OCR1BH
	
	eor cntA,cntA	;��������� �������� ��� �������� ��������� ��� A
	mov ZH,MassivH			;�������� � ������� ZH �������� ����� ���������� ������ ��� ��������� OCR1AH
	mov ZL,cntA			;�������� � ������� ZL �������� ����� ���������� ������ ��� ��������� OCR1AL

	out OCR1AH,zero 	; Write 0 to OCR1AH -- ������ ������� 0,�.�. ��������� 8-�� ������ ���
	lpm 				; lpm �������� ���: ������ � ������� r0 ������, ������. �� ������ � �������� Z 
	out OCR1AL,r0		; Write ����� to OCR1AL. �.�., ��������� ����� ��� ��������� A

	ldi cntB,13	;����� ���� ��� �������� ��������� ��� B
;	mov ZH,MassivH		; (���, ��������, ��� ����, �.�. � ������) �������� � ������� ZH �������� ����� ���������� ������ ��� ��������� OCR1BH
	mov ZL,cntB			;�������� � ������� ZL �������� ����� ���������� ������ ��� ��������� OCR1BL
	
	out OCR1BH,zero 	; Write 0 to OCR1BH -- ������ ������� 0,�.�. ��������� 8-�� ������ ���
	lpm 				; lpm �������� ���: ������ � ������� r0 ������, ������. �� ������ � �������� Z 
	out OCR1BL,r0 	; Write ����� to OCR1BL. �.�., ��������� ����� ��� ��������� B

	ldi cntC,26	;����� ���� ��� �������� ��������� ��� B
	mov ZL,cntC			;�������� � ������� ZL �������� ����� ���������� ������ ��� ��������� OCR2
	
	lpm 				; lpm �������� ���: ������ � ������� r0 ������, ������. �� ������ � �������� Z 
	out OCR2,r0 	; Write ����� to OCR2. �.�., ��������� ����� ��� ��������� C

	inc cntA	; ���������� �������� A
	inc cntB	; ���������� �������� B
	inc cntC	; ���������� �������� C
	ret	;Returns from subroutine. The return address is loaded from the STACK. The stack pointer uses a pre-increment scheme during RET.



; *** Begin of Program Execution ****************************************************
;************************************************************************************
Reset:
; Initialization of the Stack Pointer
	ldi	r16,low(RAMEND)		;
	out	SPL,r16				;Set Stack Pointer to top of RAM
	ldi	r16,high(RAMEND)	;
	out 	SPH,r16			;

	eor zero,zero 	; ���� � zero
	rcall	init_PWM1

	sei				; Global Interrupts enabled

; Endlessloop
loopForever:
	nop
	nop
;	sbic PIND, PIND6	
;	rjmp Smena_Freq		; ���� ������ D6 ������ -- ������� � Smena_Freq

	rjmp loopForever	; ���� ������ D6 �� ������ -- ������� � loopForever
			
;	ldi xlam,0b11110000 ; ���������� ��������� ������ ����� �� ����� 4,5,6,7 ����� D
;	out DDRD,xlam	; ��������� ������ ����� �� ����� 4,5,6,7 ����� D


Proverka_D6: ; �������� �� ���������� OCR1A
;	push r16
	in r16,SREG
	push r16

	sbic PIND, PIND6	
	rjmp Smena_Freq		; ���� ������ D6 ������ -- ������� � Smena_Freq

	pop r16
	out SREG,r16
;	pop r16
	reti ; ������� �� ����������

Smena_Freq:		; ������������ �������� �������� ������� ����� ��� ������ D6

	in xlam,PIND	;����� ������� PIND, �������� ����� ���� ������ D6
	andi xlam,0b01000000	; ��������� � ��� ��������� ���������� ���� ������ D6
	cp xlam,zero	;��������� � ����
	brne Smena_Freq	; ������� � �������� ����� ������ 6, ���� � �� ���������	

	in xlam,TCCR1B	;����� ������� TCCR1B, �������� ����� ���� ������� � ����
	andi xlam,0b00000111	; ��������� � ��� ��������� ���������� ���� �������
	ldi xlam2,5		;����� � ����2 ������ =00000101
	cp xlam,xlam2	;��������� � 00000101
	brne IncXlam	;��������, ���� �� �����
	ldi xlam,0 ; ����� � ���� 00000000, ���� ������� TCCR1B ����� 5

IncXlam:
	inc xlam	; ���������� ���� �� 1	(00000101 = f/1024, 00000100 = f/256, 00000011 = f/64)
	out TCCR1B,xlam	; 	; 					(00000010 = f/8, 00000001 = f )

	ldi xlam,0 ; ���������� ������ ������ D6 
	out PIND,xlam	; ����� D6 
; ����������� ���� TCCR1B ������������:
in xlam,TCCR1B	;��������� ������� TCCR1B � ����
com xlam		; ��� ������ ������ ���������� ������, � �� �����
out PORTB,xlam	; ������� B ����� ���������� ����� �� �������

	rjmp loopForever	; ������� � loopForever



TIM1_Overflow:
	;����� ���� ������� ��������, ��� cntA cntB �� ������ 39
;	out TCNT2,zero	; ��� �.� ������: ��� ������ �������� ���� TimerCounter2 � 1-�� ��������
	in r16,SREG		;Load an I/O Location to Register
	push r16
	
	mov ZH,MassivH		;�������� � ������� Z �������� ����� ���������� ������ ��� ��������� OCR1AH
	mov ZL,cntA			;�������� � ������� Z �������� ����� ���������� ������ ��� ��������� OCR1AL

	out OCR1AH,zero 	; Write 0 to OCR1AH
	lpm 				; lpm �������� ���: �� ������ Z ������ � r0
	out OCR1AL,r0		; Write ����� to OCR1AL

;	mov ZH,MassivH3 	;�������� � ������� Z �������� ����� ���������� ������ ��� ��������� OCR1BH
	mov ZL,cntB			;�������� � ������� Z �������� ����� ���������� ������ ��� ��������� OCR1BL
	
	out OCR1BH,zero 	; Write 0 to OCR1BH
	lpm 				; lpm �������� ���: �� ������ Z ������ � r0
	out OCR1BL,r0 	; Write ����� to OCR1BL

	mov ZL,cntC			;�������� � ������� ZL �������� ����� ���������� ������ ��� ��������� OCR2
	lpm 				; lpm �������� ���: ������ � ������� r0 ������, ������. �� ������ � �������� Z 
	out OCR2,r0 	; Write ����� to OCR2. �.�., ��������� ����� ��� ��������� C


	ldi xlam,Limit		; ������� � r16 ������ ����� ��� cntA cntB
	cp cntA,xlam		;��������� ������� � � ��������
	brne A_inc			;���� ����� (Z=0), �� ����� � cntA "����� 1"
	ldi cntA,-1
	A_inc:
	inc cntA			; ���� �� ����� , �� ���������� �������� A

;	ldi xlam,Limit		; ������� � r16 ������ ����� ��� cntA cntB
	cp cntB,xlam		;��������� ������� B � ��������
	brne B_inc			;���� ����� (Z=0), �� ����� � cntB "����� 1"
	ldi cntB,-1
	B_inc:
	inc cntB	; ���� �� ����� , �� ���������� �������� B

	cp cntC,xlam		;��������� ������� C � ��������
	brne C_inc			;���� ����� (Z=0), �� ����� � cntC "����� 1"
	ldi cntC,-1
	C_inc:
	inc cntC	; ���� �� ����� , �� ���������� �������� C


	pop r16
	out SREG,r16
		
	reti	;Returns from interrupt. The return address is loaded from the STACK and the global interrupt flag is set.


TIM1A_CompMatch:		; ����� ���������� ��������� ��������� ���� PORTB0 �� ������� ���������� TIM1A_CompMatch

	in r16,SREG		;Load an I/O Location to Register
	push r16

	sbic PINC, PINC1	; ���������� ��������� ����� ���� ��� ������� Skip if Bit in I/O Register Cleared
;	rjmp SetOutB0
	CBI PORTC,1
	sbis PINC, PINC1	; ���������� ��������� ����� ���� ��� ������� Skip if Bit in I/O Register Set
	SBI PORTC,1

	pop r16
	out SREG,r16
		

	reti	;Returns from interrupt. The return address is loaded from the STACK and the global interrupt flag is set.

TIM1B_CompMatch:		; ����� ���������� ��������� ��������� ���� PORTB0 �� ������� ���������� TIM1A_CompMatch

	in r16,SREG		;Load an I/O Location to Register
	push r16

	sbic PINC, PINC2	; ���������� ��������� ����� ���� ��� ������� Skip if Bit in I/O Register Cleared
;	rjmp SetOutB0
	CBI PORTC,2
	sbis PINC, PINC2	; ���������� ��������� ����� ���� ��� ������� Skip if Bit in I/O Register Set
	SBI PORTC,2

	pop r16
	out SREG,r16
		

	reti	;Returns from interrupt. The return address is loaded from the STACK and the global interrupt flag is set.



TIM2_CompMatch:		; ����� ���������� ��������� ��������� ���� PORTB0 �� ������� ���������� TIM2_CompMatch

	in r16,SREG		;Load an I/O Location to Register
	push r16

	sbic PINC, PINB3	; ���������� ��������� ����� ���� ��� ������� Skip if Bit in I/O Register Cleared
;	rjmp SetOutB0
	CBI PORTC,3
	sbis PINC, PINB3	; ���������� ��������� ����� ���� ��� ������� Skip if Bit in I/O Register Set
	SBI PORTC,3

	pop r16
	out SREG,r16
		

	reti	;Re

;***********************************************************



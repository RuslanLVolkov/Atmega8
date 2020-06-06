;150519  at8_PWM3_03_14  �������� ��� ����������� ����������. 
;(��� ���� ���� ������� v13 252 ����� �� ������ � ������� ��������� � ������� ��������� ����� ���������)
;������ �������� ����������� �������� ��� ��������� �� 1/4 ������ (�� 0 �� ����)
;��� ��� � 1 � 3 ��������� � ������ ������� ����� ��1 �� 63-�, � �� 2-� � 4-� --- �� 62-� �� �������
;����� 252 ����� �� ������, �������� ��� ����� 84 �����
;���������������� �������� �� ���� ����������� ���� (����, ����� ����� �� ��������� 251=Limit, ������ ���� = 248 (bin 11111000)
;���������������� ��������� ��� ������������� ����������� ���������� �� ������� ����
;�������� � ��������� ������������ : ���������� �������� �� ����� A B C , ��������� � ���� �������������� 
.include "m8def_03_14.inc" ; ���������� ������������  
.include "Massiv_bait_Napr_63x4.asm"		; Limit	= 251
; .include "Massiv_bait_Tok_252.asm"		; Limit	= 251

; ************** Interrupt Vectore Table ************************
.cseg		;directive defines the start of a Code Segment
.org	0x00				; Reset-Address (���������� ��������� �������� ��������� � ������ ������� � ������ 0)
	rjmp	Reset
.org	INT0addr			; External Interrupt Request 0
	reti
.org	INT1addr			; External Interrupt Request 1
	reti
.org	OC2addr 			; Timer/Counter2 Compare Match
	reti
;	rjmp	TIM2_CompMatch
.org	OVF2addr			; Timer/Counter2 Overflow
;	reti
	rjmp TIM2_Overflow
.org	ICP1addr			; Timer/Counter1 Capture Event 
	reti
.org	OC1Aaddr			; Timer/Counter1 Compare Match A
	reti
;	rjmp	TIM1A_CompMatch
.org	OC1Baddr			; Timer/Counter1 Compare Match B
	reti
;	rjmp	TIM1B_CompMatch
.org	OVF1addr			; Timer/Counter1 Overflow
;	reti
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
;	reti	
	rjmp	Refresh_ADC
.org	ERDYaddr			; EEPROM Ready
	reti
.org	ACIaddr 			; Analog Comparator
	reti


init_PWM1:		
	;���������� ������ PWM � 16�� ������� �������/�������� "1" � � 8-�� ������� �������/�������� "2"
	ldi r16,0b11110001	; ���������� TCCR1A (COM1A1 COM1A0 COM1B1 COM1B0 FOC1A FOC1B WGM11 WGM10) (xxxxxx01 =8-�� ������ ���, 
	out TCCR1A,r16		; xxxxxx10 =9-�� ������, xxxxxx11 =10-�� ������) ; COM1A1 COM1A0 COM1B1 COM1B0=11 -- Set OC1A/OC1B on Compare Match when up-counting. Clear OC1A/OC1B on Compare Match when downcounting.
	ldi r16,0b00000001	; ���������� TCCR1B	(00000101 = f/1024, 00000100 = f/256, 00000011 = f/64)
	out TCCR1B,r16	; 	; 					(00000010 = f/8, 00000001 = f )

	ldi r16,0b01110001	; ���������� TCCR2 (FOC2 WGM20 COM21 COM20 WGM21 CS22 CS21 CS20)	
	out TCCR2,r16		;(Bit 6,3 � WGM21:0: Waveform Generation Mode, Bit 5:4 � COM21:0: Compare Match Output Mode ������� ������ ��� ����� ��� ���� 
				;(00000101 = f/1024, 00000100 = f/256, 00000011 = f/64, 00000010 = f/8, 00000001 = f , � ����� �������2 ���� ��� f/32 , f/128)
				; 11 = Set OC2 on Compare Match when up-counting. Clear OC2 on Compare Match when downcounting.
;&&&&&&&&&&; ������� 150504
;	ldi r16,0b00000100 
;	out TCNT2,r16	; ��������� � ������� �������� Timer2 �������� ������ ����� � �������� Timer1 �� ��������� �����. 
					; ������ ����� ��������� ���� ����� ��� 120 ��������
; 150507 ���������������: ������, ��� ��� ����� ������� ���������. �.�., ���������� ���������� ��� �� ��������
;&&&&&&&&&&
	ldi r16,0b11011100	; ���������� ��������� ������ ���������� �� ����������� � �� ������������� ��������� 1 � 2
	out TIFR,r16	; ��������� ������ ���������� �� ����������� ��������� 1 � 2
;	ldi r16,0b11011100	;���������� ��������� ����� ���������� ��������� ���������� �� ����������� � �������������
	out TIMSK,r16	;��������� ����� ���������� ��������� ���������� �� ����������� � ������������� (���������� ����������� ��������� 1 � 2)

	ldi r16,0b00001111		; ���������� ��������� ������ ����� �� ����� 0,1,2,3  ����� B (DDRB --- Data Direction Port B )
	
	out DDRB,r16	; Set Port B as output ��������� ������ ����� �� ����� 0,1,2,3 ����� B
					; ���� 2 � 3 ������������ ��� � ��� �������� (SS, MOSI SPI ����������)
;	ldi r16,0b00011100		; ���������� ��������� ������ ����� �� ����� 2,3,4,5  ����� C (DDRC --- Data Direction Port C )
;	out DDRC,r16	; Set Port C as output ��������� ������ ����� �� ����� 1,2,3 ����� C
					; � ����� � �������� ������� ������ � �����������

	ldi MassivH,HIGH(SinTabl_1*2)	;��� ����� ���������� (����� �� �������) 50% ��������� �������� � ������� MassivH �������� ����� ���������� ������ ��� ��������� OCR1AH
;	ldi MassivH,HIGH(SinTabl_125*2)	;��� ����� ���� (����� �� �������) 125% ��������� �������� � ������� MassivH �������� ����� ���������� ������ ��� ��������� OCR1AH

	ldi cntA,SdvigA		;����� � ���-�������� �� ������ ������� ���� A (������ = 0)
	mov ZH,MassivH			;�������� � ������� ZH �������� ����� ���������� ������ ��� ��������� OCR1AH
	mov ZL,cntA			;�������� � ������� ZL �������� ����� ���������� ������ ��� ��������� OCR1AL


	out OCR1AH,zero 	; Write 0 to OCR1AH -- ������ ������� 0,�.�. ��������� 8-�� ������ ���
	lpm 				; lpm �������� ���: ������ � ������� r0 ������, ������. �� ������ � �������� Z 
	out OCR1AL,r0		; Write ����� to OCR1AL. �.�., ��������� ����� ��� ��������� A

	ldi cntB,SdvigB	;����� � ���-�������� �� ������ ������� ���� (B -A) 
;	mov ZH,MassivH		; (���, ��������, ��� ����, �.�. � ������) �������� � ������� ZH �������� ����� ���������� ������ ��� ��������� OCR1BH
	mov ZL,cntB			;�������� � ������� ZL �������� ����� ���������� ������ ��� ��������� OCR1BL
	
	out OCR1BH,zero 	; Write 0 to OCR1BH -- ������ ������� 0,�.�. ��������� 8-�� ������ ���
	lpm 				; lpm �������� ���: ������ � ������� r0 ������, ������. �� ������ � �������� Z 
	out OCR1BL,r0 	; Write ����� to OCR1BL. �.�., ��������� ����� ��� ��������� B

	ldi cntC,SdvigC	;����� � ���-�������� �� ������ ������� ���� (C -A) 
	mov ZL,cntC			;�������� � ������� ZL �������� ����� ���������� ������ ��� ��������� OCR2
	
	lpm 				; lpm �������� ���: ������ � ������� r0 ������, ������. �� ������ � �������� Z 
	out OCR2,r0 	; Write ����� to OCR2. �.�., ��������� ����� ��� ��������� C

	inc cntA	; ���������� �������� A
	inc cntB	; ���������� �������� B
	inc cntC	; ���������� �������� C
	
;	clr SchetA	; ��� ��������������� �������� ����� ������� ������������� ������� A-A',B-B',C-C'
;	clr SchetB	; ����������� ����� ���� ����� ���
;	clr SchetC
	clr Amplituda	; ���� ������� � ��������� ����� ���� ������ ������� ������� ������
	clr DeltaFi	; ���� ������� � ��������� ����� ����� �������� ��� ����� ����
	clr vhodADC	; ������� 2 ���� �������� vhodADC �. �����. ����� ���� ���������� ��� (������ �������� 3 ����� ADC0 ADC1 ADC2)
	
	ret	;Returns from subroutine. The return address is loaded from the STACK. The stack pointer uses a pre-increment scheme during RET.


init_ADC:

	ldi r16,0b01100000		; ����� ADMUX (REFS1 REFS0 ADLAR � MUX3 MUX2 MUX1 MUX0) �.� ������� ��������� ���� ADC0
	out ADMUX,r16
	ldi r16,0b11111111		; ����� ADCSRA (ADEN ADSC ADFR ADIF ADIE ADPS2 ADPS1 ADPS0) 3 ����� ���� ����� ������� �������
	out ADCSRA,r16

;	ldi r16,0b11111111		; ����� ADCSRA (ADEN ADSC ADFR ADIF ADIE ADPS2 ADPS1 ADPS0)
;	out DDRD,r16		; ��� �������� ��� ��������� ����� ��������� ��� ��� ��������

	ret	;Returns from subroutine. The return address is loaded from the STACK. The stack pointer uses a pre-increment scheme during RET.


init_OSCCAL:
	ldi r16,OSCCAL_synchr		; ��������� � OSCCAL ����������� �������� ��� ������� 50�� ��� 8��� � 252 ����
	out OSCCAL, r16		;��������� ���������: OSCCAL	= B9, BA, B4 ��� B6 ��� AT8-����������)
	ret	;Returns from subroutine. ; ( OSCCAL	= BD, BC, B5 ��� B7 ��� AT8-����)
									;(��������, ��� ������ 1, 2., 4,  8  ���
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
	rcall	init_ADC
	rcall	init_OSCCAL

	sei				; Global Interrupts enabled

; Endlessloop
loopForever:
	nop
	nop
;&&&&&&&&&&&&&
;&&&&&&&&&&&&&
	rjmp loopForever	
			

TIM1_Overflow:
	; ������������ ������� ��� ��������� �� ����� A � B
	; ��������, ��� cntA cntB �� ������ Limit (� ������ ������ 78 �����: �� 0 �� 77)
	; ���� �������������� ���� ��� ���� �� ���� PC5. 
;	out TCNT2,zero	; ��� ������ �������� ���� TimerCounter2 � 1-�� �������� --- ��� ������
	in r16,SREG		;Load an I/O Location to Register
	push r16

	mov ZH,MassivH		;�������� � ������� Z �������� ����� ���������� ������ ��� ��������� OCR1AH
	rcall Pereschet_A	; �������� ��������� ������� � ��������� ����� � ������� ��������� OCR1A
	rcall Pereschet_B	; �������� ��������� ������� � ��������� ����� � ������� ��������� OCR1B

	rcall Priem_Synchr_imp	; ���� �������������� ���� ��� ���� �� ���� PC5 (����� �28).

	pop r16
	out SREG,r16
		
	reti	;Returns from interrupt. The return address is loaded from the STACK and the global interrupt flag is set.


TIM2_Overflow:
	in r16,SREG		;Load an I/O Location to Register
	push r16
	
	mov ZH,MassivH		;�������� � ������� Z �������� ����� ���������� ������ ��� ��������� 
	rcall Pereschet_C	; �������� ��������� ������� � ��������� ����� � ������� ��������� OCR2

	rcall Synch_Pulse	; 

	pop r16
	out SREG,r16

	reti	;Returns from interrupt. The return address is loaded from the STACK and the global interrupt flag is set.


Refresh_ADC:
	in r16,SREG		;Load an I/O Location to Register
	push r16

	sbrc vhodADC,0	; ������� 2 ���� �������� vhodADC �. �����. ����� ���� ���������� ��� (������ �������� 3 ����� ADC0 ADC1 ADC2). 
					;�.�. ���� ������������� ������� ADCH: 00 --- ��������� (ADC0), 01 --- ���� (ADC1), 10 --- ������� (ADC2). 
;******** test 150504
;	in r16,ADMUX
;	sbrs r16,0	; 0-� ��� ����� ���� ������������� ������� ADCH, ���� ��� ����. �.� � ������ ���� ������ ���
;********	
	RJMP VhodDeltaFi

	sbrc vhodADC,1	; xxxxxx10 ������������� ����� ADC2 --- Freq
	RJMP VhodFreq

;******
	;&&&&&&&&&
	;Vhod-Amplituda (��������� ���� ����� ����� ADC0)

	in r16,ADCH
	andi r16,0b11110000  ; ������� ����� 4 ����
	swap r16		; ����� 4-���� ��� � ����� (������ � � ��� �����, �� ����� ���� ������� RefreshAdres:)

	sub Amplituda,r16	; ������� �� �������� �������� 
	breq Exit1			; �����, ���� �������� �� ����������
	rcall RefreshAdres

	Exit1:
	mov Amplituda,r16
	ldi r16,0b01111111		; ����� disable ADC ����� ADCSRA (ADEN ADSC ADFR ADIF ADIE ADPS2 ADPS1 ADPS0) 3 ����� ���� ����� ������� �������
	out ADCSRA,r16			; test

	ldi r16,0b01100001		; ����� ADMUX: � ���� ��� � ��������� ���� ADC1
	out ADMUX,r16
	ldi r16,0b11111111		; ����� �������� ���
	out ADCSRA,r16			; 

	inc vhodADC
	RJMP Exit2


	;&&&&&&&&&
	VhodDeltaFi:	;��������� ���� ����� ����� ADC1

	in DeltaFi,ADCH
;	mov DeltaFi,r16
	ldi r16,0b01111111		; ����� disable ADC ����� ADCSRA (ADEN ADSC ADFR ADIF ADIE ADPS2 ADPS1 ADPS0) 3 ����� ���� ����� ������� �������
	out ADCSRA,r16			; test

	ldi r16,0b01100010		; ����� ADMUX: � ���� ��� � ��������� ���� ADC2
	out ADMUX,r16
	ldi r16,0b11111111		; ����� �������� ���
	out ADCSRA,r16			; 

	inc vhodADC

	RJMP Exit2

	;&&&&&&&&&
	VhodFreq:	
	
	sbic PIND, PIND0	; ���������� ��������� ����� ���� �� ���� 0 (�������� ����� ����������� ����� �� ����)
	RJMP OSCCAL_pri_synchr		; ���������� ������ ����������� ������� ��� ��������� �� ����
	; !!!
	in r16,ADCH		;��������� ���� ����� ����� ADC2 ����� � ������ �������
;	andi r16,0b1111110  ; ������� 7 ����� ���  ��� ������ �������� --- �������� ������
	lsr r16			;����� �� 1 ��� ������
	lsr r16			;� ��� 1 �����. ������ �������� ����������� 0--63 ��.
	ldi r17,FreqMin
	add r16,r17	; ��������� � 150-��, ������� �������� 150--213��.
	cp Freq,r16   ;--- ���� ��� Freq ����� ������ ��� ����� ���������. ��������, ��� � �� ����� ������
	breq NextVhodADC			;���� ����� (���� Z=1), �� ������� � �����	
	out OSCCAL,r16	;������ ����� �������� �� ��� (������� ���� ) � ������������� ������� ����������� RC-����������  
	mov Freq,r16	

	NextVhodADC:		; ������������ � ����� ���
	ldi r16,0b01111111		; ����� disable ADC ����� ADCSRA (ADEN ADSC ADFR ADIF ADIE ADPS2 ADPS1 ADPS0) 3 ����� ���� ����� ������� �������
	out ADCSRA,r16			; test

	ldi r16,0b01100000		; ����� ADMUX: � ���� ��� � ��������� ���� ADC0
	out ADMUX,r16
	ldi r16,0b11111111		; ����� �������� ���
	out ADCSRA,r16			; 

	ldi vhodADC,0b00000000	; ����� ������ ��������� ldi �.�. ��� vhodADC ������ ������� R18 (�������� ����� ����� ������� ��������� ������� � 16-�� ��������, ���� ��������� �����)

	RJMP Exit2

	OSCCAL_pri_synchr:
	in r16,OSCCAL
	ldi r17,OSCCAL_synchr		; ��������� �� ������ ��� ����������� ������ -- hex B9/BA/B4/B6 ��� �������� 1/2/4/8 ���)
						; ��������� �� ������ ��� ����������� ���� -- hex BD/BC/B5/B7 ��� �������� 1/2/4/8 ���)
	cp r16,r17
	breq NextVhodADC		; ���� �����, �� �� �����
	out OSCCAL,r17		; �������� ��������� �����
	RJMP NextVhodADC

	Exit2:
	pop r16
	out SREG,r16
	reti	;������� �� ����������


;***********************************************************
;***********************************************************

RefreshAdres:		; ������������ ��������� ���������� ������ ������� Massiv_bait
	  sbrc r16,3
	  rjmp m_1xxx	; ����������� �� �.�.
	  sbrc r16,2
	  rjmp m_01xx
	  sbrc r16,1
	  rjmp m_001x
	  sbrc r16,0
	  rjmp m_0001

	  ldi MassivH,HIGH(SinTabl_1*2)
;	  ldi r16, 0b00000100			; ������ ��� ����� ���� ������� � ������ ����������
;	  out OSCCAL, r16				; ��� ��� ���� ������ ��������� ����� ��������� -->> OSCCAL (����� �������� ������)
	  ret

	m_0001:
	  ldi MassivH,HIGH(SinTabl_2*2)
;	  ldi r16, 0b00001010
;	  out OSCCAL, r16
	  ret

	m_001x:
	  sbrc r16,0
	  rjmp m_0011

	  ldi MassivH,HIGH(SinTabl_3*2)
;	  ldi r16, 0b00010100
;	  out OSCCAL, r16
	  ret

	m_0011:
	  ldi MassivH,HIGH(SinTabl_4*2)
;	  ldi r16, 0b00011110
;	  out OSCCAL, r16
	  ret

	m_01xx:
	  sbrc r16,1
	  rjmp m_011x
	  sbrc r16,0
	  rjmp m_0101

	  ldi MassivH,HIGH(SinTabl_5*2)
;	  ldi r16, 0b00101000
;	  out OSCCAL, r16
	  ret

	m_0101:
	  ldi MassivH,HIGH(SinTabl_6*2)
;	  ldi r16, 0b00111100
;	  out OSCCAL, r16
	  ret

	m_011x:
	  sbrc r16,0
	  rjmp m_0111

	  ldi MassivH,HIGH(SinTabl_7*2)
;	  ldi r16, 0b01010000
;	  out OSCCAL, r16
	  ret

	m_0111:
	  ldi MassivH,HIGH(SinTabl_8*2)
;	  ldi r16, 0b01100100
;	  out OSCCAL, r16
	  ret

	m_1xxx:
	  sbrc r16,2
	  rjmp m_11xx
	  sbrc r16,1
	  rjmp m_101x
	  sbrc r16,0
	  rjmp m_1001

	  ldi MassivH,HIGH(SinTabl_9*2)
;	  ldi r16, 0b01111000
;	  out OSCCAL, r16
	  ret

	m_1001:
	  ldi MassivH,HIGH(SinTabl_10*2)
;	  ldi r16, 0b10001100
;	  out OSCCAL, r16
	  ret

	m_101x:
	  sbrc r16,0
	  rjmp m_1011

	  ldi MassivH,HIGH(SinTabl_11*2)
;	  ldi r16, 0b10100000
;	  out OSCCAL, r16
	  ret

	m_1011:
	  ldi MassivH,HIGH(SinTabl_12*2)
;	  ldi r16, 0b10110100
;	  out OSCCAL, r16
	  ret

	m_11xx:
	  sbrc r16,1
	  rjmp m_111x
	  sbrc r16,0
	  rjmp m_1101

	  ldi MassivH,HIGH(SinTabl_13*2)
;	  ldi r16, 0b11001000
;	  out OSCCAL, r16
	  ret

	m_1101:
	  ldi MassivH,HIGH(SinTabl_14*2)
;	  ldi r16, 0b11011100
;	  out OSCCAL, r16
	  ret

	m_111x:
	  sbrc r16,0
	  rjmp m_1111

	  ldi MassivH,HIGH(SinTabl_15*2)
;	  ldi r16, 0b11110000
;	  out OSCCAL, r16
	  ret

	m_1111:
	  ldi MassivH,HIGH(SinTabl_16*2)
;	  ldi r16, 0b11111010
;	  out OSCCAL, r16
ret

;***********************************************************
;***********************************************************

Pereschet_A:	; ���� A. ������ �������� ��� �������� � ������� ��������� �� ������ 1/4 ������� ������ , � ����������� �� ��������� 
	cpi cntA,N2		; ����� cntA � 126
	brlo Poluperiod_1A ; ���� ������ 126. �� ������� Poluperiod_1A
	;Poluperiod_2A
	cpi cntA,N3
	brlo Quad3A ; ���� ������ 189. �� ������� Quad3A
	; Quad4A
	ldi r16,251		; ����� (252-1)
	sub r16,cntA	; ������� ����� �����
	mov ZL,r16			;�������� � ������� Z �������� ����� ���������� ������ ��� ��������� OCR1AL
	rjmp Write_OCR1AL_A2pp

	Quad3A:

	mov ZL,cntA
	subi ZL,125		; ������� ����� ����� ��� 3-�� ���������

	Write_OCR1AL_A2pp:
	out OCR1AH,zero 	; Write 0 to OCR1AH
	lpm 			; lpm �������� ���: �� ������ Z ������ � ��������� ������� , (���� lpm r16,Z ), ���� � r0, ���� ������ �������� lpm )
	ldi r16,128
	sub r16,r0			; ��������� �� 128 ��� 2-�� �����������
	out OCR1AL,r16		; Write ����� to OCR1AL

	rjmp Prirashenie_A

	Poluperiod_1A:
	cpi cntA,N1		; ����� � ������� 2-�� ��������� -- 63
	brlo Quad1A ; ���� ������ 63. �� ������� Quad1A
	; Quad2A	���� ������ ��� ����� 63
	ldi r16,125		; ����� (126-1)
	sub r16,cntA	; ������� ����� �����
	mov ZL,r16			;�������� � ������� Z �������� ����� ���������� ������ ��� ��������� OCR1AL
	rjmp Write_OCR1AL_A1pp

	Quad1A:
	mov ZL,cntA			;�������� � ������� Z �������� ����� ���������� ������ ��� ��������� OCR1AL
	adiw Z,1		; ��������� 1-�� � ������, �.�. � 1-� �� �������� ����� �� � ���� � ������� �� ������
	
	Write_OCR1AL_A1pp:
	out OCR1AH,zero 	; Write 0 to OCR1AH
	lpm 			; lpm �������� ���: �� ������ Z ������ � ��������� ������� , (���� lpm r16,Z ), ���� � r0, ���� ������ �������� lpm )
	ldi r16,128
	add r16,r0			; ����������� � 128 ��� 1-�� �����������
	out OCR1AL,r16		; Write ����� to OCR1AL

	Prirashenie_A:
	; &&&& 	���������� ����� ������ �������� 
	ldi r16,Limit		; ������� � r16 ������ ����� ��� cntA cntB
	cp cntA,r16		;��������� ������� � � ��������
	brne A_inc			;���� ����� (Z=0), �� ����� � cntA "����� 1"
	ldi cntA,-1
	A_inc:
	inc cntA			; ���� �� ����� , �� ���������� �������� A

ret

Pereschet_B:
	; ���� B. ������ �������� ��� �������� � ������� ��������� �� ������ 1/4 ������� ������ , � ����������� �� ��������� 
	; ���������� ���� �
	cpi cntB,N2		; ����� cntB � 126
	brlo Poluperiod_1B 	; ���� ������ 126. �� ������� Poluperiod_1B
	;Poluperiod_2B
	cpi cntB,N3
	brlo Quad3B 	; ���� ������ 189. �� ������� Quad3B
	; Quad4B
	ldi r16,251		; ����� (252-1)
	sub r16,cntB	; ������� ����� �����
	mov ZL,r16			;�������� � ������� Z �������� ����� ���������� ������ ��� ��������� OCR1BL
	rjmp Write_OCR1BL_B2pp

	Quad3B:
	mov ZL,cntB
	subi ZL,125		; ������� ����� ����� ��� 3-�� ���������

	Write_OCR1BL_B2pp:
	out OCR1BH,zero 	; Write 0 to OCR1BH
	lpm 			; lpm �������� ���: �� ������ Z ������ � ��������� ������� , (���� lpm r16,Z ), ���� � r0, ���� ������ �������� lpm )
	ldi r16,128
	sub r16,r0			; ��������� �� 128 ��� 2-�� �����������
	out OCR1BL,r16		; Write ����� to OCR1BL

	rjmp Prirashenie_B

	Poluperiod_1B:
	cpi cntB,N1		; ����� � ������� 2-�� ��������� -- 63
	brlo Quad1B 	; ���� ������ 63. �� ������� Quad1B
	; Quad2B	���� ������ ��� ����� 63
	ldi r16,125		; ����� (126-1)
	sub r16,cntB	; ������� ����� �����
	mov ZL,r16			;�������� � ������� Z �������� ����� ���������� ������ ��� ��������� OCR1BL
	rjmp Write_OCR1BL_B1pp

	Quad1B:
	mov ZL,cntB			;�������� � ������� Z �������� ����� ���������� ������ ��� ��������� OCR1BL
	adiw Z,1		; ��������� 1-�� � ������, �.�. � 1-� �� �������� ����� �� � ���� � ������� �� ������
	
	Write_OCR1BL_B1pp:
	out OCR1BH,zero 	; Write 0 to OCR1BH
	lpm 			; lpm �������� ���: �� ������ Z ������ � ��������� ������� , (���� lpm r16,Z ), ���� � r0, ���� ������ �������� lpm )
	ldi r16,128
	add r16,r0			; ����������� � 128 ��� 1-�� �����������
	out OCR1BL,r16		; Write ����� to OCR1BL

	Prirashenie_B:
	; &&&& 	���������� ����� ������ �������� 
	ldi r16,Limit		; ������� � r16 ������ ����� ��� cntA cntB
	cp cntB,r16		;��������� ������� B � ��������
	brne B_inc			;���� ����� (Z=0), �� ����� � cntB "����� 1"
	ldi cntB,-1
	B_inc:
	inc cntB	; ���� �� ����� , �� ���������� �������� B

ret

Pereschet_C:	; ���� C. ������ �������� ��� �������� � ������� ��������� �� ������ 1/4 ������� ������ , � ����������� �� ��������� 
	cpi cntC,N2		; ����� cntC � 126
	brlo Poluperiod_1C 	; ���� ������ 126. �� ������� Poluperiod_1C
	;Poluperiod_2C
	cpi cntC,N3
	brlo Quad3C 	; ���� ������ 189. �� ������� Quad3C
	; Quad4C
	ldi r16,251		; ����� (252-1)
	sub r16,cntC	; ������� ����� �����
	mov ZL,r16			;�������� � ������� Z �������� ����� ���������� ������ ��� ��������� OCR2
	rjmp Write_OCR2_C2pp

	Quad3C:
	mov ZL,cntC
	subi ZL,125		; ������� ����� ����� ��� 3-�� ���������

	Write_OCR2_C2pp:
	lpm 			; lpm �������� ���: �� ������ Z ������ � ��������� ������� , (���� lpm r16,Z ), ���� � r0, ���� ������ �������� lpm )
	ldi r16,128
	sub r16,r0			; ��������� �� 128 ��� 2-�� �����������
	out OCR2,r16		; Write ����� to OCR2

	rjmp Prirashenie_C

	Poluperiod_1C:
	cpi cntC,N1		; ����� � ������� 2-�� ��������� -- 63
	brlo Quad1C 	; ���� ������ 63. �� ������� Quad1C
	; Quad2C	���� ������ ��� ����� 63
	ldi r16,125		; ����� (126-1)
	sub r16,cntC	; ������� ����� �����
	mov ZL,r16			;�������� � ������� Z �������� ����� ���������� ������ ��� ��������� OCR2
	rjmp Write_OCR2_C1pp

	Quad1C:
	mov ZL,cntC			;�������� � ������� Z �������� ����� ���������� ������ ��� ��������� OCR2
	adiw Z,1		; ��������� 1-�� � ������, �.�. � 1-� �� �������� ����� �� � ���� � ������� �� ������
	
	Write_OCR2_C1pp:
	lpm 			; lpm �������� ���: �� ������ Z ������ � ��������� ������� , (���� lpm r16,Z ), ���� � r0, ���� ������ �������� lpm )
	ldi r16,128
	add r16,r0			; ����������� � 128 ��� 1-�� �����������
	out OCR2,r16		; Write ����� to OCR2

	Prirashenie_C:
	ldi r16,Limit		; ������� � r16 ������ ����� ��� cntC
	cp cntC,r16		;��������� ������� C � ��������
	brne C_inc			;���� ����� (Z=0), �� ����� � cntC "����� 1"
	ldi cntC,-1
	C_inc:
	inc cntC	; ���� �� ����� , �� ���������� �������� C

ret

;***************************
;***************************
;********8888888888888************** 

Synch_Pulse:  ; &&&&&&& !!!!! ������������ �������� �������� ��� ��� ���� !!!!!!!!!
	sbic PINB, PINB0	; ���������� ��������� ����� ���� ��� ������� Skip if Bit in I/O Register Cleared
	CBI PORTB,0			; ������ ��� ������������ ��������� �������� �������������

	mov r16,DeltaFi	; ���� �������� 
	andi r16,0b11111000  ; ������� ����� 5 ��� (��� �������� Limit � ��� ����� ������� �������� ���� �� 32 ��������)
	; andi �������� ������ � ���������� >= R16
	cp r16,cntA	; 
	brne NetSdvigPuls			;���� �� ����� (Z=0), �� �� �������� ������� �� ����� PB0

	SBI PORTB,0			;������ ������� �� ����� PB0 (Z=1) --- ������� �14

	NetSdvigPuls:	 	
ret

;***************************
;********8888888888888************** 
Priem_Synchr_imp:	; ���� �������������� ���� ��� ���� �� ���� PC5. --- ������� �28
	; _SynchEnable !!! ������������ ��� ����!!!
	sbis PIND, PIND0	; ���������� ��������� ����� ���� �� ���� 1-�� !!! ������������ ��� ����!!!
	RJMP ExitSync
	; !!!
	; ���� ���� ������� �� ���� PC5, �� ����� ����� � ��� cntA/B/C �� ������ ������� + ������ ��� ���������� ���� ��� 1-� ����
	sbis PINC, PINC5	; ���������� ��������� ����� ���� �� ���� 1-��
	RJMP ExitSync
; &&&&&& ��� ����� ����������� ���������� �� ����� �������� (������ �� ���� �������, PINC5 - ����)
; &&&&&& � ��� ����� ����������� ���� �� ����������� ���������� (PINB0 - �����, PINC5 - ����)
;	ldi cntA,96		; ����� -125 ���� 
;	ldi cntB,12		; 
;	ldi cntC,180	; 
	ldi cntA,SdvigA		; ���������������� �������� ����� ������ ��������, ��� � ������ 13
	ldi cntB,SdvigB	; �������� ����� ����� -0,5 ���� (������ ���� ��������� ���� �� 0,5 ����)
	ldi cntC,SdvigC	; 

;	ldi cntA,198		; ����� +89 ���� 
;	ldi cntB,114		; 
;	ldi cntC,30		; 
	;********8888888888888************** 

	ExitSync:
ret

;&&&&&&&&&&&&&&&&&&&&&&&&&

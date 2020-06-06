.include "m8def.inc" ; подключить заголовочник
.include "Massiv_bait.asm"
;.тут я попробую выжать из Atmega8 три фазы ШИМа
; и снять их сразу в 2-х полярном режиме (т.е. на кажд фазу --- 2 выхода)
; новое знач запис-ся в рег сравн по переполн TmrCntr1 (т.е. и для 2-го счётчика по 1-му)

; ************** Interrupt Vectore Table ************************
.cseg		;directive defines the start of a Code Segment
.org	0x00				; Reset-Address (разместить следующий фрагмент программы в памяти начиная с адреса 0)
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
;	rjmp	Proverka_D6		; проверка нажата ли кнопка D6
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
	;подготовка режима PWM у 16ти битного таймера/счётчика "1" и у 8-ми битного таймера/счётчика "2"
	ldi xlam,0b11110001	; подготовка TCCR1A (COM1A1 COM1A0 COM1B1 COM1B0 FOC1A FOC1B WGM11 WGM10) (xxxxxx01 =8-ми битный ШИМ, 
	out TCCR1A,xlam		; xxxxxx10 =9-ти битный, xxxxxx11 =10-ти битный) ; COM1A1 COM1A0 COM1B1 COM1B0=11 -- Set OC1A/OC1B on Compare Match when up-counting. Clear OC1A/OC1B on Compare Match when downcounting.
	ldi xlam,0b00000010	; подготовка TCCR1B	(00000101 = f/1024, 00000100 = f/256, 00000011 = f/64)
	out TCCR1B,xlam	; 	; 					(00000010 = f/8, 00000001 = f )
	
	ldi xlam,0b01110010	; подготовка TCCR2 (FOC2 WGM20 COM21 COM20 WGM21 CS22 CS21 CS20)	
	out TCCR2,xlam		;(Bit 6,3 – WGM21:0: Waveform Generation Mode, Bit 5:4 – COM21:0: Compare Match Output Mode регулир перекл шим вверх или вниз 
				;(00000101 = f/1024, 00000100 = f/256, 00000011 = f/64, 00000010 = f/8, 00000001 = f , у этого таймера2 есть ещё f/32 , f/128)
				; 11 = Set OC2 on Compare Match when up-counting. Clear OC2 on Compare Match when downcounting.

	ldi xlam,0b11011100	; подготовка обнуления флагов прерывания по совпадениям и по переполнениям счётчиков 1 и 2
	out TIFR,xlam	; обнуление флагов прерывания по совпадениям счётчиков 1 и 2
;	ldi xlam,0b11011100	;подготовка установки маски разрешений отработки прерываний по совпадениям и переполнениям
	out TIMSK,r16	;установка маски разрешений отработки прерываний по совпадениям и переполнениям (разрешение пренрываний счётчиков 1 и 2)

	ldi xlam,0b00001110		; подготовка включения режима выход на пинах 1,2,3  порта B (DDRB --- Data Direction Port B )
	
	out DDRB,xlam	; Set Port B as output включение режима выход на пинах 1,2,3 порта B
					; пины 2 и 3 используются ещё и для прошивки (SS, MOSI SPI интерфейса)
	out DDRC,xlam	; Set Port C as output включение режима выход на пинах 1,2,3 порта C
					; с порта С попробую снимать сигнал в противофазе
;	cbi DDRD,PD6	; PD6 = 6 (Set PD6/ICP as input) 

;  ldi xlam,0xFF	;загружаем число FF в регистр xlam
;  out DDRB,xlam	;включаем регистр B в реж выхода , он будет показывать режим по частоте
;  in xlam,TCCR1B	;загружаем регистр TCCR1B в xlam
;  com xlam		; это чттобы нужные светодиоды горели, а не гасли
;  out PORTB,xlam	; вывод в регистр B будет показывать режима по частоте

	ldi MassivH,HIGH(SinTabl*2)	;загрузка в регистр MassivH старшего байта стартового адреса для сравнения OCR1AH
;	ldi MassivH3,HIGH(SinTabl3*2)	;загрузка в регистр MassivH3 старшего байта стартового адреса для сравнения OCR1BH
	
	eor cntA,cntA	;обнуление регистра для счётчика импульсов ШИМ A
	mov ZH,MassivH			;загрузка в регистр ZH старшего байта стартового адреса для сравнения OCR1AH
	mov ZL,cntA			;загрузка в регистр ZL младшего байта стартового адреса для сравнения OCR1AL

	out OCR1AH,zero 	; Write 0 to OCR1AH -- просто запишем 0,т.к. использую 8-ми битный ШИМ
	lpm 				; lpm работает так: грузит в регистр r0 данные, хранящ. по адресу в регистре Z 
	out OCR1AL,r0		; Write число to OCR1AL. Т.е., загрузили порог для сравнения A

	ldi cntB,13	;сдвиг фазы для счётчика импульсов ШИМ B
;	mov ZH,MassivH		; (это, наверное, уже есть, т.е. м убрать) загрузка в регистр ZH старшего байта стартового адреса для сравнения OCR1BH
	mov ZL,cntB			;загрузка в регистр ZL младшего байта стартового адреса для сравнения OCR1BL
	
	out OCR1BH,zero 	; Write 0 to OCR1BH -- просто запишем 0,т.к. использую 8-ми битный ШИМ
	lpm 				; lpm работает так: грузит в регистр r0 данные, хранящ. по адресу в регистре Z 
	out OCR1BL,r0 	; Write число to OCR1BL. Т.е., загрузили порог для сравнения B

	ldi cntC,26	;сдвиг фазы для счётчика импульсов ШИМ B
	mov ZL,cntC			;загрузка в регистр ZL младшего байта стартового адреса для сравнения OCR2
	
	lpm 				; lpm работает так: грузит в регистр r0 данные, хранящ. по адресу в регистре Z 
	out OCR2,r0 	; Write число to OCR2. Т.е., загрузили порог для сравнения C

	inc cntA	; приращение счётчика A
	inc cntB	; приращение счётчика B
	inc cntC	; приращение счётчика C
	ret	;Returns from subroutine. The return address is loaded from the STACK. The stack pointer uses a pre-increment scheme during RET.



; *** Begin of Program Execution ****************************************************
;************************************************************************************
Reset:
; Initialization of the Stack Pointer
	ldi	r16,low(RAMEND)		;
	out	SPL,r16				;Set Stack Pointer to top of RAM
	ldi	r16,high(RAMEND)	;
	out 	SPH,r16			;

	eor zero,zero 	; ноль в zero
	rcall	init_PWM1

	sei				; Global Interrupts enabled

; Endlessloop
loopForever:
	nop
	nop
;	sbic PIND, PIND6	
;	rjmp Smena_Freq		; если кнопка D6 нажата -- переход к Smena_Freq

	rjmp loopForever	; если кнопка D6 не нажата -- возврат к loopForever
			
;	ldi xlam,0b11110000 ; подготовка включения режима выход на пинах 4,5,6,7 порта D
;	out DDRD,xlam	; включение режима выход на пинах 4,5,6,7 порта D


Proverka_D6: ; действия по прерыванию OCR1A
;	push r16
	in r16,SREG
	push r16

	sbic PIND, PIND6	
	rjmp Smena_Freq		; если кнопка D6 нажата -- переход к Smena_Freq

	pop r16
	out SREG,r16
;	pop r16
	reti ; возврат из прерывания

Smena_Freq:		; переключение делителя тактовой частоты после наж кнопки D6

	in xlam,PIND	;гружу регистр PIND, содержащ текущ знач кнопки D6
	andi xlam,0b01000000	; побитовое И для выделения собственно знач кнопки D6
	cp xlam,zero	;сравниваю с нулём
	brne Smena_Freq	; возврат к проверке полож кнопки 6, пока её не отпустишь	

	in xlam,TCCR1B	;гружу регистр TCCR1B, содержащ текущ знач частоты в хлам
	andi xlam,0b00000111	; побитовое И для выделения собственно знач частоты
	ldi xlam2,5		;гружу в хлам2 пятёрку =00000101
	cp xlam,xlam2	;сравниваю с 00000101
	brne IncXlam	;перескок, если не равно
	ldi xlam,0 ; гружу в хлам 00000000, если текущее TCCR1B равно 5

IncXlam:
	inc xlam	; увеличиваю хлам на 1	(00000101 = f/1024, 00000100 = f/256, 00000011 = f/64)
	out TCCR1B,xlam	; 	; 					(00000010 = f/8, 00000001 = f )

	ldi xlam,0 ; подготовка сброса кнопки D6 
	out PIND,xlam	; сброс D6 
; отображение сост TCCR1B светодиодами:
in xlam,TCCR1B	;загружаем регистр TCCR1B в хлам
com xlam		; это чттобы нужные светодиоды горели, а не гасли
out PORTB,xlam	; регистр B будет показывать режим по частоте

	rjmp loopForever	; возврат к loopForever



TIM1_Overflow:
	;здесь надо сделать проверку, что cntA cntB не больше 39
;	out TCNT2,zero	; это м.б лишнее: для точной привязки фазы TimerCounter2 к 1-му коунтеру
	in r16,SREG		;Load an I/O Location to Register
	push r16
	
	mov ZH,MassivH		;загрузка в регистр Z старшего байта стартового адреса для сравнения OCR1AH
	mov ZL,cntA			;загрузка в регистр Z младшего байта стартового адреса для сравнения OCR1AL

	out OCR1AH,zero 	; Write 0 to OCR1AH
	lpm 				; lpm работает так: по ссылке Z грузит в r0
	out OCR1AL,r0		; Write число to OCR1AL

;	mov ZH,MassivH3 	;загрузка в регистр Z старшего байта стартового адреса для сравнения OCR1BH
	mov ZL,cntB			;загрузка в регистр Z младшего байта стартового адреса для сравнения OCR1BL
	
	out OCR1BH,zero 	; Write 0 to OCR1BH
	lpm 				; lpm работает так: по ссылке Z грузит в r0
	out OCR1BL,r0 	; Write число to OCR1BL

	mov ZL,cntC			;загрузка в регистр ZL младшего байта стартового адреса для сравнения OCR2
	lpm 				; lpm работает так: грузит в регистр r0 данные, хранящ. по адресу в регистре Z 
	out OCR2,r0 	; Write число to OCR2. Т.е., загрузили порог для сравнения C


	ldi xlam,Limit		; помещаю в r16 предел счёта для cntA cntB
	cp cntA,xlam		;сравниваю счетчик А с пределом
	brne A_inc			;если равно (Z=0), то гружу в cntA "минус 1"
	ldi cntA,-1
	A_inc:
	inc cntA			; если не равно , то приращение счётчика A

;	ldi xlam,Limit		; помещаю в r16 предел счёта для cntA cntB
	cp cntB,xlam		;сравниваю счетчик B с пределом
	brne B_inc			;если равно (Z=0), то гружу в cntB "минус 1"
	ldi cntB,-1
	B_inc:
	inc cntB	; если не равно , то приращение счётчика B

	cp cntC,xlam		;сравниваю счетчик C с пределом
	brne C_inc			;если равно (Z=0), то гружу в cntC "минус 1"
	ldi cntC,-1
	C_inc:
	inc cntC	; если не равно , то приращение счётчика C


	pop r16
	out SREG,r16
		
	reti	;Returns from interrupt. The return address is loaded from the STACK and the global interrupt flag is set.


TIM1A_CompMatch:		; здесь переключаю состояние выходного бита PORTB0 по каждому совпадению TIM1A_CompMatch

	in r16,SREG		;Load an I/O Location to Register
	push r16

	sbic PINC, PINC1	; Пропустить следующую инстр если бит Нулевой Skip if Bit in I/O Register Cleared
;	rjmp SetOutB0
	CBI PORTC,1
	sbis PINC, PINC1	; Пропустить следующую инстр если бит Единица Skip if Bit in I/O Register Set
	SBI PORTC,1

	pop r16
	out SREG,r16
		

	reti	;Returns from interrupt. The return address is loaded from the STACK and the global interrupt flag is set.

TIM1B_CompMatch:		; здесь переключаю состояние выходного бита PORTB0 по каждому совпадению TIM1A_CompMatch

	in r16,SREG		;Load an I/O Location to Register
	push r16

	sbic PINC, PINC2	; Пропустить следующую инстр если бит Нулевой Skip if Bit in I/O Register Cleared
;	rjmp SetOutB0
	CBI PORTC,2
	sbis PINC, PINC2	; Пропустить следующую инстр если бит Единица Skip if Bit in I/O Register Set
	SBI PORTC,2

	pop r16
	out SREG,r16
		

	reti	;Returns from interrupt. The return address is loaded from the STACK and the global interrupt flag is set.



TIM2_CompMatch:		; здесь переключаю состояние выходного бита PORTB0 по каждому совпадению TIM2_CompMatch

	in r16,SREG		;Load an I/O Location to Register
	push r16

	sbic PINC, PINB3	; Пропустить следующую инстр если бит Нулевой Skip if Bit in I/O Register Cleared
;	rjmp SetOutB0
	CBI PORTC,3
	sbis PINC, PINB3	; Пропустить следующую инстр если бит Единица Skip if Bit in I/O Register Set
	SBI PORTC,3

	pop r16
	out SREG,r16
		

	reti	;Re

;***********************************************************



;150519  at8_PWM3_03_14  Прошивка для контроллера напряжения. 
;(Для тока пока остаётся v13 252 точки на период и большие искажения в крайнем положении ручки амплитуды)
;Сделан алгоритм перерасчёта значений для сравнения по 1/4 синуса (от 0 до макс)
;при чём в 1 и 3 квадранте в расчёт берутся точки от1 до 63-й, а во 2-м и 4-м --- от 62-й до нулевой
;Всего 252 точки на период, смещение фаз через 84 точки
;Откорректировано смещение по фазе контроллера тока (надо, чтобы число не превышало 251=Limit, сейчас макс = 248 (bin 11111000)
;Откорректированы константы для синхронизации контроллера напряжения от сетевой фазы
;Выделены в отдельные подпрограммы : перерасчёт значений по фазам A B C , выработка и приём синхроимпульса 
.include "m8def_03_14.inc" ; подключить заголовочник  
.include "Massiv_bait_Napr_63x4.asm"		; Limit	= 251
; .include "Massiv_bait_Tok_252.asm"		; Limit	= 251

; ************** Interrupt Vectore Table ************************
.cseg		;directive defines the start of a Code Segment
.org	0x00				; Reset-Address (разместить следующий фрагмент программы в памяти начиная с адреса 0)
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
	;подготовка режима PWM у 16ти битного таймера/счётчика "1" и у 8-ми битного таймера/счётчика "2"
	ldi r16,0b11110001	; подготовка TCCR1A (COM1A1 COM1A0 COM1B1 COM1B0 FOC1A FOC1B WGM11 WGM10) (xxxxxx01 =8-ми битный ШИМ, 
	out TCCR1A,r16		; xxxxxx10 =9-ти битный, xxxxxx11 =10-ти битный) ; COM1A1 COM1A0 COM1B1 COM1B0=11 -- Set OC1A/OC1B on Compare Match when up-counting. Clear OC1A/OC1B on Compare Match when downcounting.
	ldi r16,0b00000001	; подготовка TCCR1B	(00000101 = f/1024, 00000100 = f/256, 00000011 = f/64)
	out TCCR1B,r16	; 	; 					(00000010 = f/8, 00000001 = f )

	ldi r16,0b01110001	; подготовка TCCR2 (FOC2 WGM20 COM21 COM20 WGM21 CS22 CS21 CS20)	
	out TCCR2,r16		;(Bit 6,3 – WGM21:0: Waveform Generation Mode, Bit 5:4 – COM21:0: Compare Match Output Mode регулир перекл шим вверх или вниз 
				;(00000101 = f/1024, 00000100 = f/256, 00000011 = f/64, 00000010 = f/8, 00000001 = f , у этого таймера2 есть ещё f/32 , f/128)
				; 11 = Set OC2 on Compare Match when up-counting. Clear OC2 on Compare Match when downcounting.
;&&&&&&&&&&; добавил 150504
;	ldi r16,0b00000100 
;	out TCNT2,r16	; записываю в регистр счётчика Timer2 значение равное счёту в счётчике Timer1 на следующем такте. 
					; просто чтобы выровнять фазы точно под 120 градусов
; 150507 закомментировал: похоже, что это сразу сбивает генерацию. Т.е., приоритеты прерываний тут не работают
;&&&&&&&&&&
	ldi r16,0b11011100	; подготовка обнуления флагов прерывания по совпадениям и по переполнениям счётчиков 1 и 2
	out TIFR,r16	; обнуление флагов прерывания по совпадениям счётчиков 1 и 2
;	ldi r16,0b11011100	;подготовка установки маски разрешений отработки прерываний по совпадениям и переполнениям
	out TIMSK,r16	;установка маски разрешений отработки прерываний по совпадениям и переполнениям (разрешение пренрываний счётчиков 1 и 2)

	ldi r16,0b00001111		; подготовка включения режима выход на пинах 0,1,2,3  порта B (DDRB --- Data Direction Port B )
	
	out DDRB,r16	; Set Port B as output включение режима выход на пинах 0,1,2,3 порта B
					; пины 2 и 3 используются ещё и для прошивки (SS, MOSI SPI интерфейса)
;	ldi r16,0b00011100		; подготовка включения режима выход на пинах 2,3,4,5  порта C (DDRC --- Data Direction Port C )
;	out DDRC,r16	; Set Port C as output включение режима выход на пинах 1,2,3 порта C
					; с порта С попробую снимать сигнал в противофазе

	ldi MassivH,HIGH(SinTabl_1*2)	;для теста напряжения (измен по частоте) 50% амплитуды загрузка в регистр MassivH старшего байта стартового адреса для сравнения OCR1AH
;	ldi MassivH,HIGH(SinTabl_125*2)	;для теста тока (измен по частоте) 125% амплитуды загрузка в регистр MassivH старшего байта стартового адреса для сравнения OCR1AH

	ldi cntA,SdvigA		;сдвиг № ШИМ-импульса от начала отсчёта фазы A (обычно = 0)
	mov ZH,MassivH			;загрузка в регистр ZH старшего байта стартового адреса для сравнения OCR1AH
	mov ZL,cntA			;загрузка в регистр ZL младшего байта стартового адреса для сравнения OCR1AL


	out OCR1AH,zero 	; Write 0 to OCR1AH -- просто запишем 0,т.к. использую 8-ми битный ШИМ
	lpm 				; lpm работает так: грузит в регистр r0 данные, хранящ. по адресу в регистре Z 
	out OCR1AL,r0		; Write число to OCR1AL. Т.е., загрузили порог для сравнения A

	ldi cntB,SdvigB	;сдвиг № ШИМ-импульса от начала отсчёта фазы (B -A) 
;	mov ZH,MassivH		; (это, наверное, уже есть, т.е. м убрать) загрузка в регистр ZH старшего байта стартового адреса для сравнения OCR1BH
	mov ZL,cntB			;загрузка в регистр ZL младшего байта стартового адреса для сравнения OCR1BL
	
	out OCR1BH,zero 	; Write 0 to OCR1BH -- просто запишем 0,т.к. использую 8-ми битный ШИМ
	lpm 				; lpm работает так: грузит в регистр r0 данные, хранящ. по адресу в регистре Z 
	out OCR1BL,r0 	; Write число to OCR1BL. Т.е., загрузили порог для сравнения B

	ldi cntC,SdvigC	;сдвиг № ШИМ-импульса от начала отсчёта фазы (C -A) 
	mov ZL,cntC			;загрузка в регистр ZL младшего байта стартового адреса для сравнения OCR2
	
	lpm 				; lpm работает так: грузит в регистр r0 данные, хранящ. по адресу в регистре Z 
	out OCR2,r0 	; Write число to OCR2. Т.е., загрузили порог для сравнения C

	inc cntA	; приращение счётчика A
	inc cntB	; приращение счётчика B
	inc cntC	; приращение счётчика C
	
;	clr SchetA	; это экспериментальн счетчики текущ позиции противофазных выходов A-A',B-B',C-C'
;	clr SchetB	; проверяется всего лишь младш бит
;	clr SchetC
	clr Amplituda	; этот регистр б содержать старш байт адреса текущей таблицы синуса
	clr DeltaFi	; этот регистр б содержать число соотв смещению имп синхр тока
	clr vhodADC	; младшие 2 бита регистра vhodADC б. опред. какой вход использует АЦП (сейчас подпаяно 3 входа ADC0 ADC1 ADC2)
	
	ret	;Returns from subroutine. The return address is loaded from the STACK. The stack pointer uses a pre-increment scheme during RET.


init_ADC:

	ldi r16,0b01100000		; гружу ADMUX (REFS1 REFS0 ADLAR – MUX3 MUX2 MUX1 MUX0) т.е сначала считываем вход ADC0
	out ADMUX,r16
	ldi r16,0b11111111		; гружу ADCSRA (ADEN ADSC ADFR ADIF ADIE ADPS2 ADPS1 ADPS0) 3 младш бита устан деление частоты
	out ADCSRA,r16

;	ldi r16,0b11111111		; гружу ADCSRA (ADEN ADSC ADFR ADIF ADIE ADPS2 ADPS1 ADPS0)
;	out DDRD,r16		; это назначаю как временный вывод состояния АЦП для проверки

	ret	;Returns from subroutine. The return address is loaded from the STACK. The stack pointer uses a pre-increment scheme during RET.


init_OSCCAL:
	ldi r16,OSCCAL_synchr		; записываю в OSCCAL подобранное значение для частоты 50Гц при 8МГц и 252 тоек
	out OSCCAL, r16		;заводские константы: OSCCAL	= B9, BA, B4 или B6 для AT8-напряжения)
	ret	;Returns from subroutine. ; ( OSCCAL	= BD, BC, B5 или B7 для AT8-тока)
									;(вероятно, для частот 1, 2., 4,  8  МГц
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
	; перезагрузка номеров для сравнения по фазам A и B
	; проверка, что cntA cntB не больше Limit (в первой версии 78 точек: от 0 до 77)
	; Приём синхроимпульса смещ фаз тока на пине PC5. 
;	out TCNT2,zero	; для точной привязки фазы TimerCounter2 к 1-му коунтеру --- это лишнее
	in r16,SREG		;Load an I/O Location to Register
	push r16

	mov ZH,MassivH		;загрузка в регистр Z старшего байта стартового адреса для сравнения OCR1AH
	rcall Pereschet_A	; основная процедура расчёта и занесения числа в регистр сравнения OCR1A
	rcall Pereschet_B	; основная процедура расчёта и занесения числа в регистр сравнения OCR1B

	rcall Priem_Synchr_imp	; Приём синхроимпульса смещ фаз тока на пине PC5 (ножка №28).

	pop r16
	out SREG,r16
		
	reti	;Returns from interrupt. The return address is loaded from the STACK and the global interrupt flag is set.


TIM2_Overflow:
	in r16,SREG		;Load an I/O Location to Register
	push r16
	
	mov ZH,MassivH		;загрузка в регистр Z старшего байта стартового адреса для сравнения 
	rcall Pereschet_C	; основная процедура расчёта и занесения числа в регистр сравнения OCR2

	rcall Synch_Pulse	; 

	pop r16
	out SREG,r16

	reti	;Returns from interrupt. The return address is loaded from the STACK and the global interrupt flag is set.


Refresh_ADC:
	in r16,SREG		;Load an I/O Location to Register
	push r16

	sbrc vhodADC,0	; младшие 2 бита регистра vhodADC б. опред. какой вход использует АЦП (сейчас подпаяно 3 входа ADC0 ADC1 ADC2). 
					;т.е. чему соответствует регистр ADCH: 00 --- амплитуда (ADC0), 01 --- фаза (ADC1), 10 --- частота (ADC2). 
;******** test 150504
;	in r16,ADMUX
;	sbrs r16,0	; 0-й бит опред чему соответствует регистр ADCH, ампл или фазе. т.е с какого пина снимаю ацп
;********	
	RJMP VhodDeltaFi

	sbrc vhodADC,1	; xxxxxx10 соответствует входу ADC2 --- Freq
	RJMP VhodFreq

;******
	;&&&&&&&&&
	;Vhod-Amplituda (считанное знач соотв входу ADC0)

	in r16,ADCH
	andi r16,0b11110000  ; выделяю старш 4 бита
	swap r16		; обмен 4-вёрок бит в байте (вообще м и без этого, но тогда надо править RefreshAdres:)

	sub Amplituda,r16	; вычитаю из предыдущ значения 
	breq Exit1			; выход, если значение не изменилось
	rcall RefreshAdres

	Exit1:
	mov Amplituda,r16
	ldi r16,0b01111111		; делаю disable ADC гружу ADCSRA (ADEN ADSC ADFR ADIF ADIE ADPS2 ADPS1 ADPS0) 3 младш бита устан деление частоты
	out ADCSRA,r16			; test

	ldi r16,0b01100001		; гружу ADMUX: в след раз б считывать вход ADC1
	out ADMUX,r16
	ldi r16,0b11111111		; снова запускаю АЦП
	out ADCSRA,r16			; 

	inc vhodADC
	RJMP Exit2


	;&&&&&&&&&
	VhodDeltaFi:	;считанное знач соотв входу ADC1

	in DeltaFi,ADCH
;	mov DeltaFi,r16
	ldi r16,0b01111111		; делаю disable ADC гружу ADCSRA (ADEN ADSC ADFR ADIF ADIE ADPS2 ADPS1 ADPS0) 3 младш бита устан деление частоты
	out ADCSRA,r16			; test

	ldi r16,0b01100010		; гружу ADMUX: в след раз б считывать вход ADC2
	out ADMUX,r16
	ldi r16,0b11111111		; снова запускаю АЦП
	out ADCSRA,r16			; 

	inc vhodADC

	RJMP Exit2

	;&&&&&&&&&
	VhodFreq:	
	
	sbic PIND, PIND0	; Пропустить следующую инстр если на пине 0 (проверка полож выключателя синхр от сети)
	RJMP OSCCAL_pri_synchr		; Отключение ручной регулировки частоты при синхрониз от сети
	; !!!
	in r16,ADCH		;считанное знач соотв входу ADC2 подкл к потенц частоты
;	andi r16,0b1111110  ; выделяю 7 старш бит  для уменьш дребезга --- наверное лишнее
	lsr r16			;сдвиг на 1 бит вправо
	lsr r16			;и ещё 1 сдвиг. Теперь диапазон регулировки 0--63 ед.
	ldi r17,FreqMin
	add r16,r17	; прибавляю к 150-ти, получаю диапазон 150--213ед.
	cp Freq,r16   ;--- пока что Freq нужен только для этого сравнения. Возможно, что и не нужен вообще
	breq NextVhodADC			;если равно (флог Z=1), то перейти к метке	
	out OSCCAL,r16	;просто гружу значение от АЦП (старшие биты ) в калибровочный регистр встроенного RC-генератора  
	mov Freq,r16	

	NextVhodADC:		; переключение № входа АЦП
	ldi r16,0b01111111		; делаю disable ADC гружу ADCSRA (ADEN ADSC ADFR ADIF ADIE ADPS2 ADPS1 ADPS0) 3 младш бита устан деление частоты
	out ADCSRA,r16			; test

	ldi r16,0b01100000		; гружу ADMUX: в след раз б считывать вход ADC0
	out ADMUX,r16
	ldi r16,0b11111111		; снова запускаю АЦП
	out ADCSRA,r16			; 

	ldi vhodADC,0b00000000	; здесь просто использую ldi т.к. для vhodADC выбрал регистр R18 (напрямую вроде можно грузить константы начиная с 16-го регистра, если правильно помню)

	RJMP Exit2

	OSCCAL_pri_synchr:
	in r16,OSCCAL
	ldi r17,OSCCAL_synchr		; константа от Атмела для контроллера напряж -- hex B9/BA/B4/B6 при номинале 1/2/4/8 МГц)
						; константа от Атмела для контроллера тока -- hex BD/BC/B5/B7 при номинале 1/2/4/8 МГц)
	cp r16,r17
	breq NextVhodADC		; Если равно, но на выход
	out OSCCAL,r17		; Записать заводскую конст
	RJMP NextVhodADC

	Exit2:
	pop r16
	out SREG,r16
	reti	;возврат из прерывания


;***********************************************************
;***********************************************************

RefreshAdres:		; подпрограмма изменения стартового адреса таблицы Massiv_bait
	  sbrc r16,3
	  rjmp m_1xxx	; оптимизация от Д.Д.
	  sbrc r16,2
	  rjmp m_01xx
	  sbrc r16,1
	  rjmp m_001x
	  sbrc r16,0
	  rjmp m_0001

	  ldi MassivH,HIGH(SinTabl_1*2)
;	  ldi r16, 0b00000100			; сделал для теста рост частоты с ростом напряжения
;	  out OSCCAL, r16				; при чём взял просто удвоенный номер амплитуды -->> OSCCAL (кроме нулевого номера)
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

Pereschet_A:	; Фаза A. Расчёт значения для загрузки в регистр сравнения по данным 1/4 периода синуса , в зависимости от квадранта 
	cpi cntA,N2		; сравн cntA с 126
	brlo Poluperiod_1A ; если меньше 126. то переход Poluperiod_1A
	;Poluperiod_2A
	cpi cntA,N3
	brlo Quad3A ; если меньше 189. то переход Quad3A
	; Quad4A
	ldi r16,251		; гружу (252-1)
	sub r16,cntA	; получаю младш адрес
	mov ZL,r16			;загрузка в регистр Z младшего байта стартового адреса для сравнения OCR1AL
	rjmp Write_OCR1AL_A2pp

	Quad3A:

	mov ZL,cntA
	subi ZL,125		; получаю младш адрес для 3-го квадранта

	Write_OCR1AL_A2pp:
	out OCR1AH,zero 	; Write 0 to OCR1AH
	lpm 			; lpm работает так: по ссылке Z грузит в указанный регистр , (напр lpm r16,Z ), либо в r0, если просто написать lpm )
	ldi r16,128
	sub r16,r0			; вычитание из 128 для 2-го полупериода
	out OCR1AL,r16		; Write число to OCR1AL

	rjmp Prirashenie_A

	Poluperiod_1A:
	cpi cntA,N1		; сравн с началом 2-го квадранта -- 63
	brlo Quad1A ; если меньше 63. то переход Quad1A
	; Quad2A	если больше или равно 63
	ldi r16,125		; гружу (126-1)
	sub r16,cntA	; получаю младш адрес
	mov ZL,r16			;загрузка в регистр Z младшего байта стартового адреса для сравнения OCR1AL
	rjmp Write_OCR1AL_A1pp

	Quad1A:
	mov ZL,cntA			;загрузка в регистр Z младшего байта стартового адреса для сравнения OCR1AL
	adiw Z,1		; прибавляю 1-цу к адресу, т.к. в 1-м кв значения начин не с нуля и доходят до максим
	
	Write_OCR1AL_A1pp:
	out OCR1AH,zero 	; Write 0 to OCR1AH
	lpm 			; lpm работает так: по ссылке Z грузит в указанный регистр , (напр lpm r16,Z ), либо в r0, если просто написать lpm )
	ldi r16,128
	add r16,r0			; прибавление к 128 для 1-го полупериода
	out OCR1AL,r16		; Write число to OCR1AL

	Prirashenie_A:
	; &&&& 	приращение счётч номера импульса 
	ldi r16,Limit		; помещаю в r16 предел счёта для cntA cntB
	cp cntA,r16		;сравниваю счетчик А с пределом
	brne A_inc			;если равно (Z=0), то гружу в cntA "минус 1"
	ldi cntA,-1
	A_inc:
	inc cntA			; если не равно , то приращение счётчика A

ret

Pereschet_B:
	; Фаза B. Расчёт значения для загрузки в регистр сравнения по данным 1/4 периода синуса , в зависимости от квадранта 
	; аналогично фазе А
	cpi cntB,N2		; сравн cntB с 126
	brlo Poluperiod_1B 	; если меньше 126. то переход Poluperiod_1B
	;Poluperiod_2B
	cpi cntB,N3
	brlo Quad3B 	; если меньше 189. то переход Quad3B
	; Quad4B
	ldi r16,251		; гружу (252-1)
	sub r16,cntB	; получаю младш адрес
	mov ZL,r16			;загрузка в регистр Z младшего байта стартового адреса для сравнения OCR1BL
	rjmp Write_OCR1BL_B2pp

	Quad3B:
	mov ZL,cntB
	subi ZL,125		; получаю младш адрес для 3-го квадранта

	Write_OCR1BL_B2pp:
	out OCR1BH,zero 	; Write 0 to OCR1BH
	lpm 			; lpm работает так: по ссылке Z грузит в указанный регистр , (напр lpm r16,Z ), либо в r0, если просто написать lpm )
	ldi r16,128
	sub r16,r0			; вычитание из 128 для 2-го полупериода
	out OCR1BL,r16		; Write число to OCR1BL

	rjmp Prirashenie_B

	Poluperiod_1B:
	cpi cntB,N1		; сравн с началом 2-го квадранта -- 63
	brlo Quad1B 	; если меньше 63. то переход Quad1B
	; Quad2B	если больше или равно 63
	ldi r16,125		; гружу (126-1)
	sub r16,cntB	; получаю младш адрес
	mov ZL,r16			;загрузка в регистр Z младшего байта стартового адреса для сравнения OCR1BL
	rjmp Write_OCR1BL_B1pp

	Quad1B:
	mov ZL,cntB			;загрузка в регистр Z младшего байта стартового адреса для сравнения OCR1BL
	adiw Z,1		; прибавляю 1-цу к адресу, т.к. в 1-м кв значения начин не с нуля и доходят до максим
	
	Write_OCR1BL_B1pp:
	out OCR1BH,zero 	; Write 0 to OCR1BH
	lpm 			; lpm работает так: по ссылке Z грузит в указанный регистр , (напр lpm r16,Z ), либо в r0, если просто написать lpm )
	ldi r16,128
	add r16,r0			; прибавление к 128 для 1-го полупериода
	out OCR1BL,r16		; Write число to OCR1BL

	Prirashenie_B:
	; &&&& 	приращение счётч номера импульса 
	ldi r16,Limit		; помещаю в r16 предел счёта для cntA cntB
	cp cntB,r16		;сравниваю счетчик B с пределом
	brne B_inc			;если равно (Z=0), то гружу в cntB "минус 1"
	ldi cntB,-1
	B_inc:
	inc cntB	; если не равно , то приращение счётчика B

ret

Pereschet_C:	; Фаза C. Расчёт значения для загрузки в регистр сравнения по данным 1/4 периода синуса , в зависимости от квадранта 
	cpi cntC,N2		; сравн cntC с 126
	brlo Poluperiod_1C 	; если меньше 126. то переход Poluperiod_1C
	;Poluperiod_2C
	cpi cntC,N3
	brlo Quad3C 	; если меньше 189. то переход Quad3C
	; Quad4C
	ldi r16,251		; гружу (252-1)
	sub r16,cntC	; получаю младш адрес
	mov ZL,r16			;загрузка в регистр Z младшего байта стартового адреса для сравнения OCR2
	rjmp Write_OCR2_C2pp

	Quad3C:
	mov ZL,cntC
	subi ZL,125		; получаю младш адрес для 3-го квадранта

	Write_OCR2_C2pp:
	lpm 			; lpm работает так: по ссылке Z грузит в указанный регистр , (напр lpm r16,Z ), либо в r0, если просто написать lpm )
	ldi r16,128
	sub r16,r0			; вычитание из 128 для 2-го полупериода
	out OCR2,r16		; Write число to OCR2

	rjmp Prirashenie_C

	Poluperiod_1C:
	cpi cntC,N1		; сравн с началом 2-го квадранта -- 63
	brlo Quad1C 	; если меньше 63. то переход Quad1C
	; Quad2C	если больше или равно 63
	ldi r16,125		; гружу (126-1)
	sub r16,cntC	; получаю младш адрес
	mov ZL,r16			;загрузка в регистр Z младшего байта стартового адреса для сравнения OCR2
	rjmp Write_OCR2_C1pp

	Quad1C:
	mov ZL,cntC			;загрузка в регистр Z младшего байта стартового адреса для сравнения OCR2
	adiw Z,1		; прибавляю 1-цу к адресу, т.к. в 1-м кв значения начин не с нуля и доходят до максим
	
	Write_OCR2_C1pp:
	lpm 			; lpm работает так: по ссылке Z грузит в указанный регистр , (напр lpm r16,Z ), либо в r0, если просто написать lpm )
	ldi r16,128
	add r16,r0			; прибавление к 128 для 1-го полупериода
	out OCR2,r16		; Write число to OCR2

	Prirashenie_C:
	ldi r16,Limit		; помещаю в r16 предел счёта для cntC
	cp cntC,r16		;сравниваю счетчик C с пределом
	brne C_inc			;если равно (Z=0), то гружу в cntC "минус 1"
	ldi cntC,-1
	C_inc:
	inc cntC	; если не равно , то приращение счётчика C

ret

;***************************
;***************************
;********8888888888888************** 

Synch_Pulse:  ; &&&&&&& !!!!! формирование импульса смещения для фаз тока !!!!!!!!!
	sbic PINB, PINB0	; Пропустить следующую инстр если бит Нулевой Skip if Bit in I/O Register Cleared
	CBI PORTB,0			; просто для формирования окончания импульса синхронизации

	mov r16,DeltaFi	; беру смещение 
	andi r16,0b11111000  ; выделяю старш 5 бит (для неправыш Limit и для более чёткого смещения фазы по 32 позициям)
	; andi работает только с регистрами >= R16
	cp r16,cntA	; 
	brne NetSdvigPuls			;если не равно (Z=0), то не выдавать импульс на ножку PB0

	SBI PORTB,0			;выдать импульс на ножку PB0 (Z=1) --- контакт №14

	NetSdvigPuls:	 	
ret

;***************************
;********8888888888888************** 
Priem_Synchr_imp:	; Приём синхроимпульса смещ фаз тока на пине PC5. --- контакт №28
	; _SynchEnable !!! Закомментить для тока!!!
	sbis PIND, PIND0	; Пропустить следующую инстр если на пине 1-ца !!! Закомментить для тока!!!
	RJMP ExitSync
	; !!!
	; Если есть импульс на пине PC5, то сброс счёта № ШИМ cntA/B/C на исходн позицию + дельта для подстройки сети под 1-ю фазу
	sbis PINC, PINC5	; Пропустить следующую инстр если на пине 1-ца
	RJMP ExitSync
; &&&&&& Для синхр контроллера напряжения от внешн импульса (наприм от сети питания, PINC5 - вход)
; &&&&&& и Для синхр контроллера тока от контроллера напряжения (PINB0 - выход, PINC5 - вход)
;	ldi cntA,96		; сдвиг -125 град 
;	ldi cntB,12		; 
;	ldi cntC,180	; 
	ldi cntA,SdvigA		; Экспериментально подобрал Здесь другие значения, чем в версии 13
	ldi cntB,SdvigB	; точность синхр около -0,5 град (первая фаза опережает сеть на 0,5 град)
	ldi cntC,SdvigC	; 

;	ldi cntA,198		; сдвиг +89 град 
;	ldi cntB,114		; 
;	ldi cntC,30		; 
	;********8888888888888************** 

	ExitSync:
ret

;&&&&&&&&&&&&&&&&&&&&&&&&&

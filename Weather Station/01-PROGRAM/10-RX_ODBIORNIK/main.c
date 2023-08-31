#define F_CPU 4000000UL

//BIBLIOTEKI----------------------------------------------------------------------------------------------------------------
//blblioteki podstawowe
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <avr/sleep.h>
#include <time.h>
#include <avr/wdt.h>
#include <math.h>

//blblioteki dodatkowe
#include "rfm69.h"
#include "SSD1306.h"

//PORTY----------------------------------------------------------------------------------------------------------------
//definicja portów
#define LED_1 PB0 //led
#define SW PD3 //przycisk
#define V_BATT PC0	//pomiar napiecia bateria

//STA£E----------------------------------------------------------------------------------------------------------------
//stale wartosci 
#define U_max_batt 4.18 //napiecie maksymalne baterii; 4.2 po dzielniku to 2.1
#define U_min_batt 3.0 //napiecie minimalne baterii; 3.0 po dzielniku to 1.5
#define U_coff 2.0 //podzial napiecia przez dzielnik baterii
#define U_max_sol 1.47 //wspolczynnik pomnozenia 3,73; Vmax 5,5V
#define U_min_sol 0.0 //napiecie minimalne panelu slonecznego	0
#define U_batt_low 10 //prog ladowania	

//pozostale parametry
#define table_size 6	//wielkosc tablicy
#define temp_offset 50	//offset odejmowania temperatury
#define press_offset 850	//offset dodawania cisnienia
#define sec_sleep 15 // czas uspienia w sekundach
#define sec_alarm 60 //czas alarmu w sekundach
#define sec_reset 86400 //czas resetu w sekundach - obecnie doba
#define confirm 94 // znacznik potwierdzenia 

//DEFINICJE TABLIC----------------------------------------------------------------------------------------------------------------
volatile uint8_t send[table_size];	//tablica odbierana
volatile char buf_t [4];	//tablica stanowiaca bufor 
volatile char buf_p [4];	//tablica stanowiaca bufor 
volatile char buf_h [3];	//tablica stanowiaca bufor 
volatile char buf_s [3];	//tablica stanowiaca bufor 
volatile char buf_batt_tx [3];	//tablica stanowiaca bufor 
volatile char buf_batt_rx [3];	//tablica stanowiaca bufor 

//DEFINICJE ZMIENNYCH----------------------------------------------------------------------------------------------------------------
//zmienne niezbedne do obliczen parametrów
volatile uint8_t batt_u_tx = 0;
volatile uint8_t batt_u_rx = 0;
volatile uint8_t soll_u = 0;
volatile int8_t temp = 0;
volatile int8_t temp_1 = 0;
volatile uint16_t press = 0;
volatile uint8_t hum = 0;

//zmienne uspienia
volatile uint64_t sec_timer = 0;	//zmienne wybudzania
volatile uint64_t sec_timer_1 = 0;	//zmienne wybudzania
volatile uint64_t sec_timer_2 = 0;	//zmienne wybudzania
volatile uint8_t mode_sw = 0; //zmienne przelacznika
volatile uint8_t tx_batt_mode = 0; //tryb baterii nadajnika
volatile uint8_t rx_batt_mode = 0; //tryb baterii nadajnika
volatile uint8_t rx_status = 0; //status odebrania wiadomoœci
volatile uint8_t rx_status_start = 0; //status odebrania wiadomoœci na starcie

//PETLE----------------------------------------------------------------------------------------------------------------

//mapowanie danych WORK
float map(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//funkcja constrain - WORK
uint8_t constrain (uint8_t input_data, uint8_t a, uint8_t b)
{
	if ((input_data>=a) && (input_data<=b))
	{
		//nie rob nic	
	}
	else if (input_data<a)
	{
		input_data=a;
	}
	else if (input_data>b)
	{
	input_data=b;
	}
	else
	{
	input_data=0;
	}
	return input_data;
}

//ekran pogody - WORK
void screen_1 (void)
{
	//temp
	OLED_SetCursor(0, 0);
	OLED_DisplayString("Temp.: ");
	temp_1=temp;
	if (temp>=0)	//warunek do poprawnego wyswietlania
	{
	OLED_DisplayString(buf_t);
	}
	else if((temp<0)&&(temp>-10))
	{
	temp=fabs(temp);
	OLED_DisplayChar('-');
	OLED_DisplayNumber(10, temp, 1);
	}
	else
	{		
	temp=fabs(temp);
	OLED_DisplayChar('-');
	OLED_DisplayNumber(10, temp, 2);
	}
	OLED_DisplayString(" ~C ");
	temp=temp_1;
	
	//cisnienie
	OLED_SetCursor(1, 0);
	OLED_DisplayString("Cisnienie: ");
	if ( press>=1000)	//warunek do poprawnego wyswietlania
	{
	OLED_DisplayNumber(10, press , 4);	
	}
	else
	{
	OLED_DisplayNumber(10, press , 3);		
	}	
	OLED_DisplayString(" hPa ");
	
	//wilgotnosc
	OLED_SetCursor(2, 0);
	OLED_DisplayString("Wilgotnosc: ");
	OLED_DisplayString(buf_h);	
	OLED_DisplayString(" % ");
	
	//slonce
	OLED_SetCursor(3, 0);
	OLED_DisplayString("Slonce: ");
	OLED_DisplayString(buf_s);	
	OLED_DisplayString(" % ");
			
}

//ekran zasilania - WORK
void screen_2 (void)
{
	//Nadajnik
	OLED_SetCursor(0, 0);
	OLED_DisplayString("Bat.nadajnika: ");
	OLED_DisplayString(buf_batt_tx);
	OLED_DisplayString(" % ");
	
	//komunikat ladowania nadajnika
	if(tx_batt_mode == 1){
	OLED_SetCursor(1, 0);
	OLED_DisplayString("Naladuj nadajnik ");
	}
	else
	{
	OLED_DisplayString("                  ");
	}	

	//Odbiornik
	OLED_SetCursor(2, 0);
	OLED_DisplayString("Bat.odbiornika: ");
	OLED_DisplayString(buf_batt_rx);
	OLED_DisplayString(" % ");
	
	//komunikat ladowania odbiornika
	if(rx_batt_mode == 1)
	{
	OLED_SetCursor(3, 0);
	OLED_DisplayString("Naladuj odbiornik ");
	}
	else
	{
	OLED_DisplayString("                  ");
	}
	
}


//inicjalizacja ADC - WORK
void adc_init( void ) {
	//ustawienie wejœæ
	DDRC &= ~( 1 << V_BATT );

	ADCSRA = ( 1 << ADEN ) //ADEN=1 w³¹czenie przetwornika ADC
	| ( 1 << ADPS0 ) // ustawienie preskalera na 128
	| ( 1 << ADPS1 )
	| ( 1 << ADPS2 );

	ADMUX  = ( 1 << REFS1 ) | ( 1 << REFS0 ) | ( 1 << ADLAR ); // REFS1:0: wybór napiêcia odniesienia ADC na wewnêtrzne Ÿród³o 2,56V

}

//odczyt ADC - WORK
uint8_t adc_read( void ) {

	volatile uint8_t adc_out = 0;     //zmienna wyjsciowa

	//konwersja

		ADCSRA |= ( 1 << ADSC ); //ADSC: uruchomienie pojedynczej konwersji
		while ( ADCSRA & ( 1 << ADSC ) ); //czeka na zakoñczenie konwersji
		{
			adc_out = ADCH;
		}

		_delay_ms(10);

	return adc_out;
}

//odbior danych - WORK
void recive_and_copy_data_3 (void)
{	
	RFM69_RX_EVENT( send );	//odbior
	 
	if(send[5] == confirm)
	{	
	//odczyt danych pogody
	temp = send[0];
	temp = temp-temp_offset;	
	press = send[1];
	press = press+press_offset;	
	hum = send[2];
			
	//zmiana przeslanych danych pogody na tekst
	itoa(temp, buf_t, 10);
	itoa(press, buf_p, 10);
	itoa(hum, buf_h, 10);
	
	//odczyt adc z nadajnika
	batt_u_tx = send[3];

	soll_u = send[4];		
		
	}
	else
	{
		//nie rob nic
	}
	
	
}
	
//aktywacja uspienia - WORK
void sleep_active( void ) {
   
    set_sleep_mode( SLEEP_MODE_PWR_SAVE );
    sleep_enable();
}


//petla timer 2 - WORK
void int_timer_2_1( void ) {
	
	cli();
	ASSR |= ( 1 << AS2 );   //podlaczenie zewnetrznego oscylatora
	TIMSK |= ( 1 << OCIE2 );                  // tryb porownania
	TCCR2 |= ( 1 << WGM21 ) | ( 1 << CS22 ) | ( 1 << CS21 ) | ( 1 << CS20 ); // tryb CTC, preskaler 1024
	OCR2   = 31;         // przerwanie co 1 s przy zalozeniu ze Fext = 32,768 kHz
	
	sei();
}

//odczyt adc - zmienic
void adc_preparation (void)
{
	volatile float voltage_data = 0.0;
	
	//adc0 batt tx
	voltage_data = (((batt_u_tx * 2.68) / 256) * U_coff) + 0.7;
	batt_u_tx = map (voltage_data, U_min_batt, U_max_batt , 0 , 100 );
	batt_u_tx = constrain(batt_u_tx, 0, 100);
	itoa( batt_u_tx, buf_batt_tx, 10 );	
	
	if(batt_u_tx <= U_batt_low )
	{
	tx_batt_mode = 1;	
	}
	else
	{
	tx_batt_mode = 0;	
	}	
	
	//adc1 soll u - WORK
	voltage_data = ((soll_u * 2.56) / 256) ;	 
	soll_u = map (voltage_data, 0, U_max_sol, 0, 100);
	soll_u = constrain(soll_u, 0, 100);
	itoa( soll_u, buf_s, 10 );
	
	//adc baterii odbiornika - WORK	
	batt_u_rx = adc_read();
	voltage_data = (((batt_u_rx * 2.56) / 256) * U_coff) + 0.3 ;
	batt_u_rx = map (voltage_data, U_min_batt , U_max_batt  , 0 , 100 );
	batt_u_rx = constrain(batt_u_rx, 0, 100);
	itoa( batt_u_rx, buf_batt_rx, 10 );
	
	if(batt_u_rx <= U_batt_low )
	{
		rx_batt_mode = 1;
	}
	else
	{
		rx_batt_mode = 0;
	}
	
}

//obsluga przerwania systemu od przycisku - WORK
void e_int_init (void)
{
	
	PORTD &= ~( 1 << SW );	//port wejscia
	PORTD |= (1 << SW);		//podciagniety wysoko
	
	cli();
	MCUCR &= ~((1 << ISC11) | (0 << ISC10));	//int1 aktywne przez zbocze opadajace
	GICR |= (1 << INT1);	//wlacz przerwanie od int1
	sei();
	
}


//petla glowna
int main( void ) {

	//obs³uga diod led  - jako wyjœcia
	DDRB |= ( 1 << LED_1 );

	//inicjalizacja rozpoczeta
	PORTB |= ( 1 << LED_1 );
	_delay_ms( 100 );

	//modu³ radiowy
	//rfm69_init( FREQ( 433 ), "T1D1" , ADDR_FILTER_BROADCAST, 0, 255 ); //inicjalizacja modulu czestotliwosc, nazwa sieci, filtr, adres modulu, adres broadcastu
	rfm69_init( FREQ( 433 ), 0, 0, 0, 0 );

	//ekran
	OLED_Init();  
	OLED_Clear();
		
	//ADC
	adc_init();
	
	//obsluga przerwan
	e_int_init();	//od przycisku
	int_timer_2_1(); //od timera
	
	//czyszczenie tablic
	for (int i=0; i<=(table_size-1); i++)
	{
		send[i] = 0;
	}
	
	mode_sw = 0;	//wyczyszczenie trybu przelacznika
	
	adc_preparation();	//uruchom pomiar adc
	
	//inicjalizacja zakonczona
	PORTB &= ~( 1 << LED_1 );
	_delay_ms( 100 );
	
	//ekran powitalny
	OLED_SetCursor(0,0);
	_delay_ms(1500);
	OLED_Clear();

	while ( 1 ) {
				
	//PETLA GLOWNA FINAL	
			
	//OBSLUGA ODBIORU	
	recive_and_copy_data_3();
			
	if (send[5]==confirm)
	{
		PORTB |= ( 1 << LED_1 );
		adc_preparation();
		send[5] = 0;
		rx_status = 1;
		rx_status_start = 1;
		mode_sw =1;
		OLED_Clear();
		sec_timer_2 = 0;
		_delay_ms(10);
		PORTB &= ~( 1 << LED_1 );
	}
	
	//OBSLUGA KOMUNIKATU POCZATKOWEGO	
	if (rx_status_start==0)
	{
	OLED_SetCursor(0, 0);
	OLED_DisplayString("OCZEKIWANIE NA SYGNAL");	
	}			
		
	//OBS£UGA ALARMU BRAKU NADAWANIA - WORK
	if(sec_timer_2>=sec_alarm)
	{
		OLED_Clear();
		OLED_SetCursor(0, 0);
		OLED_DisplayString("URUCHOM NADAJNIK");
		OLED_SetCursor(1, 0);
		OLED_DisplayString("BRAK SYGNALU");
		OLED_SetCursor(2, 0);
		OLED_DisplayString("NASTAPI RESET");
		_delay_ms(5000);
		OLED_Clear();
		wdt_enable( WDTO_15MS );
		while(1)
		{
		}				
	}
			
	//OBSLUGA EKRANU - WORK
	if(mode_sw == 1)
	{	
	screen_1();	
	}
	else if (mode_sw == 2)
	{
	screen_2();	
	}
	else if (mode_sw == 3)
	{
		OLED_Clear();
		mode_sw = 1;
	}	
	
	//RESET
	if(sec_timer_2>= sec_reset)
	{   
	    wdt_enable( WDTO_15MS );
		while(1)
		{	
		sec_timer_2 = 0;
		}	
	}
	}
}

//przerwanie od przycisku - WORK
ISR(INT1_vect) 
{
		
		PORTB |= ( 1 << LED_1 );
		OLED_Clear();		
		//eliminacja zdrgan styku
	    _delay_ms( 150 );
	    if ( !( PIND & SW ) ) {
		    mode_sw++;		 
	    }
	    _delay_ms( 100 );
		PORTB &= ~( 1 << LED_1 );
}

//przerwanie - WORK
ISR( TIMER2_COMP_vect ) {	
	sec_timer_2++;
}
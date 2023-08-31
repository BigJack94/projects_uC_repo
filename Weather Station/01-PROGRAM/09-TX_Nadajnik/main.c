#define F_CPU 4000000UL

/*
GOTOWE

*/

//blblioteki----------------------------------------------------------------------------------------------------------------
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#include "rfm69.h"
#include "bme280.h"

//porty----------------------------------------------------------------------------------------------------------------
#define LED_1 PB0
#define V_BATT PC0  //ADC0
#define V_SOLL PC1  //ADC1     

//definicje i stale----------------------------------------------------------------------------------------------------------------
#define table_send 6
#define temp_offset 50
#define press_offset 850
#define sec_int 20 // czas uspienia w sekundach 
#define sec_reset 86400 //czas resetu w sekundach - obecnie doba

//tablica znaków modulu radiowego----------------------------------------------------------------------------------------------------------------
uint8_t out_buf[ table_send ];

//zmienne przerwan licznika----------------------------------------------------------------------------------------------------------------
uint8_t volatile timer_int = ( sec_int - 1 );    //licznik sekund
uint64_t volatile timer_wd = 0;    //licznik sekund

//mapowanie danych - WORK
uint16_t map( uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max ) {
    return ( x - in_min ) * ( out_max - out_min ) / ( in_max - in_min ) + out_min;
}

//inicjalizacja ADC - WORK
void adc_init( void ) {
    //ustawienie wejœæ
    DDRC &= ~( 1 << V_BATT );
    DDRC &= ~( 1 << V_SOLL );

    ADCSRA = ( 1 << ADEN ) //ADEN=1 w³¹czenie przetwornika ADC)
             | ( 1 << ADPS0 ) // ustawienie preskalera na 128
             | ( 1 << ADPS1 )
             | ( 1 << ADPS2 );

    ADMUX  = ( 1 << REFS1 ) | ( 1 << REFS0 ) | ( 1 << ADLAR ); // REFS1:0: wybór napiêcia odniesienia ADC na wewnêtrzne Ÿród³o 2,56V, rozdzielczosc 255

}

//odczyt ADC0 - WORK
volatile uint8_t adc_read_0( void ) {

    //konwersja
    ADCSRA |= ( 1 << ADSC ); //ADSC: uruchomienie pojedynczej konwersji
    while ( ADCSRA & ( 1 << ADSC ) );  //czeka na zakoñczenie konwersji

    _delay_ms( 15 );

    return ADCH;
}

//odczyt ADC1 - WORK
volatile uint8_t adc_read_1( void ) {

    ADMUX |= _BV( MUX0 );

    //konwersja
    ADCSRA |= ( 1 << ADSC ); //ADSC: uruchomienie pojedynczej konwersji
    while ( ADCSRA & ( 1 << ADSC ) );  //czeka na zakoñczenie konwersji

    _delay_ms( 15 );

    ADMUX &= ~_BV( MUX0 );

    return ADCH;
}

//odczyt - WORK
void read_and_transmit_3( void ) {

    //czyszczenie tablicy
    for ( int i = 0; i <= ( table_send - 1 ); i++ ) {
        out_buf[i] = 0;
    }

    volatile float value = 0.0; //zmienna wartoœci    
    volatile uint8_t value_aux = 0;
    
    PORTB |= ( 1 << LED_1 );

    //temperatura
    value = bme280_readTemperature( 0 ) + temp_offset; //deg C
    value_aux =  (uint8_t) value;
    out_buf[0] =  value_aux;

    //cisnienie
    value =  bme280_readPressure( 0 ) / 100 ; //dPa
    out_buf[1] = value - press_offset;

    //wilgotnosc
    value = bme280_readHumidity( 0 );  //%
    out_buf[2] = value;

    //adc0
    out_buf[3] = adc_read_0();

    //adc1
    out_buf[4] = adc_read_1();

    out_buf[5] = 94;      //byla 5 pozycja w tableki i zmienilem na 9

    //wysylanie
    rfm69_transmit( 0, out_buf, table_send );

    PORTB &= ~( 1 << LED_1 );

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

//aktywacja uspienia - WORK
void sleep_active( void ) {

    rfm69_standby(); //wprowadzenie w tryb standby
    rfm69_sleep(); //uspienie modulu

    set_sleep_mode( SLEEP_MODE_PWR_SAVE );
    sleep_enable();
}

//petla glowna - WORK
int main( void ) {

    //INICJALIZACJE

    //RADIO
    //rfm69_init( FREQ( 433 ), "T1D1" , ADDR_FILTER_BROADCAST, 0, 255 ); //inicjalizacja modulu czestotliwosc, nazwa sieci, filtr, adres modulu, adres broadcastu
    rfm69_init( FREQ( 433 ), 0, 0, 0, 0 );

    //CZUJNIK
    bme280_init( 0 );

    //ADC
    adc_init();

    //Timer2
    int_timer_2_1();

    //LED
    DDRB |= ( 1 << LED_1 );

    //obs³uga diod led
    PORTB |= ( 1 << LED_1 );
    _delay_ms( 200 );
    PORTB &= ~( 1 << LED_1 );
    _delay_ms( 200 );


    for ( int i = 0; i <= ( table_send - 1 ); i++ ) {
        out_buf[i] = 0;
    }

    while ( 1 ) {

        //wysylanie lub uspienie
        if ( timer_int >= sec_int ) {
            read_and_transmit_3();
            timer_int = 0;
        } else {
            sleep_active();
        }

        //auto reset
        if ( timer_wd >= sec_reset ) {
            wdt_enable( WDTO_15MS );
            while ( 1 ) {
                timer_wd = 0;
            }
        }
    }
}

//przerwanie - WORK
ISR( TIMER2_COMP_vect ) {
    timer_int++;
    timer_wd++;
}
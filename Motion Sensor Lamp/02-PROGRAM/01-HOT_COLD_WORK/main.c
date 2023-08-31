#define F_CPU 1200000UL

//BIBLIOTEKI
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>


#include <stdio.h>
#include <stdlib.h>
#include <time.h>

//PORTY
#define OUT_LED_H      PB0
#define OUT_LED_C      PB1
#define AIN_FOT        PB3
#define IN_SW_1        PB2
#define IN_SW_3        PB4

//DEFINICJE WARTOŒCI
// moce dzia³ania LEDów
#define led_h_power_max 134      
#define led_c_power_max 207

#define led_h_power_min 67
#define led_c_power_min 104

//zakresy dzia³ania fotorezystora           
/*
3,3V - potencjometr rozkrecony na 10k, fotorezystor zas³oniêty
0,28V - potencjometr rozkrecony na 10k, fotorezystor naswietlony 
3,91 V - zasilanie
*/
    
    //sprawdzic inne ustawienia
#define night_0 20 //granica wlaczenia       
#define night_100 220 //granica wylaczenia         


//ZMIENNE GLOBALNE
volatile uint8_t MODE = 0;
volatile uint8_t FOT = 0;
volatile uint8_t hot_led;
volatile uint8_t cold_led;

//INICJALIZACJA  ADC
void adc_setup( void ) {     //ok

    DDRB &= ~_BV( AIN_FOT );

    // PB3
    ADMUX |= ( 1 << MUX1 ) | ( 1 << MUX0 );
    ADMUX |= ( 1 << ADLAR );

    // F_CPU/128
    ADCSRA |= ( 1 << ADPS2 ) | ( 1 << ADPS1 ) | ( 1 << ADPS0 ) | ( 1 << ADEN );

}

//ODCZYT ADC            //ok
uint8_t adc_read( void ) {
    // Start the conversion
    ADCSRA |= ( 1 << ADSC );

    // Wait for it to finish
    while ( ADCSRA & ( 1 << ADSC ) );

    return ADCH;
}

//STEROWANIE OPÓNIENIEM W us
void delay_us( int us ) {
    for ( int i = 0; i < us; i++ ) {
        _delay_us( 1 );
    }
}

//STEROWANIE OPÓNIENIEM W ms
void delay_ms( int ms ) {
    for ( int i = 0; i < ms; i++ ) {
        _delay_ms( 1 );
    }
}

//INICJALIZACJA PWM  
void pwm_setup( void ) {
    //WYJŒCIE PWM
    DDRB |= ( 1 << OUT_LED_H );
    DDRB |= ( 1 << OUT_LED_C );

    // Set Timer 0 prescaler to clock/8.
    //preskaler 8
    TCCR0B |= ( 1 << CS01 );

    // Set to 'Normal PWM' mode
    TCCR0A |= ( 1 << WGM00 ) | ( 1 << WGM02 );

    // Clear OC0B output on compare match, upwards counting.
    TCCR0A |= ( 1 << COM0A1 );
    TCCR0A |= ( 1 << COM0B1 );
}

//ZAPIS PWM I URUCHOMIENIE
void pwm_write_led_h( int val ) {
    OCR0A = val;
}

void pwm_write_led_c( int val ) {
    OCR0B = val;
}

//petla sprawdzania polozenia przelacznika   ok
uint8_t sw_mode( void ) {

    uint8_t state_sw = 0;

    if (( ( PINB & _BV( IN_SW_1 ) ) ) && ( ( PINB & _BV( IN_SW_3 ) ) ) ) {     //przelacznik na srodku
        state_sw = 0;
    } else if ((( PINB & _BV( IN_SW_1 ) ) ) && ( !( PINB & _BV( IN_SW_3 ) ) ) ) {
        state_sw = 1;
    } else if (( !( PINB & _BV( IN_SW_1 ) ) ) && (( PINB & _BV( IN_SW_3 ) ) ) ) {
        state_sw = 2;
    } else {
        state_sw = 3;
    }

    return   state_sw;
}

//funkcja mapowania
uint8_t map( uint8_t x, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max ) {
    return ((( x - in_min ) * ( out_max - out_min ) ) / (( in_max - in_min ) + out_min ) );
}  

//funkcja constrain
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
          
//tryb 1     tryb inteligentny / doœwietlaj¹cy
void mode_1( void ) { 
  
//zakresy dzialania fotokomorki  
const uint8_t range_1 = 30;
const uint8_t range_2 = 60;    
const uint8_t range_3 = 90;    
                  

    if (( FOT >= range_1 ) && ( FOT < range_2 ) ) {       //rosnie

        hot_led = 0;
        cold_led = map( FOT, range_1, range_2, 20, 100 );

    } else if (( FOT >= range_2 ) && ( FOT < range_3 ) ) {   //opada

        hot_led = map( FOT, range_3, range_2, 20 , 90 );        
        cold_led = 0;

    } else if ( FOT >= range_3 ) {    //staly                         
        hot_led = 20;
        cold_led = 0;
                                                      
    } else {             //wylaczony
                                                                       
        hot_led = 0;
        cold_led = 0;                                                                   
    }
}

//tryb 2  tryb sta³y - praca na 90%
void mode_2( void ) {
    hot_led = 95;                                                         
    cold_led = 95;
}

//tryb 3   tryb rosn¹cy
void mode_3( void ) {
    
//zakresy dzialania fotokomorki  
const uint8_t range_1 = 15;
const uint8_t range_2 = 60;                                                              

    if (( FOT >= range_1 ) && ( FOT < range_2 ) ) {       //rosnie

        hot_led = 0;                                                                             
        cold_led = map( FOT, range_1, range_2, 0, 100 );

    }  else if ( FOT >= range_2 ) {    //staly
        hot_led = 100;
        cold_led = 0;

    } else {             //wylaczony

        hot_led = 0;
        cold_led = 0;
    }                                                            
}


//PETLA GLOWNA
int main( void ) {

    //inicjalizacja
    adc_setup();     //inicjalizacja ADC
    pwm_setup();    //inicjalizacja PWM

    //DEKLARACJA WEJŒÆ
    DDRB &= ~_BV( IN_SW_1 );
    DDRB &= ~_BV( IN_SW_3 );
    //podcigniecie wejsc pod stan wysoki
    PORTB |= ( 1 << IN_SW_1 );
    PORTB |= ( 1 << IN_SW_3 );


    while ( 1 ) {

        FOT = adc_read(); //ODCZYT WARTOŒCI FOTOREZYSTORA
        FOT = map( FOT, night_0, night_100, 0, 100 ); //przeliczenie wartosci fotorezystora na procenty  
        FOT = constrain( FOT, 0, 100 ); 
        
        MODE  = sw_mode(); //stan przelacznika
        //MODE = 0;

        //przelaczanie stanów
        switch ( MODE ) {

        case 0:
           mode_1();
            break;

        case 1:
            mode_2();  
            break;

        case 2:   
            mode_3();
            break;                                          

        default:
            MODE = 0;
            hot_led = 0;
            cold_led = 0; 
            break;
        }

        //URUCHOM LEDY
        //skalowanie z procentów na PWM   
       // hot_led = constrain(hot_led 0, 100 ); 
        hot_led = map( hot_led, 0, 100, led_h_power_min , led_h_power_max ); 
       // cold_led = constrain( cold_led, 0, 100 ); 
        cold_led = map( cold_led, 0, 100, led_h_power_min  , led_c_power_max );  

                         
        //zapis PWM
        pwm_write_led_h( hot_led );
        pwm_write_led_c( cold_led );
    }
}
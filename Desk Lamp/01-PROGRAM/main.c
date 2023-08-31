#define F_CPU 9600000UL

//BIBLIOTEKI
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>  

             
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
                                 
//PORTY
#define T1        PB0
#define T2        PB1
#define T3        PB2
#define SW        PB3    //pin z przerwaniem            
#define POT       PB4    //pin z potencjometrem  

//ZMIENNE GLOBALNE
volatile uint8_t MODE = 0;
volatile uint8_t POT_VALUE = 0;

//INICJALIZACJA  POTENCJOMETRU
void adc_setup_P( void ) {

    DDRB &= ~_BV( POT );
    // Set the ADC input to PB4/ADC2

    ADMUX |= ( 1 << MUX1 );
    ADMUX |= ( 1 << ADLAR );

    // Set the prescaler to clock/128 & enable ADC
    // At 9.6 MHz this is 75 kHz.
    // See ATtiny13 datasheet, Table 14.4.
    ADCSRA |= ( 1 << ADPS1 ) | ( 1 << ADPS0 ) | ( 1 << ADEN );

}

//ODCZYT ADC
int adc_read_P( void ) {
    // Start the conversion
    ADCSRA |= ( 1 << ADSC );

    // Wait for it to finish
    while ( ADCSRA & ( 1 << ADSC ) );

    return ADCH;
}

//PRZERWANIE PCINT3 PB3                                  
ISR( PCINT0_vect ) {

    if ( !( PINB &_BV( SW ) ) )  {
        MODE++;
        _delay_ms( 200 ); 
        
    }

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

//PÊTLA SOFTWARE PWM
void spwm( uint16_t time_1, uint16_t number ) {
    
    uint16_t period = 256;         //wartosc maksymalna
    uint8_t limes = 2;
               
    //WY£¥CZ JEŒLI NISKA WARTOŒÆ ADC
    if ( time_1 < limes ) {
        PORTB &= ~( _BV( number ) );
    }                                           
    //STEROWANIE WYPE£NIENIEM PRZEBIEGU
    else {                                                      
        PORTB |= _BV( number );
        delay_us( time_1 );
        PORTB &= ~( _BV( number ) );
        delay_us(period-time_1 );
    }                   
}

//PÊTLA LOOP LED
void led_3w( int speed ) {
     
int i;
speed=speed/22;

//led 1                            
for(i=1; i<=254; i++)
{          
   spwm( i, T1 );
    delay_ms( speed ); 
} 

for(i=254; i>= 1; i--)
{          
   spwm( i, T1 );
    delay_ms( speed );                                    
} 

//led 2
for(i=1; i<=254; i++)
{          
   spwm( i, T2 );
    delay_ms( speed ); 
}

for(i=254; i>= 1; i--)
{          
   spwm( i, T2 );
    delay_ms( speed );                   
}

//led 3
for(i=1; i<=254; i++)
{          
   spwm( i, T3 );
    delay_ms( speed ); 
} 

for(i=254; i>= 1; i--)
{          
   spwm( i, T3 );
    delay_ms( speed );                   
}

 
                               
}    

//GENERATOR LICZB LOSOWYCH
int random_number (int min, int max)
{
    int tmp;
    if (max>=min)                      
        max-= min;
    else
    {
        tmp= min - max;
        min= max;
        max= tmp;
    }
    return max ? (rand() % max + min) : min;
}
  
  
//EFEKT SWIECY
void candel (void)
{         
int i=random_number (120,250);

            spwm( i, T1 );
            spwm( i, T2 );
            spwm(i, T3 );     
}                                                         
  
//WLACZENIE PRZERWANIA
void int_init( void ) {
    
    
    DDRB &= ~_BV( SW );                       
    cli();                                 
    //OBS£UGA PRZERWAÑ PCINT3 PB3
    PCMSK |= _BV( PCINT3 );
    GIMSK |= _BV( PCIE );
    //DEKLARACJA WEJŒÆ WYBORU W£¥CZENIA/WY£¥CZENIA
    //PORTB &= ~_BV( SW );
    sei();
}

//PETLA GLOWNA
int main( void ) {                
                                                          
    adc_setup_P();                //inicjalizacja ADC Potencjometru

    //DEKLARACJA WYJŒÆ
    DDRB |= _BV( T1 );
    DDRB |= _BV( T2 );
    DDRB |= _BV( T3 ); 

    int_init();    //w³¹czenie obs³ugi przerwania PCINT3

    while ( 1 ) {

        POT_VALUE = adc_read_P();       //ODCZYT WARTOŒCI POTENCJOMETRU

        //PRZE£¥CZANIE USTAWIEÑ OBS£UGI CZÊSTOTLIWOŒCI
        switch ( MODE ) {

        case 0:
           
            spwm( POT_VALUE, T1 );
            break;

        case 1:
           
            spwm( POT_VALUE, T2 );
            break;

        case 2:
            
            spwm( POT_VALUE, T3 );
            break;

        case 3:
           spwm( POT_VALUE, T1 );
            spwm( POT_VALUE, T2 );
            spwm( POT_VALUE, T3 );                   
            break;                                   

        case 4:
           led_3w ( POT_VALUE);   
           break;    
        
        case 5:
            PORTB |= _BV( T1 );
            PORTB |= _BV( T2 );
            PORTB |= _BV( T3 );
            break; 
                                           

        default:
            MODE = 0;
            PORTB &= ~ _BV( T1 );
            PORTB &= ~ _BV( T2 );
            PORTB &= ~ _BV( T3 ); 
            break;
        }
    }
}
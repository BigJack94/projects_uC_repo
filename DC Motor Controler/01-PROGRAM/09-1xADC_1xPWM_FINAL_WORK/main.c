//PREDKOSC PROCESORA
#define F_CPU 9600000UL

//BIBLIOTEKI WEJ�CIOWE
#include <util/delay.h>
#include <avr/io.h>

//OBS�UGA PORT�W WEJSCIOWYCH

#define P    PB4    //potencjometr  
#define L1   PB2    //led
#define L2   PB3    //led                        
#define S    PB1    //przelacznik
#define O    PB0    //silnik 


//WARTO�CI SKALOWANIA
#define pwm_min  1         //warto�ci min max pwm
#define pwm_max  255
#define pot_min  70        //warto�ci min max odczytywane prze adc, napi�cia na potencjometrze
#define pot_max  235


//ZMIENNE OBSLUGI ADC

volatile uint16_t PWM = 0;                 //zmienna obs�ugi wyjscia
volatile uint16_t POT = 0;                 //zmienna obslugi potencjometru
volatile uint16_t SOFT = 1;              //zmienna obs�ugi soft start
volatile uint16_t MODE = 0;              //zmienna wykrywania stanu zworki reguluj�cej skalowanie obrot�w

//ODCZYT Z POTENCJOMETRU
void adc_P( void ) {

    // Start the conversion
    ADCSRA |= ( 1 << ADSC );

    // Wait for it to finish
    while ( ADCSRA & ( 1 << ADSC ) );

    POT = ADCH;                                                

}

//INICJALIZACJA ADC
void adc_setup( void ) {

    //ZDEFINIOWANIE PORTU JAKO WEJ�CIA
    DDRB &= ~_BV( P );

    //wyb�r kana�u
    ADMUX |= _BV( MUX1 );

    //wyr�wnanie do prawej
    ADMUX |= ( 1 << ADLAR );

    // podzia� zegara przez 128
    ADCSRA |= ( 1 << ADPS1 ) | ( 1 << ADPS0 ) | ( 1 << ADEN );

    //w��czenie przetwornika
    ADCSRA |= ( 1 << ADEN );

}

//INICJALIZACJA PWM
void pwm_setup( void ) {
    //WYJ�CIE PWM
    DDRB |= ( 1 << O );

    // Set Timer 0 prescaler to clock/8.
    //preskaler 8, cz�stotliwo�� dzia�ania 1,2 MHz/510
    TCCR0B |= ( 1 << CS01 );

    // Set to 'Normal PWM' mode
    TCCR0A |= ( 1 << WGM00 ) | ( 1 << WGM02 );

    // Clear OC0B output on compare match, upwards counting.
    TCCR0A |= ( 1 << COM0A1 );

}

//ZAPIS PWM I URUCHOMIENIE
void pwm_write( int val ) {
    OCR0A = val;
}


//STEROWANIE OPӏNIENIEM W us
void delay_us( int us ) {
    for ( int i = 0; i < us; i++ ) {
        _delay_us( 1 );
    }
}


//normalizacja wyniku
int norm( int value, int min, int max, int out_min, int out_max ) {
    if ( value >= max )
        value = out_max;
    else if ( value <= min )
        value = out_min;
    else
        value = value;

    return value;
}

//funkcja map do przeskalowywania - dzia�a
long map( long x, long in_min, long in_max, long out_min, long out_max ) {
    return ( x - in_min ) * ( out_max - out_min ) / ( in_max - in_min ) + out_min;
}


//PETLA GLOWNA
int main( void ) {

    _delay_ms( 1000 );                  //op�nienie wst�pne

    //inicjalizacja
    pwm_setup();                         //wyj�cia PWM
    adc_setup();                         //wej�cia ADC

    //obs�uga diod led  - jako wyj�cia
    DDRB |=  _BV( L1 );
    DDRB |=  _BV( L2 );

    //obs�uga diod led  - stan wysoki - led wy��czone
    PORTB |= _BV( L1 );
    PORTB |= _BV( L2 );

    //obs�uga prze��cznika  - wej�cie i podciagni�cie do stanu wysokiego
    DDRB &= ~_BV( S );
    PORTB |= _BV( S );

    while ( 1 ) {

        //odczyt po�o�enia zworki
        if ( !( PINB &_BV( S ) ) ) {
            MODE = 2;         //zworka zwarta
        }

        else {            //zworka rozwarta
            MODE = 1;
        }

        //odczyt z adc
        adc_P();


        //je�li prze��cznik trybu w pozycji �rodkowej lub sterowania termo + termistor od��czony == wy��cz silnik  i wy��cz LED trybu
        if ( POT <= 10 ) {
            PORTB |= _BV( L2 );
            PORTB |= _BV( L1 );
            PWM = 0;
        }

        //pozycja pracy lub sterowanie termo + termistor pod��czony == praca silnika
        else {
            //normalizacja wyniku do warto�ci wyst�puj�cych na potencjometrze
            POT = norm( POT, pot_min, pot_max, pot_min, pot_max );


            //wyb�r trybu pracy

            if ( MODE == 2 ) { //temperatura wzrasta obroty spadaj�
                PORTB |= _BV( L1 );                        //niebieska dioda on, zielona off
                PORTB &= ~_BV( L2 );
                //przeskalowanie wyniku
                PWM = map( POT, pot_min, pot_max , pwm_max , pwm_min );       //odwr�� skal�  i przeskaluj do warto�ci PWM
            }

            else if ( MODE == 1 ) {   //temperatura wzrasta obroty rosn�
                PORTB |= _BV( L2 );          //zielona dioda on, niebieska off
                PORTB &= ~_BV( L1 );
                //przeskalowanie wyniku
                PWM = map( POT, pot_min, pot_max , pwm_min , pwm_max );   //przeskauj do warto�ci PWM
            }

            else {              //b��d - nigdy nie osi�gany
                PORTB |= _BV( L1 );
                PORTB |= _BV( L2 );
                PWM = 0;
            }



        }


        //SOFT START
        if ( PWM > SOFT ) {
            for ( SOFT; SOFT <= PWM; SOFT++ ) {
                pwm_write( SOFT );
                _delay_ms( 1 );

            }
        } else if ( SOFT > PWM ) {
            for ( PWM; SOFT <= PWM; PWM-- ) {
                pwm_write( SOFT );
                _delay_ms( 1 );

            }
        } else {
            pwm_write( PWM );

        }

        SOFT = PWM;



    }
}
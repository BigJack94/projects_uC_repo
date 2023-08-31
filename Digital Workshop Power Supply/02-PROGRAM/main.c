//FCPU
#define F_CPU 8000000UL

//LIB IN
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include <stdio.h>
#include <time.h>

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "lcdpcf8574/lcdpcf8574.h"
#include "pcf8574/pcf8574.h"

//IO

//LED
#define START_1 PC2
#define READY_1 PC3
#define ALARM_1 PC4
#define START_2 PC5
#define READY_2 PC6
#define ALARM_2 PC7
//SWITCHS  
#define P_C_1 PD2
#define P_C_2 PD3
#define CH    PB2
//ADC
#define POT      PA0
#define U_BATT   PA1            
#define U_1      PA2
#define U_2      PA3
#define I_1      PA5    //PA4 nie dzia�a  
#define I_2      PA5
//LCD 2x16
#define LCD_SCL PC0
#define LCD_SDA PC1
//RELAYS
#define R_1 PD4
#define R_2 PD5
//OTHERS
#define CHARGE PD6


//VAR <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//PRZYCISK 3
uint8_t mode_lcd = 0;                //zmienna ekranu - ekran 1, 2, 3
uint8_t old_mode_lcd = 0;            //zmienna usuwania starego ekranu
uint8_t mode_lcd_light = 0;          //zmienna ekranu - ekran pod�wietlenie
uint8_t mode_pot = 0;                //zmienna obs�ugi potencjometru zmiany pr�du

//PRZYCISK 1 I 2
uint8_t p1_mode = 0;        //przycisk P1 - kasuj alarm, w��cz, wy��cz
uint8_t p2_mode = 0;        //przycisk P2 - kasuj alarm, w��cz, wy��cz
uint8_t ch_ch = 3;          //przycisk P1 i P2 - wyb�r kana�u

//ADC
uint16_t u_batt = 0;        //odczyt napi�cia bateri
uint16_t pot_i = 0;         //odczyt napi�cia potencjometru
uint16_t u_1 = 0;           //odczyt napi�cia przetwornicy kana�u 1
uint16_t u_2 = 0;           //odczyt napi�cia przetwornicy kana�u 2
uint16_t i_1 = 0;           //odczyt napi�cia bocznika 1
uint16_t i_2 = 0;           //odczyt napi�cia bocznika 2

//LED
uint8_t led_1_state = 0;    //status LED 1
uint8_t led_2_state = 0;    //status LED 2

//PRZEKA�NIKI
uint8_t r_1 = 0;            //stan przeka�nik 1
uint8_t r_2 = 0;            //stan przeka�nik 2

//POZOSTA�E
uint8_t batt_charge = 0;  //zmiena statusu ��dowania baterii
char conv[4];         //tablica konwersji wyswietlacza

//WARTO�CI PR�D�W I NAPI��
double i_1_value = 0;         //pob�r pr�du - kana� 1
double i_2_value = 0;         //pob�r pr�du - kana� 2

double i_max_1_value = 0.5;         //pob�r pr�du - kana� 1
double i_max_2_value = 0.5;         //pob�r pr�du - kana� 2

double u_1_value = 0;           //napi�cie wy�wietlane - kana� 1
double u_2_value = 0;           //napi�cie wy�wietlane - kana� 2

double p_1_value = 0;           //moc wy�wietlane - kana� 1
double p_2_value = 0;           //moc wy�wietlane - kana� 2

double r_1_value = 0;           //moc wy�wietlane - kana� 1
double r_2_value = 0;           //moc wy�wietlane - kana� 2

double u_batt_value = 0;        //napi�cie wy�wietlane - baterie
double u_pot_value = 0;         //napi�cie wy�wietlane - potencjometr

//WARTO�CI STA�E I ODNIESIENIA
#define U_coff 16.95          //skalowanie warto�ci napi�cia                   
#define U_delta 0           //wspo�czynnik korekcyjny napi�cia                                                  
#define I_coff 1              //skalowanie napi�cia pomiaru pr�du                            
#define Batt_coff 2.0239       //skalowanie napi�cia baterii
#define Pot_coff 2.08         //skalowanie napi�cia potencjometru                          
#define U_ref 2.6             //napi�cie odniesienia                                                  

#define Ubatt_max 4.2        //poziomy napi�� w baterii - max  
#define Ubatt_min 3.9        //poziomy napi�� w baterii - min 

#define Upot_max 5.0         //poziom napi�cia w potencjometrze - max

//P�TLE OBS�UGI <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//FUNKCJA MAP
double map( double x, double in_min, double in_max, double out_min, double out_max ) {
    return ((( x - in_min ) * ( out_max - out_min ) ) / (( in_max - in_min ) + out_min ) );
}

//URUCHOMIENIE ADC
void ADC_setup( void ) {
    //ustawienie portow jako wej�cia
    DDRA &= ~( 1 << POT );
    DDRA &= ~( 1 << U_BATT );
    DDRA &= ~( 1 << U_1 );
    DDRA &= ~( 1 << U_2 );
    DDRA &= ~( 1 << I_1 );
    DDRA &= ~( 1 << I_2 );

    ADMUX |= _BV( REFS0 );  //napiecie odniesienia jako Wwewn�trze 2.56V
    ADMUX |= _BV( REFS1 );

    ADCSRA = ( 1 << ADEN ) | ( 1 << ADPS2 ) | ( 1 << ADPS1 ) | ( 1 << ADPS0 ); // enable ADC, 128 clk divider
}

//INIT LED
void LED_setup( void ) {
    //ustawienie port�w jako wyj�cia
    DDRC |= ( 1 << START_1 );
    DDRC |= ( 1 << READY_1 );
    DDRC |= ( 1 << ALARM_1 );
    DDRC |= ( 1 << START_2 );
    DDRC |= ( 1 << READY_2 );
    DDRC |= ( 1 << ALARM_2 );
    //ustawienie stanu niskiego wyj��
    PORTC |= ( 1 << START_1 );
    PORTC |= ( 1 << READY_1 );
    PORTC |= ( 1 << ALARM_1 );
    PORTC |= ( 1 << START_2 );
    PORTC |= ( 1 << READY_2 );
    PORTC |= ( 1 << ALARM_2 );
}

//INIT PRZEKA�NIK�W
void RELAY_setup( void ) {
    //ustawienie port�w jako wyj�cia
    DDRD |= ( 1 << R_1 );
    DDRD |= ( 1 << R_2 );
    //ustawienie stanu niskiego wyj��
    PORTD &= ~( 1 << R_1 );
    PORTD &= ~( 1 << R_2 );
}

//USTAWIENIE KANA�U POT PA0
int adc_channel_pot( void ) {

    ADMUX &= ~ _BV( MUX0 );
    ADMUX &= ~ _BV( MUX1 );
    ADMUX &= ~ _BV( MUX2 );
    ADMUX &= ~ _BV( MUX3 );
    ADMUX &= ~ _BV( MUX4 );;

    ADCSRA |= ( 1 << ADSC );    // ADC Start Conversion
    while ( ADCSRA & ( 1 << ADSC ) );

    ADMUX &= ~ _BV( MUX0 );
    ADMUX &= ~ _BV( MUX1 );
    ADMUX &= ~ _BV( MUX2 );
    ADMUX &= ~ _BV( MUX3 );
    ADMUX &= ~ _BV( MUX4 );

    return ADC;
}

//USTAWIENIE KANA�U BATT PA1
int adc_channel_batt( void ) {

    ADMUX |= _BV( MUX0 );

    ADCSRA |= ( 1 << ADSC );    // ADC Start Conversion
    while ( ADCSRA & ( 1 << ADSC ) );

    ADMUX &= ~ _BV( MUX0 );

    return ADC;
}

//USTAWIENIE KANA�U U1 PA2
int adc_channel_u1( void ) {
    ADMUX |= _BV( MUX1 );

    ADCSRA |= ( 1 << ADSC );    // ADC Start Conversion
    while ( ADCSRA & ( 1 << ADSC ) );

    ADMUX &= ~ _BV( MUX1 );

    return ADC;
}

//USTAWIENIE KANA�U U2 PA3
int adc_channel_u2( void ) {
    ADMUX |= _BV( MUX0 );
    ADMUX |= _BV( MUX1 );

    ADCSRA |= ( 1 << ADSC );    // ADC Start Conversion
    while ( ADCSRA & ( 1 << ADSC ) );

    ADMUX &= ~ _BV( MUX0 );
    ADMUX &= ~ _BV( MUX1 );

    return ADC;
}

//USTAWIENIE KANA�U I1 PA6
int adc_channel_i1( void ) {
    ADMUX |= _BV( MUX1 );
    ADMUX |= _BV( MUX2 );

    ADCSRA |= ( 1 << ADSC );    // ADC Start Conversion
    while ( ADCSRA & ( 1 << ADSC ) );

    ADMUX &= ~ _BV( MUX1 );
    ADMUX &= ~ _BV( MUX2 );

    return ADC;
}

//USTAWIENIE KANA�U I2 PA5
int adc_channel_i2( void ) {
    ADMUX |= _BV( MUX0 );
    ADMUX |= _BV( MUX2 );

    ADCSRA |= ( 1 << ADSC );    // ADC Start Conversion
    while ( ADCSRA & ( 1 << ADSC ) );

    ADMUX &= ~ _BV( MUX0 );
    ADMUX &= ~ _BV( MUX2 );

    return ADC;
}



//KONWERSJA WARTO�CI ADC NA WARTO�� NAPI�CIA
double adc_convert_voltage( uint16_t input_value, double coff ) { //warto�� wej�ciowa i warto�� wsp�czynnika obliczeniowego
    double out_value;

    out_value = (( input_value * U_ref ) / 1024 ) * coff;

    return out_value;
}


//INICJALIZACJA PRZERWA� ZEWN�TRZNYCH
void INT_setup( void ) {
    //ustawienie portow jako wej�cia
    DDRD &= ~( 1 << P_C_1 );
    DDRD &= ~( 1 << P_C_2 );
    DDRB &= ~( 1 << CH );

    //zablokowanie przerwa�
    cli();

    //ustawienie stanu wysokiego port�w
    PORTD |= ( 1 << P_C_1 );
    PORTD |= ( 1 << P_C_2 );
    PORTB |= ( 1 << CH );

    //w��czenie obs�ugi przerwa�  3 kana��w
    GICR |= ( 1 << INT0 );
    GICR |= ( 1 << INT1 );
    GICR |= ( 1 << INT2 );

    //wlaczenie wyzwalania przerwania zmian� na  stan niski
    MCUCR |= ( 1 << ISC01 );  //INT0
    MCUCR |= ( 1 << ISC11 );  //INT1
    MCUCSR &= ~( 1 << ISC2 ); //INT2

    sei();    //odblokowanie przerwa�
}

//P�TLE PRZERWA� ZEWN�TRZNYCH

//przerwanie od przycisku 1
ISR( INT0_vect ) {
    //p1_mode++;
    //eliminacja drga� styk�w
    _delay_ms( 30 );
    if ( !( PIND & P_C_1 ) ) {
        p1_mode++;
    }
    _delay_ms( 30 );
}

//przerwanie od przycisku 2
ISR( INT1_vect ) {
    //p2_mode++;
    //eliminacja drga� styk�w
    _delay_ms( 30 );
    if ( !( PIND & P_C_2 ) ) {
        p2_mode++;
    }
    _delay_ms( 30 );
}

//przerwanie od przycisku 3
ISR( INT2_vect ) {

    //d�ugie przytrzymanie przycisku
    _delay_ms( 500 );

    if ( !( PINB &_BV( CH ) ) )  {

        mode_lcd++;
        _delay_ms( 30 );

    } else {
        //ch_ch++;
        //eliminacja drga� styk�w
        _delay_ms( 30 );
        if ( !( PINB & CH ) ) {
            ch_ch++;
        }
        _delay_ms( 30 );
    }                


}

//testowanie dzia�ania diod led od alert�w
void test_led( void ) {
    PORTC &= ~( 1 << START_1 );
    PORTC &= ~( 1 << START_2 );
    _delay_ms( 200 );
    PORTC |= ( 1 << START_1 );
    PORTC |= ( 1 << START_2 );
    _delay_ms( 200 );
    PORTC &= ~( 1 << READY_1 );
    PORTC &= ~( 1 << READY_2 );
    _delay_ms( 200 );
    PORTC |= ( 1 << READY_1 );
    PORTC |= ( 1 << READY_2 );
    _delay_ms( 200 );
    PORTC &= ~( 1 << ALARM_1 );
    PORTC &= ~( 1 << ALARM_2 );
    _delay_ms( 200 );
    PORTC |= ( 1 << ALARM_1 );
    PORTC |= ( 1 << ALARM_2 );
    _delay_ms( 200 );

}

//odczyt i konwersja warto�ci baterii
void batt_power( void ) {
    u_batt = adc_channel_batt();                                    //odczyt warto�ci adc baterii
    u_batt_value = adc_convert_voltage( u_batt, Batt_coff );
    u_batt = map( u_batt_value, Ubatt_min, Ubatt_max, 0, 100 );          //konwersja warto�ci adc na % baterii

}

//ekran 5 - stan baterii
void batt_lcd_value( void ) {

    itoa( u_batt, conv, 10 );                                       //konwersja int na char
    lcd_gotoxy( 0, 0 );
    lcd_puts( "BAT:" );
    lcd_puts( conv );
    lcd_puts( " % " );                                               //wy�wietlenie wraz ze znakiem
    lcd_puts( "U=" );
    dtostrf( u_batt_value, 3, 2, conv );
    lcd_puts( conv );
    lcd_puts( "V" );
    lcd_gotoxy( 0, 1 );
    lcd_puts( "Ladowanie:" );

    if ( batt_charge == 0 ) {
        lcd_puts( "ON " );
    } else {
        lcd_puts( "OFF" );
    }
}

//odczyt i konwersja warto�ci nat�enia
void pot_val( void ) {
    pot_i = adc_channel_pot();                   //odczyt wartosci potencjometru
    u_pot_value = adc_convert_voltage( pot_i, Pot_coff );
    u_pot_value = map( u_pot_value, 0, Upot_max , 0, 3 );    //konwersja warto�ci z ADC na zakres ustawienia pr�du
}

//ustawienie i wy�wietlenie warto�ci maksymalnego pr�du
void pot_lcd( void ) {

    switch ( ch_ch ) {

    case 0:
        lcd_gotoxy( 14, 1 );
        lcd_puts( " " );
        lcd_gotoxy( 14, 0 );
        lcd_puts( " " );
        break;

    case 1:
        i_max_1_value = u_pot_value;

        lcd_gotoxy( 14, 1 );
        lcd_puts( " " );
        lcd_gotoxy( 14, 0 );
        lcd_puts( "*" );
        break;

    case 2:
        i_max_2_value = u_pot_value;

        lcd_gotoxy( 14, 1 );
        lcd_puts( "*" );
        lcd_gotoxy( 14, 0 );
        lcd_puts( " " );

        break;

    default:
        ch_ch = 0;
        break;


    }
}

//kolor led w zale�no�ci od statusu
void led_status( void ) {
    //KANA� 1
    switch ( p1_mode ) {

    case 0:
        PORTC &= ~( 1 << READY_1 );
        PORTC |= ( 1 << START_1 );
        PORTC |= ( 1 << ALARM_1 );
        break;

    case 1:
        PORTC &= ~( 1 << START_1 );
        PORTC |= ( 1 << READY_1 );
        PORTC |= ( 1 << ALARM_1 );

        break;

    case 2:
        PORTC &= ~( 1 << ALARM_1 );
        PORTC |= ( 1 << START_1 );
        PORTC |= ( 1 << READY_1 );

        break;

    case 3:
        PORTC |= ( 1 << ALARM_1 );
        PORTC |= ( 1 << START_1 );
        PORTC |= ( 1 << READY_1 );
        p1_mode = 0;
        break;

    default:
        p1_mode = 0;
        break;
    }

    //KANA� 2
    switch ( p2_mode ) {

    case 0:
        PORTC &= ~( 1 << READY_2 );
        PORTC |= ( 1 << START_2 );
        PORTC |= ( 1 << ALARM_2 );
        break;

    case 1:
        PORTC &= ~( 1 << START_2 );
        PORTC |= ( 1 << READY_2 );
        PORTC |= ( 1 << ALARM_2 );

        break;

    case 2:
        PORTC &= ~( 1 << ALARM_2 );
        PORTC |= ( 1 << START_2 );
        PORTC |= ( 1 << READY_2 );

        break;

    case 3:
        PORTC |= ( 1 << ALARM_2 );
        PORTC |= ( 1 << START_2 );
        PORTC |= ( 1 << READY_2 );
        p2_mode = 0;
        break;

    default:
        p2_mode = 0;
        break;
    }
}

//ustawienie przeka�nik�w
void relay_mode( void ) {
    //kana� 1
    if ( p1_mode == 1 ) {
        PORTD |= ( 1 << R_1 );
    } else {
        PORTD &= ~( 1 << R_1 );
    }

    //kana� 2
    if ( p2_mode == 1 ) {
        PORTD |= ( 1 << R_2 );
    } else {
        PORTD &= ~( 1 << R_2 );
    }
}

//sprawdzenie �adowania
void batt_charge_proces( void ) {
    if ( !( PIND &_BV( CHARGE ) ) ) {
        batt_charge = 1;
    } else {
        batt_charge = 0;
    }
}

//odczyt i konwersja kana��w pomiaru napi�cia i pr�du
void measure_and_conv_i_u_channels( void ) {

    //warto�� napi�cia 1
    u_1 = adc_channel_u1();
    u_1_value = adc_convert_voltage( u_1, U_coff );
    u_1_value = u_1_value + U_delta;

    //warto�� nat�enia 1
    i_1 = adc_channel_i1();
    i_1_value = adc_convert_voltage( i_1, I_coff );
    i_1_value = map( i_1_value, 0, 1.5, 0, 3 );

    //warto�� napi�cia 2
    u_2 = adc_channel_u2();
    u_2_value = adc_convert_voltage( u_2, U_coff );
    u_2_value = u_2_value + U_delta;

    //warto�� nat�enia 2
    i_2 = adc_channel_i2();
    i_2_value = adc_convert_voltage( i_2, I_coff );
    i_2_value = map( i_2_value, 0, 1.5, 0, 3 );

}

//sprawdzenie przeci��enia
void alarm_channel( void ) {
    //kana� 1
    if ( i_max_1_value <= i_1_value ) {
        p1_mode = 2;
    } else {

    }

    //kana� 2
    if ( i_max_2_value <= i_2_value ) {
        p2_mode = 2;
    } else {

    }

}

//wy�wietlenie mocy maksymalnej na kana�
void total_power( void ) {

    //kana� 1
    lcd_gotoxy( 0, 0 );
    lcd_puts( "1:P=" );
    p_1_value = u_1_value * i_1_value;
    dtostrf( p_1_value, 3, 3, conv );
    lcd_puts( conv );
    lcd_gotoxy( 12, 0 );
    lcd_puts( "W" );

    //kana� 2
    lcd_gotoxy( 0, 1 );
    lcd_puts( "2:P=" );
    p_2_value = u_2_value * i_2_value;
    dtostrf( p_2_value, 3, 3, conv );
    lcd_puts( conv );
    lcd_gotoxy( 12, 1 );
    lcd_puts( "W" );

}

//wy�wietlenie rezystancji obci��enia
void load_resistance( void ) {

    //kana� 1
    lcd_gotoxy( 0, 0 );
    lcd_puts( "1:R=" );
    r_1_value = u_1_value / i_1_value;
    if ( !( i_1_value == 0 ) ) {
        dtostrf( r_1_value, 3, 1, conv );
        lcd_puts( conv );
    } else {
        lcd_puts( "NC" );
    }
    lcd_gotoxy( 12, 0 );
    lcd_puts( "ohm" );

    //kana� 2
    lcd_gotoxy( 0, 1 );
    lcd_puts( "2:R=" );
    r_2_value = u_2_value / i_2_value;
    if ( !( i_2_value == 0 ) ) {
        dtostrf( r_2_value, 3, 1, conv );
        lcd_puts( conv );
    } else {
        lcd_puts( "NC" );
    }
    lcd_gotoxy( 12, 1 );
    lcd_puts( "ohm" );

}



//PETLA GLOWNA
int main( void ) {

    //op�nienie przed startem
    _delay_ms( 100 );

    //inicjalizacja
    lcd_init( LCD_DISP_ON ); //wyswietlacz
    lcd_led( 0 );
    lcd_home();
    lcd_gotoxy( 0, 0 );
    lcd_puts( "Uruchamianie" );
    lcd_gotoxy( 0, 1 );
    lcd_puts( "Prosze czekac" );

    RELAY_setup();       //RELAY
    ADC_setup();        //ADC
    LED_setup();        //LED
    INT_setup();        //SWITCH

    //port �adowania
    DDRD &= ~( 1 << CHARGE );      //port jako wej�cie
    PORTD &= ~( 1 << CHARGE );     //domy�lnie stan niski

    //op�nienie przed startem
    _delay_ms( 200 );

    //testowanie systemu
    test_led();    //LEDy

    //LCD - zakonczono inicjalizacje
    lcd_clrscr();
    lcd_home();
    lcd_gotoxy( 0, 0 );
    lcd_puts( "Zakonczono" );
    lcd_gotoxy( 0, 1 );
    lcd_puts( "Sukcesem" );
    _delay_ms( 200 );
    lcd_clrscr();

    while ( 1 ) {

        relay_mode();    //kontrola przeka�nik�w
        led_status();    //ustawienie statusu LED
        batt_power();    //odczyt warto�ci baterii
        old_mode_lcd = mode_lcd;  //kasowanie starego ekranu  - cz�� 1
        batt_charge_proces();    //kontrola �adowania baterii
        measure_and_conv_i_u_channels();    //odczyt warto�ci kana��w pomiaru ADC
        alarm_channel();        //sprawdzenie przeci��enia

        //tryb wyswietlania lcd
        switch ( mode_lcd ) {

            //ekran 1 - domy�lny - bie��ce warto�ci napi�cia  i pr�du
        case 0:
            //warto�� napi�cia 1
            lcd_gotoxy( 0, 0 );
            lcd_puts( "1:U=" );
            dtostrf( u_1_value, 3, 2, conv );
            lcd_puts( conv );
            lcd_gotoxy( 8, 0 );
            lcd_puts( "V" );

            //warto�� nat�enia 1
            lcd_gotoxy( 9, 0 );
            lcd_puts( "I=" );
            dtostrf( i_1_value, 3, 2, conv );     //konwersja float na tekst - minimalna dlugo�� stringu 3, dok�adno�� 2 miesjca po przecinku
            lcd_puts( conv );
            lcd_puts( "A" );

            //warto�� napi�cia 2
            lcd_gotoxy( 0, 1 );
            lcd_puts( "2:U=" );
            dtostrf( u_2_value, 3, 2, conv );
            lcd_puts( conv );
            lcd_gotoxy( 8, 1 );
            lcd_puts( "V" );

            //warto�� nat�enia 2
            lcd_gotoxy( 9, 1 );
            lcd_puts( "I=" );
            dtostrf( i_2_value, 3, 2, conv );     //konwersja float na tekst - minimalna dlugo�� stringu 3, dok�adno�� 2 miesjca po przecinku
            lcd_puts( conv );
            lcd_puts( "A" );

            break;

            //ekran 2 - maksymalne warto�ci pr�du
        case 1:

            pot_val();     //odczyt warto�ci
            pot_lcd();    //przelaczenie potencjometru

            //linia 1 - kana� 1
            lcd_gotoxy( 0, 0 );
            lcd_puts( "1:Imax=" );
            dtostrf( i_max_1_value, 4, 2, conv );
            lcd_puts( conv );
            lcd_puts( "A" );

            //linia 2 - kana� 2
            lcd_gotoxy( 0, 1 );
            lcd_puts( "2:Imax=" );
            dtostrf( i_max_2_value, 4, 2, conv );
            lcd_puts( conv );
            lcd_puts( "A" );

            break;

            //ekran 3 - bie��ca moc
        case 2:
            total_power();       //wy�wietlanie mocy ca�kowitej

            break;

            //ekran 4 - rezystancja wewn�trzna �r�d�a
        case 3:

            load_resistance(); //wy�wietlenie rezystancji �r�d�a
            break;


            //ekran 5 - parametry baterii
        case 4:
            batt_lcd_value(); //wy�wietlenie stanu bateri

            break;

        default:
            mode_lcd = 0;
            break;
        }

        //kontrola stanu baterii - zgaszenie wy�wietlacza jesli mniej ni� 15%
        if (( u_batt <= 10 ) || ( !( batt_charge = 1 ) ) ) {     //je�li baterii mniej ni� 15% i nie ma �adowania wy��cz ekran
            lcd_led( 1 );
        } else {
            lcd_led( 0 );
        }

        //kasowanie starego ekranu  - cz�� 1
        if ( old_mode_lcd == mode_lcd ) {
        } else {
            lcd_clrscr();     //wyczy�� ekran
        }



    }
}

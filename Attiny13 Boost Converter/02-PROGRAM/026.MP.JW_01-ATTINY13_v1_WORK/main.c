#define F_CPU 9600000UL

//BIBLIOTEKI
#include <util/delay.h>
#include <avr/io.h>        

//Uin=10,36 V
//Uout=0,943 V

//PORTY                  
#define OUT_PWM       PB0        
#define IN_ADC_MES      PB4    //pot pin  

#define U_set 217  //869         for 10 bits, 207 for 8 bits, voltage divider 11,2=Vin/Vout      
#define D_min 10         
#define D_max 70        

//global variables                                         
uint16_t DUTY = 0;                                                   
volatile uint16_t ADC_DATA = 0;   
uint8_t OVC = 0;   

                                                                                                                                                                                          
//ADC init
void adc_setup ( void ) {

    DDRB &= ~_BV( IN_ADC_MES );
    // Set the ADC input to PB4/ADC2

    ADMUX |= ( 1 << MUX1 );                                     
    ADMUX |= ( 1 << REFS0 );      //Ureff 1,1V 
    ADMUX |= ( 1 << ADLAR );
                                                                                                                                                             
    //Clock/8 
   // ADCSRA |= ( 1 << ADPS1 ) | ( 1 << ADPS1 )| ( 1 << ADPS0 ) | ( 1 << ADEN );  
    ADCSRA |= ( 1 << ADPS0 ) | ( 1 << ADEN );     
                                                  
    DIDR0 |= ( 1 << ADC2D ) ;   

}

//adc read
int adc_read( void ) {
    // Start the conversion
    ADCSRA |= ( 1 << ADSC );
    _delay_ms( 10 ); 
    // Wait for it to finish
    while ( ADCSRA & ( 1 << ADSC ) );
    _delay_ms( 10 ); 
    return ADCH;                  
}

//delay in us
void delay_us( int us ) {
    for ( int i = 0; i < us; i++ ) {
        _delay_us( 1 );
    }
}

//delay in ms                                                 
void delay_ms( int ms ) {
    for ( int i = 0; i < ms; i++ ) {
        _delay_ms( 1 );
    }
}        

//pwm init
void pwm_setup( void ) {
    //set output
    DDRB |= ( 1 << OUT_PWM  );

    // Set Timer 0 prescaler to clock/1.
    TCCR0B |= ( 1 << CS00 );

    // Set to 'Fast PWM' mode
    TCCR0A |= ( 1 << WGM00 ) | ( 1 << WGM01 );        

    // Clear OC0B output on compare match, upwards counting.                       
    TCCR0A |= ( 1 << COM0A1 );          
                                                                                                
}
                                                     

//write pwm and set
void pwm_write (uint8_t val ) {
    OCR0A = val;
}   
        
 //map
uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//constrain
uint16_t constrain(uint16_t x,uint16_t  a,uint16_t b) {
    if(x<a) {
        return a;
    }
    else if(x>b) {
        return b;  
    }
    else
        return x;
}


//main loop
int main( void ) {

    adc_setup();                //init ADC
    pwm_setup();            //init PWM  
    
    pwm_write(100);

    while ( 1 ) {
                           
            ADC_DATA = adc_read(); 
                
          if(ADC_DATA <=(U_set+10))
          {
            if(ADC_DATA<U_set)
            {              
                                                                              
                DUTY++;  
            } 
            else if(ADC_DATA>U_set)
            {                                                                                                       
              //   if(ADC_DATA>250)
             //    {  
              //   pwm_write(1);
            //     delay_ms( 100 );                                                                                                                   
             //    }   
            //     else
             //    {
                 DUTY--;                                                                                                                   
             //    }                                        
                
            }
            else
            {   
                                            
            }  
            
          DUTY = constrain(DUTY , 0, ((255*D_max)/100));    
          pwm_write(DUTY);
          //_delay_ms( 200 ); 
          }
          else
          {   
          OVC = 1;
          
          }                             
            
          if(OVC==1)
          { 
          pwm_write(0);
          //_delay_ms( 100 ); 
          OVC = 0;            
          }                                               
            
            
             
           // 
           // pwm_write(20); 
                          
           //DUTY = constrain(DUTY , ((255*D_min)/100), ((255*D_max)/100));    //set pwm range as additional condition      
                                                                          
  }                                                                     
                                     
}            
                  
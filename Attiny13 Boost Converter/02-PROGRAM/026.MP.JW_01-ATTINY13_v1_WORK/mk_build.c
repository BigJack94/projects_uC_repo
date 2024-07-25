#include <avr/io.h>
#include <avr/pgmspace.h>

#include "mk_build.h"

/* -------------- INSTRUKCJA U¯YCIA --------------------------
  jeœli chcesz u¿ywaæ opcji MkBuild w MkClipse 
  zainkluduj do dowolnego pliku projektu "mk_build.h" po to 
  aby móc korzystaæ ze stringów zdefiniowanych poni¿ej
  -------------- przyk³ad: --------------
    // zainkluduj na pocz¹tku dowolnego pliku projektu
    #include <avr/pgmspace.h>
    #include "mk_build.h"

    // nastêpnie u¿yj w kodzie swojego programu
    lcd_cls();
    lcd_str_P( PSTR("FW version:") );
    lcd_str_P( MkBuildVersion );
    lcd_locate( 1,0 );
    lcd_str_P( PSTR("build:") );
    lcd_str_P( MkBuildNumber );
------------------------------------------------------------ */

/* Do not any changes in next four lines */
/* These lines will be updated automaticaly by MkClipse */
const char MkBuildVersion[] PROGMEM = {"1.0.0"};
const char MkBuildNumber[] PROGMEM = {"680"};
const char MkBuildDate[] PROGMEM = {"25.07.2024"};
const char MkBuildTime[] PROGMEM = {"21:16:50"};


//Adres http://192.168.4.1
// device.local

//biblioteki
//WiFi
#ifdef ESP32
#include <WiFi.h>
#include <AsyncTCP.h>
#else
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#endif
#include <ESPAsyncWebServer.h>
//pamiec eeprom
#include <EEPROM.h>
//obsluga RTC
#include "RTClib.h"
//mDSN
#include <ESP8266mDNS.h>
//ledy neopixel
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#include <pcf8574.h> //ekspander


//HARDWARE----------------------------------------------------
//konfiguracja pinow
#define led_pin 13  //pin led
#define SDA_pin 4    //i2c SDA
#define SCL_pin 5    //i2c SCL
#define SW_pin A0    //obecnosc napiecie zasilania

//konfiguracja pinow ekspandera
#define pump_add_pin 3  //studnia
#define pump_main_pin 4 //zbiornik
#define tank_add_min_pin 0  //studnia
#define tank_main_min_pin 1 //zbiornik
#define tank_main_max_pin 2 //zbionik

//zmienne obsługi czujników i wyjsc pompy
uint8_t pump_add_state = 1;
String pump_add_text = "Pompa nie pracuje";

uint8_t pump_main_state = 1;
String pump_main_text = "Pompa nie pracuje";

uint8_t tank_add_min_state = 1;
String tank_add_min_text = "Brak czynnika";

uint8_t tank_main_min_state = 1;
String tank_main_min_text = "Brak czynnika";

uint8_t tank_main_max_state = 1;
String tank_main_max_text = "Brak czynnika";


// instancja bibliotek
RTC_DS1307 rtc;  //RTC
PCF8574 ex1(0x20); //ekspander

//led RGB
#define led_number 1 //ilosc zastosowanych ledow
Adafruit_NeoPixel pixels(led_number, led_pin, NEO_GRB + NEO_KHZ800); //ledy RGB

//zmienne do obsługi programu - gotowe
uint8_t led_mode = 0;
uint8_t relay_1 = 0;

//obsluga przycisku
uint8_t short_sw = 1;
uint8_t long_sw = 0;
uint8_t long_long_sw = 0;

uint32_t sw1 = 0;
uint32_t sw2 = 0;  
uint32_t sw3 = 0; 

//obsluga opoznienia milis
uint64_t milis_accual = 0;
uint64_t milis_1 = 0;

uint64_t milis_pump_main_1 = 0;
uint64_t milis_pump_main_2 = 0;

uint64_t counter_pump_main_1 = 0;
uint64_t counter_pump_main_2 = 0;

uint64_t milis_pump_add_1 = 0;
uint64_t milis_pump_add_2 = 0;

uint64_t counter_pump_add_1 = 0;
uint64_t counter_pump_add_2 = 0;

//zmienne do obslugi trybu pracy urzadzenia
String text_device_mode = "1";                   //zmienna string aktualnego trybu WiFi
int data_device_mode = 1;                         //aktualny tryb WiFi
int data_device_mode_buff = 1;
const char* id_device_mode= "device_mode";  //identyfikator aktualnego trybu WiFi

//zmienne do obslugi opoznienia zmiany pracy pomp
String text_delay_work =  "1";                   //zmienna string aktualnego trybu WiFi
int data_delay_work = 1;                         //aktualny tryb WiFi
int data_delay_work_buff = 1;
const char* id_delay_work= "delay_work";  //identyfikator aktualnego trybu WiFi

//zmienne do obslugi wielkosci tablic
const int data_size = 64;                               //wielkosc pamieci na dane czasu przekaznikow
const int wifi_size = 20;                               //wielkosc pamieci na dane wifi
const int add_data_size = 10;                               //wielkosc pamieci na dane czasu przekaznikow
const int eeprom_size = ((4 * wifi_size) + data_size + add_data_size);  //wielkosci pamieci eeprom

//tablice do obslugi daty
int buff_memory_1[data_size];  //bufor daty zapisu do pamieci - bufor pamieci eeprom

//STRONA----------------------------------------------------
//zmienne do obslugi strony - dane czasu aktualnego GODZINY MINUTY
String text_acc_time;                   //zmienna string do obslugi aktualnego czasu
int acc_time_h;                         //aktualny czas minuty
int acc_time_min;                       //aktualny czas minuty
const char* id_acc_time = "acc_h_min";  //identyfikator linijek aktualnego czasu na stronie

//zmienne do obslugi zapisu nazwy wifi w trybie AP
char buff_name_wifi[wifi_size];              //nazwa sieci
char buff_pass_wifi[wifi_size];              //haslo sieci
String buff_1_name_text_wifi;                //nazwa sieci jako teskt
String buff_1_pass_text_wifi;                //haslo sieci jako teskt
String text_name_wifi;                       //zmienna string do obslugi aktualnej nazwy wifi
String text_pass_wifi;                       //zmienna string do obslugi aktualnego hasla wifi
const char* id_name_wifi = "wifi_name";      //identyfikator linijek w stronie
const char* id_pass_wifi = "wifi_password";  //identyfikator linijek w stronie

//zmienne do obslugi zapisu nazwy wifi w trybie ST
char buff_name_wifi_st[wifi_size];              //nazwa sieci
char buff_pass_wifi_st[wifi_size];              //haslo sieci
String buff_1_name_text_wifi_st;                //nazwa sieci jako teskt
String buff_1_pass_text_wifi_st;                //haslo sieci jako teskt
String text_name_wifi_st;                       //zmienna string do obslugi aktualnej nazwy wifi
String text_pass_wifi_st;                       //zmienna string do obslugi aktualnego hasla wifi
const char* id_name_wifi_st = "wifi_name_st";      //identyfikator linijek w stronie
const char* id_pass_wifi_st = "wifi_password_st";  //identyfikator linijek w stronie

//zmienne do obslugi strony - dane przekaznikow GODZINA MINUTA WLACZENIA, GODZINA MINUTA WYLACZENIA - 1 KANAL X4 PRZEKAZNIKI X4
String text_value[(data_size / 2)];  //zmienna string do obslugi tresci ze strony
int data_buff[data_size];            //bufor daty aktualny - bufor danych w postali liczbowej
const char* id_value[(data_size / 2)] = { "r1.st1", "r1.sp1", "r1.st2", "r1.sp2", "r1.st3", "r1.sp3", "r1.st4", "r1.sp4", "r2.st1", "r2.sp1", "r2.st2", "r2.sp2", "r2.st3", "r2.sp3", "r2.st4", "r2.sp4",
                                          "r3.st1", "r3.sp1", "r3.st2", "r3.sp2", "r3.st3", "r3.sp3", "r3.st4", "r3.sp4", "r4.st1", "r4.sp1", "r4.st2", "r4.sp2", "r4.st3", "r4.sp3", "r4.st4", "r4.sp4" };  //identyfikator linijek w stronie

//zmienne do wyswietlania dzialania przekaznikow
String R1_State = "OFF";

//zmienne do nadpisania RTC
int rtc_active = 0;
const char* rtc_id = "rtc_value";
String rtc_text;

//zmienne do obslugi tryby WiFi
String text_wifi_mode = "1";                   //zmienna string aktualnego trybu WiFi
int data_wifi_mode = 1;                         //aktualny tryb WiFi
int data_wifi_mode_buff = 1;
const char* id_wifi_mode= "wifi_mode";  //identyfikator aktualnego trybu WiFi

//zmienne do nadpisania ustawien wifi w trybie AP
int wifi_set_active = 0;
const char* wifi_set_id = "wifi_set_value";
String wifi_set_text;

//zmienne do nadpisania ustawien wifi w trybie ST
int wifi_set_active_st = 0;
const char* wifi_set_id_st = "wifi_set_value_st";
String wifi_set_text_st;

//zmienne do zapisuj do eeprom
int memory_save_active = 0;
const char* memory_save_id = "memory_save_value";
String memory_save_text;

//te ustawienia dzialaja
//domysle parametry sieci wifi po resecie
char* ssid_default = "default_ssid";
char* password_default = "default_password";
uint8_t connect_status = 0;  //status polaczenia
const char* mDSN_name = "device";

//wyglad strony
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>AUTOMAT</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  </head><body>
  <h1>Sterownik do podlewania</h1> 
  <form action="/get">
   <h2>Aktualny czas</h2> 
   <input type="time" step="60" name="acc_h_min" value="%V_acc_h_min%" ><br><br>
   Nadpisz czas zegara <input type="checkbox" name="rtc_value" value="%RTC_VALUE%" ><br>
   <h2>Ustawienia czasow pracy</h2>
   <p>Stan: %R1_VAL% </p> 
    Start 1 <input type="time" step="60" name="r1.st1" value="%Vr1.st1%" ><br><br>
    Stop 1 <input type="time" step="60" name="r1.sp1" value="%Vr1.sp1%"  ><br><br>
    Start 2 <input type="time" step="60" name="r1.st2" value="%Vr1.st2%" ><br><br>
    Stop 2 <input type="time" step="60" name="r1.sp2" value="%Vr1.sp2%"  ><br><br>
    Start 3 <input type="time" step="60" name="r1.st3" value="%Vr1.st3%" ><br><br>
    Stop 3 <input type="time" step="60" name="r1.sp3" value="%Vr1.sp3%"  ><br><br>
    Start 4 <input type="time" step="60" name="r1.st4" value="%Vr1.st4%" ><br><br>
    Stop 4 <input type="time" step="60" name="r1.sp4" value="%Vr1.sp4%"  ><br><br>
   <h3>Stan wejsc/wyjsc </h3>
    Czujnik poziomu wody rezerwy (zbiornika) min: %Vtank_main_min% <br><br>
    Czujnik poziomu wody rezerwy (zbiornika) max: %Vtank_main_max% <br><br>
    Czujnik poziomu wody zasilania: %Vtank_add_min% <br><br>
    Pompa zasilania rezerwy (zbiornika): %Vpump_add% <br><br>
    Pompa podlewania: %Vpump_main% <br><br>
   <h3>Dodatkowe ustawienia </h3>
   Opoznienie zmiany stanu pomp [1-120s] <input type="number" step="1" min="1" max="120" name="delay_work" value="%Vdelay_work%" ><br><br>
   Tryb pracy urzadzenia <input type="number" step="1" min="0" max="6" name="device_mode" value="%Vdevice_mode%"><br><br>
    0-STOP  <br><br>
    1-Tylko pompa podlewania - tryb auto <br><br>
    2-Tylko pompa rezerwy - tryb auto <br><br>
    3-Obie pompy - tryb auto <br><br>
    4-Tylko pompa podlewania - wymuszenie ciaglej pracy <br><br>
    5-Tylko pompa rezerwy - wymuszenie ciaglej pracy <br><br>
    6-Obie pompy - wymuszenie ciaglej pracy <br><br>
   <h3>Aktualne ustawienia sieci</h3>
    Domyslny tryb pracy sieci (po utracie zasilania) <input type="number" min="0" max="3" name="wifi_mode" value="%Vwifi_mode%"><br><br>
    0-Siec wylaczona <br><br>
    1-Access Point (tworzy wlasna siec) <br><br>
    2-Station mode (podlacza sie do istniejacej sieci) <br><br>
    3-Access Point z ustawieniami domyslnymi (tworzy wlasna siec) <br><br>
    Ustaw nowa siec w trybie Access Point<input type="checkbox" name="wifi_set_value" value="%WIFI_SET_VALUE%" ><br><br>
    Nazwa AP (Max. 16 znakow) <input type="text" maxlength="16" name="wifi_name" value="%Vwifi_name%"  required><br><br>
    Haslo AP (Max. 16 znakow) <input type="text" maxlength="16" name="wifi_password" value="%Vwifi_password%"  ><br><br>
    Podlacz sie do istniejacej sieci (tryb Station Mode)<input type="checkbox" name="wifi_set_value_st" value="%WIFI_SET_VALUE_st%" ><br><br>
    Nazwa ST (Max. 16 znakow) <input type="text" maxlength="16" name="wifi_name_st" value="%Vwifi_name_st%"  required><br><br>
    Haslo ST (Max. 16 znakow) <input type="text" maxlength="16" name="wifi_password_st" value="%Vwifi_password_st%"  ><br><br>
    Zapisz dane do pamieci <input type="checkbox" name="memory_save_value" value="%MEMORY_SAVE_VALUE%" ><br><br><center>
    <input type="submit" value="WPROWADZ NOWE USTAWIENIA"><br><br>
  </form>
</body></html>)rawliteral";

void notFound(AsyncWebServerRequest* request) {
  request->send(404, "text/plain", "Not found");
}


AsyncWebServer server(80);

//aktualizacja RTC
void rtc_save(void) {
  if (rtc_active == 1) {
    rtc.adjust(DateTime(2023, 1, 1, acc_time_h, acc_time_min, 0));  //ustawienie czasu rok, miesiac, dzien miesiaca, godzina, minuty, sekundy
  }
  rtc_active = 0;
}

//funkcja obslugi strony - konwersja ze strony do hardwaru
void web_start(void) {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", index_html, processor);
  });

  server.on("/get", HTTP_GET, [](AsyncWebServerRequest* request) {
    //aktualny czas------------------------------------------------------------
    if (request->hasParam(id_acc_time)) {
      text_acc_time = request->getParam(id_acc_time)->value();
    }
    //aktualny czas kanal 1
    if (request->hasParam(id_value[0])) {
      text_value[0] = request->getParam(id_value[0])->value();
    }

    if (request->hasParam(id_value[1])) {
      text_value[1] = request->getParam(id_value[1])->value();
    }

    if (request->hasParam(id_value[2])) {
      text_value[2] = request->getParam(id_value[2])->value();
    }

    if (request->hasParam(id_value[3])) {
      text_value[3] = request->getParam(id_value[3])->value();
    }

    if (request->hasParam(id_value[4])) {
      text_value[4] = request->getParam(id_value[4])->value();
    }

    if (request->hasParam(id_value[5])) {
      text_value[5] = request->getParam(id_value[5])->value();
    }

    if (request->hasParam(id_value[6])) {
      text_value[6] = request->getParam(id_value[6])->value();
    }

    if (request->hasParam(id_value[7])) {
      text_value[7] = request->getParam(id_value[7])->value();
    }
    //aktualny czas kanal 2------------------------------------------------------------
    if (request->hasParam(id_value[8])) {
      text_value[8] = request->getParam(id_value[8])->value();
    }

    if (request->hasParam(id_value[9])) {
      text_value[9] = request->getParam(id_value[9])->value();
    }

    if (request->hasParam(id_value[10])) {
      text_value[10] = request->getParam(id_value[10])->value();
    }

    if (request->hasParam(id_value[11])) {
      text_value[11] = request->getParam(id_value[11])->value();
    }

    if (request->hasParam(id_value[12])) {
      text_value[12] = request->getParam(id_value[12])->value();
    }

    if (request->hasParam(id_value[13])) {
      text_value[13] = request->getParam(id_value[13])->value();
    }

    if (request->hasParam(id_value[14])) {
      text_value[14] = request->getParam(id_value[14])->value();
    }

    if (request->hasParam(id_value[15])) {
      text_value[15] = request->getParam(id_value[15])->value();
    }
    //aktualny czas kanal 3------------------------------------------------------------
    if (request->hasParam(id_value[16])) {
      text_value[16] = request->getParam(id_value[16])->value();
    }

    if (request->hasParam(id_value[17])) {
      text_value[17] = request->getParam(id_value[17])->value();
    }

    if (request->hasParam(id_value[18])) {
      text_value[18] = request->getParam(id_value[18])->value();
    }

    if (request->hasParam(id_value[19])) {
      text_value[19] = request->getParam(id_value[19])->value();
    }

    if (request->hasParam(id_value[20])) {
      text_value[20] = request->getParam(id_value[20])->value();
    }

    if (request->hasParam(id_value[21])) {
      text_value[21] = request->getParam(id_value[21])->value();
    }

    if (request->hasParam(id_value[22])) {
      text_value[22] = request->getParam(id_value[22])->value();
    }

    if (request->hasParam(id_value[23])) {
      text_value[23] = request->getParam(id_value[23])->value();
    }
    //aktualny czas kanal 4------------------------------------------------------------
    if (request->hasParam(id_value[24])) {
      text_value[24] = request->getParam(id_value[24])->value();
    }

    if (request->hasParam(id_value[25])) {
      text_value[25] = request->getParam(id_value[25])->value();
    }

    if (request->hasParam(id_value[26])) {
      text_value[26] = request->getParam(id_value[26])->value();
    }

    if (request->hasParam(id_value[27])) {
      text_value[27] = request->getParam(id_value[27])->value();
    }

    if (request->hasParam(id_value[28])) {
      text_value[28] = request->getParam(id_value[28])->value();
    }

    if (request->hasParam(id_value[29])) {
      text_value[29] = request->getParam(id_value[29])->value();
    }

    if (request->hasParam(id_value[30])) {
      text_value[30] = request->getParam(id_value[30])->value();
    }

    if (request->hasParam(id_value[31])) {
      text_value[31] = request->getParam(id_value[31])->value();
    }
    //nazwa sieci i haslo------------------------------------------------------------AP
    if (request->hasParam(id_name_wifi)) {
      text_name_wifi = request->getParam(id_name_wifi)->value();
      buff_1_name_text_wifi = text_name_wifi;
    }
    if (request->hasParam(id_pass_wifi)) {
      text_pass_wifi = request->getParam(id_pass_wifi)->value();
      buff_1_pass_text_wifi = text_pass_wifi;
    }

    //nazwa sieci i haslo------------------------------------------------------------ST
    if (request->hasParam(id_name_wifi_st)) {
      text_name_wifi_st = request->getParam(id_name_wifi_st)->value();
      buff_1_name_text_wifi_st = text_name_wifi_st;
    }
    if (request->hasParam(id_pass_wifi_st)) {
      text_pass_wifi_st = request->getParam(id_pass_wifi_st)->value();
      buff_1_pass_text_wifi_st = text_pass_wifi_st;
    }

    //Tryb pracy sieci WiFi
    if (request->hasParam(id_wifi_mode)) {
      text_wifi_mode = request->getParam(id_wifi_mode)->value();
    }

    //Tryb pracy urzadzenia
    if (request->hasParam(id_device_mode)) {
      text_device_mode = request->getParam(id_device_mode)->value();
    }

    //czas opoznienia zadzialania urzadzenia
    if (request->hasParam(id_delay_work)) {
      text_delay_work = request->getParam(id_delay_work)->value();
    }

    //nadpisywanie RTC------------------------------------------------------------
    if (request->hasParam(rtc_id)) {
      rtc_text = request->getParam(rtc_id)->value();
      rtc_text = "true";
      rtc_active = 1;  //nadpisz RTC
    } else {
      rtc_text = "false";  //inaczej nie dziala
      rtc_active = 0;      //nie nadpisuj RTC
    }

    //nadpisywanie WiFi------------------------------------------------------------AP
    if (request->hasParam(wifi_set_id)) {
      wifi_set_text = request->getParam(wifi_set_id)->value();
      wifi_set_text = "true";
      wifi_set_active = 1;  //nadpisz dane sieci
    } else {
      wifi_set_text = "false";  //inaczej nie dziala
      wifi_set_active = 0;      //nie nadpisuj danych sieci
    }

    //nadpisywanie WiFi------------------------------------------------------------ST
    if (request->hasParam(wifi_set_id_st)) {
      wifi_set_text_st = request->getParam(wifi_set_id_st)->value();
      wifi_set_text_st = "true";
      wifi_set_active_st = 1;  //nadpisz dane sieci
    } else {
      wifi_set_text_st = "false";  //inaczej nie dziala
      wifi_set_active_st = 0;      //nie nadpisuj danych sieci
    }

    //nadpisywanie EEPROM------------------------------------------------------------
    if (request->hasParam(memory_save_id)) {
      memory_save_text = request->getParam(memory_save_id)->value();
      memory_save_text = "true";
      memory_save_active = 1;  //nadpisz eeprom
    } else {
      memory_save_text = "false";  //inaczej nie dziala
      memory_save_active = 0;      //nie nadpisuj eepromu
    }

    //dodatkowe operacje po zebraniu danych ze strony------------------------------------------------------------
    data_time_value_to_int();  //konwertowanie wartosci z tekstu
    //serial_debug();            //debug po wprowadzeniu danych
    rtc_save();  //aktualizacja czasu RTC

    request->send(200, "text/html", "Zapisano<br><a href=\"/\">Powrot</a>");
  });
  server.onNotFound(notFound);
  server.begin();
}

//konwersja tekstu do liczby
void data_time_value_to_int(void) {

  //aktualny czas------------------------------------------------------------
  acc_time_h = text_acc_time.substring(0, 3).toInt();
  acc_time_min = text_acc_time.substring(3).toInt();

  //przekazniki------------------------------------------------------------
  int u = 0;  //zmienna pomocnicza
  for (int i = 0; i <= (data_size / 2); i++) {
    data_buff[u] = text_value[i].substring(0, 3).toInt();  //godziny
    u++;
    data_buff[u] = text_value[i].substring(3).toInt();  //minuty
    u++;
  }
  //wifi------------------------------------------------------------AP
  text_name_wifi.toCharArray(buff_name_wifi, wifi_size);
  text_pass_wifi.toCharArray(buff_pass_wifi, wifi_size);

  //wifi------------------------------------------------------------ST
  text_name_wifi_st.toCharArray(buff_name_wifi_st, wifi_size);
  text_pass_wifi_st.toCharArray(buff_pass_wifi_st, wifi_size);

  //wifi tryb pracy
  data_wifi_mode =  text_wifi_mode.toInt();
  //urzadzenie tryb pracy
  data_device_mode =  text_device_mode.toInt();
  //opoznienie zadzialania pomp
  data_delay_work =  text_delay_work.toInt();


  //text_name_wifi.toCharArray(buff_name_wifi, text_name_wifi.length() + 1); //stara definicja
  //text_pass_wifi.toCharArray(buff_pass_wifi, text_pass_wifi.length() + 1);  //stara definicja
}

//konwersja liczby do wartosci do tekstu
void data_time_value_to_string(void) {

  //aktualny czas------------------------------------------------------------
  String text_h_RTC = String(acc_time_h, DEC);
  String text_min_RTC = String(acc_time_min, DEC);

  if (acc_time_h < 10) {
    text_h_RTC = "0" + text_h_RTC;
  }
  if (acc_time_min < 10) {
    text_min_RTC = "0" + text_min_RTC;
  }

  text_acc_time = text_h_RTC + ":" + text_min_RTC;


  //przekazniki------------------------------------------------------------
  String text_h;
  String text_min;

  int u = 0;  //zmienna pomocnicza
  for (int i = 0; i <= (data_size / 2); i++) {
    text_h = String(data_buff[u], DEC);
    text_min = String(data_buff[u + 1], DEC);

    if (data_buff[u] < 10) {
      text_h = "0" + text_h;
    }
    if (data_buff[u + 1] < 10) {
      text_min = "0" + text_min;
    }

    text_value[i] = text_h + ":" + text_min;
    u = u + 2;
  }
  //wifi------------------------------------------------------------AP
  text_name_wifi = String(buff_name_wifi);
  text_pass_wifi = String(buff_pass_wifi);

  //wifi------------------------------------------------------------ST
  text_name_wifi_st = String(buff_name_wifi_st);
  text_pass_wifi_st = String(buff_pass_wifi_st);
  //wifi------------------------------------------------------------MODE
  text_wifi_mode = String(data_wifi_mode);
  //device------------------------------------------------------------MODE
  text_device_mode = String(data_device_mode);
  //delay------------------------------------------------------------work
  text_delay_work = String(data_delay_work);
}

//funkcja aktualizowania zmiennych z hadrwaru do strony
String processor(const String& var) {

  //konwersja liczby do wartosci do tekstu
  data_time_value_to_string();

  //aktualny czas---------------
  if (var == "V_acc_h_min") {
    return text_acc_time;
  }
  //kanal 1---------------
  else if (var == "Vr1.st1") {
    return text_value[0];
  } else if (var == "Vr1.sp1") {
    return text_value[1];
  } else if (var == "Vr1.st2") {
    return text_value[2];
  } else if (var == "Vr1.sp2") {
    return text_value[3];
  } else if (var == "Vr1.st3") {
    return text_value[4];
  } else if (var == "Vr1.sp3") {
    return text_value[5];
  } else if (var == "Vr1.st4") {
    return text_value[6];
  } else if (var == "Vr1.sp4") {
    return text_value[7];
  }
  //nazwa sieci i haslo---------------AP
  else if (var == "Vwifi_name") {
    return buff_1_name_text_wifi;
  } else if (var == "Vwifi_password") {
    return buff_1_pass_text_wifi;
  }
  //nazwa sieci i haslo---------------ST
  else if (var == "Vwifi_name_st") {
    return buff_1_name_text_wifi_st;
  } else if (var == "Vwifi_password_st") {
    return buff_1_pass_text_wifi_st;
  }
  //tryb sieci WiFi
  else if (var == "Vwifi_mode") {
    return text_wifi_mode;
  }
  //tryb pracy urzadzenia
  else if (var == "Vdevice_mode") {
    return text_device_mode;
  }
  //opoznienie zmiany trybu pracy
  else if (var == "Vdelay_work") {
    return text_delay_work;
  }
  //stan przekaznikow---------------
  else if (var == "R1_VAL") {
    return R1_State;
  }
  //stan wejsc i wyjsc--------------- 
    else if (var == "Vtank_main_min") {
    return tank_main_min_text;
  }
    else if (var == "Vtank_main_max") {
    return tank_main_max_text;
  }
    else if (var == "Vtank_add_min") {
    return tank_add_min_text;
  }
    else if (var == "Vpump_add") {
    return pump_add_text;
  }
    else if (var == "Vpump_main") {
    return pump_main_text;
  }
  //---------------
  return String();
}

//HARDWARE------------------------------------------------------------>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<

//petla wyboru koloru LED
void led_mode_neo (int state_led, int bright_led)
{

  bright_led = 10;  //domyslnie wpisana wartość ignoruje podane 
        
  if (state_led == 0) //wylacz
  {
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));   //G  R B
    pixels.show();   //ustaw
    pixels.clear(); // Set all pixel colors to 'off'
  }

  else  if (state_led == 1) //czerwony
  {
    pixels.setPixelColor(0, pixels.Color(0, 255, 0));
    pixels.show();   //ustaw
  }
  else  if (state_led == 2) //zielony
  {
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    pixels.show();   //ustaw
  }
  else  if (state_led == 3) //niebieski
  {
    pixels.setPixelColor(0, pixels.Color(0, 0, 255));
    pixels.show();   //ustaw
  }
  else  if (state_led == 4) //fioletowy
  {
    pixels.setPixelColor(0, pixels.Color(0, 255, 255));
    pixels.show();   //ustaw
  }
  else  if (state_led == 5) //lazurowy
  {
    pixels.setPixelColor(0, pixels.Color(255, 0, 255));
    pixels.show();   //ustaw
  }
  else  if (state_led == 6) //zolty
  {
    pixels.setPixelColor(0, pixels.Color(255, 255, 0));
    pixels.show();   //ustaw
  }
  else  if (state_led == 7) //bialy
  {
    pixels.setPixelColor(0, pixels.Color(255, 255, 255));
    pixels.show();   //ustaw
  }
  else
  {
    state_led=0;
  }
  pixels.setBrightness(bright_led); //usatw jasnosc
}

//Reads a string out of memory
String read_string(int l, int p) {
  String temp;
  for (int n = p; n < l + p; ++n) {
    if (char(EEPROM.read(n)) != ';') {
      if (isWhitespace(char(EEPROM.read(n)))) {
        //do nothing
      } else temp += String(char(EEPROM.read(n)));

    } else n = l + p;
  }
  return temp;
}

//Write data to memory
void write_to_Memory(String s, String p) {
  s += ";";
  write_EEPROM(s, data_size);
  p += ";";
  write_EEPROM(p, (wifi_size + data_size));
  EEPROM.commit();
}

//Write data to memory
void write_to_Memory_st(String s, String p) {
  s += ";";
  write_EEPROM(s, (2*wifi_size) + data_size);
  p += ";";
  write_EEPROM(p, (3*wifi_size) + data_size);
  EEPROM.commit();
}

//write to memory
void write_EEPROM(String x, int pos) {

  for (int n = pos; n < x.length() + pos; n++) {

    if ((EEPROM.read(n)) != (x[n - pos])) {
      EEPROM.write(n, x[n - pos]);
    }
  }
}

//petla zapisu do pamieci eeprom - DO NAPISANIA
void save_EEPROM(void) {
  if (memory_save_active == 1) {

    led_mode_neo (5, 128); //zapis eeprom - lazurowy

    //porownanie danych
    for (int i = 0; i <= data_size - 1; i++)  //przejrzyj tablice danych
    {
      if (buff_memory_1[i] != data_buff[i])  //jesli tablica danych rozni sie od bufora pamieci to
      {
        buff_memory_1[i] = data_buff[i];  //zapisz bufor pamieci eeprom

        EEPROM.write(i, buff_memory_1[i]);  //zapisz do pamieci eeprom
        delay(10);                          //opoznienie do zapisu
      }
    }
    EEPROM.commit();  //potwierdzenie zapisywania

    //zapisz nazwe sieci
    //dodawanie nazwy wifi i hasla wifi AP
    write_to_Memory(buff_1_name_text_wifi, buff_1_pass_text_wifi);
    delay(10);

    //dodawanie nazwy wifi i hasla wifi
    write_to_Memory_st(buff_1_name_text_wifi_st, buff_1_pass_text_wifi_st);
    EEPROM.commit();  //potwierdzenie zapisywania
    delay(10);

    //zapisz dodatkowe ustawienia sieci
    
    if(data_wifi_mode_buff !=data_wifi_mode)
    {
      EEPROM.write(eeprom_size - 1, data_wifi_mode);
      EEPROM.commit();  //potwierdzenie zapisywania
      delay(10);
    }

       if(data_device_mode_buff !=data_device_mode)
    {
      EEPROM.write(eeprom_size - 2, data_device_mode);
      EEPROM.commit();  //potwierdzenie zapisywania
      delay(10);
    }

       if(data_delay_work_buff !=data_delay_work)
    {
      EEPROM.write(eeprom_size - 3, data_delay_work);
      EEPROM.commit();  //potwierdzenie zapisywania
      delay(10);
    }

    memory_save_active = 0;
   
    
    delay(200);
  }

  else {
  }
}


//funkcja ładowania danych z pamieci eeprom i przekazywania ich do tablic
void load_EEPROM(void) {
  //wczytaj dane z pamieci eeprom
  //przyporzadkuj dane do sterowania przekaznikami
  for (int i = 0; i <= data_size - 1; i++) {

    buff_memory_1[i] = EEPROM.read(i);  //do bufora danych
    data_buff[i] = buff_memory_1[i];    //do bufora pamieci
    delay(10);                          //opoznienie odczytu
  }

  //odczyt ustawien wifi AP
  buff_1_name_text_wifi = read_string(wifi_size, data_size);
  buff_1_pass_text_wifi = read_string(wifi_size, wifi_size + data_size);
  delay(10); 

  //odczyt ustawien wifi ST
  buff_1_name_text_wifi_st = read_string(wifi_size, (2*wifi_size) + data_size);
  buff_1_pass_text_wifi_st = read_string(wifi_size, (3*wifi_size) + data_size);  
  delay(10); 

  //odczyt danych z ustawieniami dodatkowymi
  data_wifi_mode=EEPROM.read(eeprom_size - 1);
  data_wifi_mode_buff=data_wifi_mode;
  delay(10); 

  data_device_mode=EEPROM.read(eeprom_size - 2);
  data_device_mode_buff=data_device_mode;
  delay(10); 

  data_delay_work=EEPROM.read(eeprom_size - 3);
  data_delay_work_buff=data_delay_work;
  delay(10); 
}

//odczyt RTC
void rtc_read(void) {
  //odczyt czasu z RTC
  DateTime now = rtc.now();
  acc_time_h = now.hour();
  acc_time_min = now.minute();

  //polnoc jako godzina 0
  if ((acc_time_h * 100 + acc_time_min > 2359) && (acc_time_h * 100 + acc_time_min < 1)) {
    acc_time_h = 0;
    acc_time_min = 0;
  }
}



//aktualizacja hasla wifi AP
void wifi_save(void) {
  if (wifi_set_active == 1) {
    delay(100);
    WiFi.softAPdisconnect(true);
    WiFi.disconnect(true);

    delay(100);
    WiFi.softAP(buff_name_wifi, buff_pass_wifi);

    delay(100);
    led_mode_neo (6, 128); //uruchomienie nowej sieci - zolty
    delay(500);
    wifi_set_active = 0;
  } else {
  }
}

//aktualizacja hasla wifi ST
void wifi_save_st (void) {
  if (wifi_set_active_st == 1) {
    delay(100);
    WiFi.softAPdisconnect(true);
    WiFi.disconnect(true);

    delay(100);
    WiFi.begin(buff_name_wifi_st, buff_pass_wifi_st);

    delay(100);
    led_mode_neo (6, 128); //uruchomienie nowej sieci - zolty
    delay(500);
    wifi_set_active_st = 0;
  } else {
  }
}

//sprawdzanie statusu podłączenia
void wifi_ST_status (void){
  if(WiFi.status()==WL_CONNECTED)
  {
    led_mode_neo (6, 128); //fioletowy jesli podlaczono
  }
  else
  {

  }
}


//sprawdzenie warunku przekaznikow - pojedynczy przedzial czasowy
int relay_control(int tab_num) {
  int h_st = data_buff[tab_num];
  int min_st = data_buff[tab_num + 1];
  int h_sp = data_buff[tab_num + 2];
  int min_sp = data_buff[tab_num + 3];
  int relay_state = 0;  //stan przekaznika

  //czasy stop i start rowne sobie
  if ((h_st * 100 + min_st) == (h_sp * 100 + min_sp)) {
    relay_state = 0;
  }
  //czas startu mniejszy niz start stopu - np. 13:24 ST > 14:15 SP
  else if ((h_st * 100 + min_st) < (h_sp * 100 + min_sp)) {
    if ((acc_time_h * 100 + acc_time_min >= h_st * 100 + min_st) && (acc_time_h * 100 + acc_time_min <= h_sp * 100 + min_sp)) {
      relay_state = 1;
    } else {
      relay_state = 0;
    }
  }
  //czas startu wiekszy niz start stopu - np. 23:24 ST > 01:15 SP
  else if ((h_st * 100 + min_st) > (h_sp * 100 + min_sp)) {
    //przed polnoca
    if ((acc_time_h * 100 + acc_time_min >= h_st * 100 + min_st) && (acc_time_h * 100 + acc_time_min <= 2359)) {
      relay_state = 1;
    }
    //po polnocy
    else if ((acc_time_h * 100 + acc_time_min >= 0) && (acc_time_h * 100 + acc_time_min <= h_sp * 100 + min_sp)) {
      relay_state = 1;
    } else {
      relay_state = 0;
    }
  }
  //stan nieprzewidziany
  else {
    relay_state = 0;
  }
  return relay_state;
}

//sprawdzanie warunku w poszczegolnych przekaznikach
void relay_calculate(void) {
  //przekaznik 1
  if ((relay_control(0) == 1) || (relay_control(4) == 1) || (relay_control(8) == 1) || (relay_control(12) == 1)) {
    relay_1 = 1;
    R1_State = "ON";
  } else {
    relay_1 = 0;
    R1_State = "OFF";
  }
}

//debug
void serial_debug(void) {
  Serial.println("czas RTC jako liczba");
  Serial.println(acc_time_h);
  Serial.println(acc_time_min);

  Serial.println("przekaznik 1 czas 1-start- jako liczba");
  Serial.println(data_buff[0]);
  Serial.println(data_buff[1]);
  Serial.println("przekaznik 1 czas 1-stop- jako liczba");
  Serial.println(data_buff[2]);
  Serial.println(data_buff[3]);

  Serial.println("wifi data jako string ze strony");
  Serial.println(text_name_wifi);
  Serial.println(text_pass_wifi);

  Serial.println("wifi data jako tablica danych");
  Serial.println(buff_name_wifi);
  Serial.println(buff_pass_wifi);

  Serial.println("nadpisz RTC");
  Serial.println(rtc_active);

  Serial.println("nadpisz WiFi");
  Serial.println(wifi_set_active);

  Serial.println("nadpisz EEPROM");
  Serial.println(memory_save_active);
}

//obsluga przycisku wyjscia analogowego
void sw_analog_state_milis(uint8_t PIN, uint16_t marg_value, uint16_t debouncing_time, uint16_t time_press_1, uint16_t time_press_2) {

  if (analogRead(PIN) < marg_value) {
      
    if ((millis()  - milis_1 >= debouncing_time)  && (millis()- milis_1 < time_press_1) && (millis() - milis_1 < time_press_2)) //krotkie
    {
          if(sw1 == 0)
          {
          short_sw = !short_sw;
          Serial.println("krotkie"); 
          sw1 ++;
          } 
        
    }

    else if ((millis() - milis_1 >= time_press_1) && (millis() - milis_1 < time_press_2))//dlugie
    {
            if(sw2==0)
            {
            long_sw = !long_sw;
            Serial.println("dlugie"); 
            sw2 ++; 
            }
    }    

    else if(millis() - milis_1 >= time_press_2) //dlugie dlugie
    {
            if(sw3==0)
            {         
            long_long_sw = !long_long_sw;
            Serial.println("dlugie dlugie");  
            sw3 ++;
            }
    }    

  }  

  else
  {
    milis_1 = millis(); //nadpisz czas w chwili kiedy przycisk nie jest nacisniety
    sw1 = 0;
    sw2 = 0;
    sw3 = 0;
  }
   
  //vcc = long_sw;  
}

//reset nazwy sieci
void wifi_reset(void) {
  if (long_long_sw == 1) {
    WiFi.softAPdisconnect(true);
    delay(100);
    WiFi.softAP(ssid_default, password_default);
    led_mode_neo (4, 128);  //reset sieci na domyslna - fioletowy 
    delay(1000);
    connect_status = 1;
    long_long_sw = 0;
    short_sw = 1;
  }
}

//zarzadzanie siecia
void network_admin(void) {
  if ((short_sw == 1) && (connect_status == 0))  //rozglaszaj siec
  {
          if (data_wifi_mode == 2)
          {
          WiFi.begin(buff_name_wifi_st, buff_pass_wifi_st);
          
          }
          else
          {
          WiFi.softAP(buff_name_wifi, buff_pass_wifi);
          
          }
          connect_status = 1;

  } else if ((short_sw == 0) && (connect_status == 1))  //nie rozglaszaj sieci
  {
    WiFi.softAPdisconnect(true);
    WiFi.disconnect(true);
    connect_status = 0;
  }

  //stan diody
  if (connect_status == 0)
  {
    led_mode_neo (2, 128); //nie rozglasza sieci - zielony
  }
  else if (connect_status == 1)
  {
   led_mode_neo (3, 128); //nie rozglasza sieci - zielony 
  }
}

//odczyt i zapis ekspandera
void IO_read_write (void)
{

  tank_add_min_state= digitalRead(ex1, tank_add_min_pin);
  tank_main_min_state= digitalRead(ex1, tank_main_min_pin);
  tank_main_max_state= digitalRead(ex1, tank_main_max_pin);

  //WEJSCIA
  if(tank_add_min_state == HIGH)
  {
    tank_add_min_text = "Brak czynnika";
  }
  else
  {
    tank_add_min_text = "Czynnik obecny";
  }

    if(tank_main_min_state == HIGH)
  {
    tank_main_min_text = "Brak czynnika";
  }
  else
  {
    tank_main_min_text = "Czynnik obecny";
  }

    if(tank_main_max_state == HIGH)
  {
    tank_main_max_text = "Brak czynnika";
  }
  else
  {
    tank_main_max_text = "Czynnik obecny";
  }

  //testy
  //pump_add_state = 0;
  //pump_main_state = 0;
  //WYJSCIA
  if(pump_add_state == 0)
  {
    digitalWrite(ex1, pump_add_pin, HIGH);
    pump_add_text = "Pompa pracuje";
  }
  else
  {
    digitalWrite(ex1, pump_add_pin, LOW);
    pump_add_text = "Pompa nie pracuje";
  }

  if(pump_main_state == 0)
  {
    digitalWrite(ex1, pump_main_pin, HIGH);
    pump_main_text = "Pompa pracuje";
  }
  else
  {
    digitalWrite(ex1, pump_main_pin, LOW);
    pump_main_text = "Pompa nie pracuje";
  }
}


//program obslugi pompy studni
void pump_add_work (void)
{
  if((tank_add_min_state == LOW) && (tank_main_min_state == HIGH) && (tank_main_max_state == HIGH)) //jesli w studni jest woda, a zbiornik jest pusty i nie przepelniony to wlacz pompe
  {
       
    if(counter_pump_add_1==0)
    {
      milis_pump_add_1 = millis();
      counter_pump_add_1 = 1;
    }
    else
    {

    }

    if((counter_pump_add_1 = 1)&&(millis()-milis_pump_add_1>=1000*data_delay_work))
    {
    pump_add_state = 0;
    counter_pump_add_2 = 0;
    milis_pump_add_2 = 0;
    }
    else
    {

    }


  }
  else if ((tank_add_min_state == HIGH) || (tank_main_max_state == LOW))
  {
    if(counter_pump_add_2==0)
    {
      milis_pump_add_2 = millis();
      counter_pump_add_2 = 1;
    }
    else
    {

    }
    if((counter_pump_add_2 = 1)&&(millis()-milis_pump_add_2>=1000*data_delay_work))
    {
    pump_add_state = 1;
    counter_pump_add_1 = 0;
    milis_pump_add_1 = 0;
    }
    else
    {

    }
  }
  else
  {

  }

}

//program obslugi pompy glownej
void pump_main_work (void)
{
  if((relay_1==1) && (tank_main_min_state == LOW)) //wlacz pompe jesli jest woda w zbiorniku i czas na podlewanie
  {

    if(counter_pump_main_1==0)
    {
      milis_pump_main_1 = millis();
      counter_pump_main_1 = 1;
    }
    else
    {

    }

    if((counter_pump_main_1 = 1)&&(millis()-milis_pump_main_1>=1000*data_delay_work))
    {
    pump_main_state = 0;
    counter_pump_main_2 = 0;
    milis_pump_main_2 = 0;
    }
    else
    {

    }
  }
  else
  {

    if(counter_pump_main_2==0)
    {
      milis_pump_main_2 = millis();
      counter_pump_main_2 = 1;
    }
    else
    {

    }
    if((counter_pump_main_2 = 1)&&(millis()-milis_pump_main_2>=1000*data_delay_work))
    {
    pump_main_state = 1;
    counter_pump_main_1 = 0;
    milis_pump_main_1 = 0;
    }
    else
    {

    }
  }
}

void setup() {

  
  //konfiguracja ledów
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
  pixels.begin();

  //komunikacja startu
  led_mode_neo (7, 128); //bialy kolor

  //ustawienia portu szeregowego
  Serial.begin(115200);

  //inicjalizacja pamieci eerpom
  EEPROM.begin(eeprom_size);

  delay(300);

  //wczytanie pamieci eeprom
  load_EEPROM();

  //uruchomienie sieci WiFi
  connect_status = 1;
  //WiFi.softAP(buff_name_wifi, buff_pass_wifi);
  
  if(data_wifi_mode == 1)
  {
  //konwertowanie danych z eepromu na dane do wczytania sieci wifi
  buff_1_name_text_wifi.toCharArray(buff_name_wifi, wifi_size);
  buff_1_pass_text_wifi.toCharArray(buff_pass_wifi, wifi_size);

  WiFi.softAP(buff_name_wifi, buff_pass_wifi);
  }
  else if (data_wifi_mode == 2)
  {
  buff_1_name_text_wifi_st.toCharArray(buff_name_wifi_st, wifi_size);
  buff_1_pass_text_wifi_st.toCharArray(buff_pass_wifi_st, wifi_size);

  WiFi.begin(buff_name_wifi_st, buff_pass_wifi_st);
  }
  else if (data_wifi_mode == 3)
  {
  WiFi.softAP(ssid_default, password_default);
  }
  else
  {
  connect_status = 0;
  }
  

  IPAddress myIP = WiFi.softAPIP();
  MDNS.begin(mDSN_name, WiFi.softAPIP());
  web_start();  //strona internetowa
  MDNS.addService("http", "tcp", 80);
  

  //inicjalizacja RTC
  rtc.begin();

    //definicja pinow ekspandera
  pinMode(ex1, pump_add_pin, OUTPUT);
  pinMode(ex1, pump_main_pin, OUTPUT);
  pinMode(ex1, tank_add_min_pin, INPUT);
  pinMode(ex1, tank_main_min_pin, INPUT);
  pinMode(ex1, tank_main_max_pin, INPUT);
  //stan poczatkowy pinow expandera
  digitalWrite(ex1, pump_add_pin, LOW);
  digitalWrite(ex1, pump_main_pin, LOW);

  led_mode_neo (8, 128); //zakonczenie startu - zgas
  delay(100);
}

void loop() {

  rtc_read();  //sprawdz czas

  MDNS.update();  //aktualizacja mDNS

  relay_calculate();  // sprawdzenie warunku czasu przekaznikow

  sw_analog_state_milis(SW_pin, 100, 35, 1000, 3000);  //odczyt przycisku z mechanizmem milis

  wifi_reset();     //reset wifi
  network_admin();  //widocznosc sieci wifi z domyslnymi ustawieniami

  delay(100);   

  wifi_save();    //aktualizacja wifi AP
  wifi_save_st();    //aktualizacja wifi ST
  wifi_ST_status(); //sprawdzenie podlaczenia do sieci lokalnej
  save_EEPROM();  //zapis do pamieci eeprrom

  IO_read_write();  //odczyt zapis ekspandera
  //tryb działania urzadzenia
  if(long_sw==0)
  {
        if(data_device_mode==1)
        {
          pump_main_work();
          pump_add_state = 1;
        }
        else if (data_device_mode==2)
        {
          pump_add_work();
          pump_main_state = 1;
        }
        else if(data_device_mode==3)
        {
          pump_add_work();
          pump_main_work();
        }
        else if(data_device_mode==4)
        {
          pump_add_state = 1;
          pump_main_state = 0;
        }
        else if(data_device_mode==5)
        {
          pump_add_state = 0;
          pump_main_state = 1;
        }
        else if(data_device_mode==6)
        {
          pump_add_state = 0;
          pump_main_state = 0;
        }
        else
        {
          pump_add_state = 1;
          pump_main_state = 1;
          led_mode_neo (1, 128); //praca wstrzymana koloro czerwony
        }
  }
  else if (long_sw>1)
  {
    long_sw = 0;
    
  }
  else
  {
    pump_add_state = 1;
    pump_main_state = 1;
    led_mode_neo (1, 128); //praca wstrzymana koloro czerwony
  }


  delay(100);
}

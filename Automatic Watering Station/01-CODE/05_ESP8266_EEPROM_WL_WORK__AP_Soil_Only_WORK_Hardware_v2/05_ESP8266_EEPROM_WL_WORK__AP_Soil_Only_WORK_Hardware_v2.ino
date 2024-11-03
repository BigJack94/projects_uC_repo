//Adres: 192.168.4.1
//Adres: device.local

//biblioteki wifi - OK
#ifdef ESP32
#include <WiFi.h>
#include <AsyncTCP.h>
#else
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#endif
#include <ESPAsyncWebServer.h>

//biblioteki sprzetowe
#include <EEPROM.h>
#include <EasyButton.h>
//mDSN
#include <ESP8266mDNS.h>

//konfiguracja pinów - OK
#define sw_pin 5               //przelacznik
#define status_led_r_pin 16    //dioda czerwona
#define status_led_g_pin 14    //dioda czerwona
#define status_led_b_pin 13    //dioda czerwona
#define pump_pin 12            //tranzystor pompy
#define soil_sensor_pin A0     //odczyt wartosci czujnik gleby
#define water_level_pin 4      //poziom wody odczyt
#define water_level_vcc_pin 2  //poziom wody zasilanie

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//zmienne do obslugi wielkosci tablic - OK
const int data_size = 3;                              //wielkosc pamieci na dane czasu przekaznikow
const int wifi_size = 20;                             //wielkosc pamieci na dane wifi
const int eeprom_size = (2 * wifi_size) + data_size;  //wielkosci pamieci eeprom

//obsluga poziomu wilgotnosci z czujnika - OK
String text_wet_sensor;
int data_wet_sensor = 0;

//obsluga poziomu wody z czujnika - OK
String text_water_level;
int data_water_level = 0;

//obsluga minimalnego poziomu wilgotnosci - OK
String text_wet_set;                   //zmienna string do obslugi tresci ze strony
int data_wet_set = 0;                  //bufor daty aktualny - bufor danych w postali liczbowej
const char* id_wet_set = "N_wet_set";  //identyfikator linijek w stronie

//obsluga wymuszenie pracy pompy- OK
String text_pump_work_force;
int data_pump_work_force = 0;
const char* id_pump_work_force = "N_pump_work_force";

//obsluga czasu pracy pompy - OK
String text_pump_time_set;                         //zmienna string do obslugi tresci ze strony
int data_pump_time_set = 0;                        //bufor daty aktualny - bufor danych w postali liczbowej
const char* id_pump_time_set = "N_pump_time_set";  //identyfikator linijek w stronie

//obsluga ilosci wlaczen pompy w cyklu - OK
String text_pump_cycle_set;                          //zmienna string do obslugi tresci ze strony
int data_pump_cycle_set = 0;                         //bufor daty aktualny - bufor danych w postali liczbowej
const char* id_pump_cycle_set = "N_pump_cycle_set";  //identyfikator linijek w stronie

//zmienne do nadpisania ustawien wifi - OK
int wifi_set_active = 0;
const char* wifi_set_id = "wifi_set_value";
String wifi_set_text;

//zmienne do zapisuj do eeprom - OK
int memory_save_active = 0;
const char* memory_save_id = "memory_save_value";
String memory_save_text;

//zmienne do obslugi zapisu nazwy wifi  - OK
char buff_name_wifi[wifi_size];          //nazwa sieci
char buff_pass_wifi[wifi_size];          //haslo sieci
String buff_1_name_text_wifi;            //nazwa sieci jako teskt
String buff_1_pass_text_wifi;            //haslo sieci jako teskt
String text_name_wifi;                   //zmienna string do obslugi aktualnej nazwy wifi
String text_pass_wifi;                   //zmienna string do obslugi aktualnego hasla wifi
const char* id_name_wifi = "wifi_name";  //identyfikator linijek w stronie
const char* id_pass_wifi = "wifi_pass";  //identyfikator linijek w stronie

//domysle parametry sieci wifi po resecie - OK
char* ssid_default = "default_ssid";
char* password_default = "default_password";
uint8_t connect_status = 0;  //status polaczenia
const char* mDSN_name = "device";

//bufory danych 
int data_buff[data_size]; 
int buff_memory_1[data_size];

//zmienne hardwarowe do obsługi programu
//-----------------------------------------------------------------------------------------------------------------------------DO WPROWADZENIA
int soil_min = 0;                //minimalna wilgotnosc gleby w %
int pump_master_sw = 0;          //zmienna do wymuszenia wlaczenia pompy
unsigned long pump_time = 3000;  // czas pracy pompy w ms
int pump_cycle_per_space = 3;    //ilosc zadzialan na cykl
//-----------------------------------------------------------------------------------------------------------------------------POMOCNICZE
int S_M_S_value = 0;                 //wartość czujnika gleby
int S_M_S_value_raw = 0;             //wartosc czujnika gleby bez przelcizenia
int alarm = 0;                       //alarm
int pump_max = 254;                  //maksymalna moc pompy
int pump_cycle = 0;                  // cykle pompy na dobe
int water_level = 0;                 //poziom wody
int pump_save = 0;                   //zmienna do zapisywania pierwszego cyklu zadzialania pompy
unsigned long accual_time = 0;       //zmienna do obliczania biezacego czasu
unsigned long first_time_pump = 0;   //zmienna pierwszego cyklu zadzialania
unsigned long pump_min_cycle_space = 24 * 60;  //ilosc minut na jeden cykl h*60
int led_mode = 0;                    //tryb diody led

//zmienne przycisku
int sw_short = 1;           //stan przełącznika przy krotkim wciśnięciu
int sw_long = 0;            //stan przełącznika przy długim wcisnieciu
int sw_reset = 0;         //stan przycisku przy sekwencji resetu
int sw_stop = 0;         //stan przycisku przy sekwencji

// instancja biblioteki przelacznikow
EasyButton button(sw_pin);

// funkcje przyciskow - GOTOWE
void LongPress() {
  sw_long = !sw_long;
  Serial.println("dlugie");
}

void ShortPress() {
  sw_short = !sw_short;
  Serial.println("krotkie");
}

void ResetPress() {

  sw_reset = !sw_reset;
  Serial.println("sewkencja resetu");
}

void StopPress() {

  sw_stop = !sw_stop;
  Serial.println("sewkencja stopu");
}

void buttonISR() {
  button.read();
}
//----------------------------------------------

//PĘTLE WŁASNE - HARDWARE

//sprawdzenie ilości wody w zbiorniku - GOTOWE
void tank_check(void) {
  digitalWrite(water_level_vcc_pin, HIGH);
  Serial.println("sprawdzanie stanu wody");
  delay(300);

  water_level = digitalRead(water_level_pin);  //odczyt poziomu wody

  if (water_level != 1)  //jesli brak wody to wlacz alarm
  {
    led_mode = 1;
    alarm = 1;
  } else {
    alarm = 0;
    
  }
  led_work();
  digitalWrite(water_level_vcc_pin, LOW);
  delay(100);
}

//sprawdzanie poziomu wilgotności gleby - GOTOWE
void soil_status(void) {
  int soil_wet = 255;  //czujnik w wodzie
  int soil_dry = 700;   //czujnik w powietrzu

  //obliczanie wartosci do skalowania
  S_M_S_value = analogRead(soil_sensor_pin);
  S_M_S_value_raw = S_M_S_value;
  S_M_S_value = map(S_M_S_value, soil_dry, soil_wet, 0, 100);

  //zabezpieczenie przed przewinieciem
  if(S_M_S_value >= 100)
  {
    S_M_S_value = 100;
  }
  else if (S_M_S_value <=0) {
   S_M_S_value = 0; 
  }
  else
  {
  S_M_S_value = S_M_S_value;   
  }
  
  Serial.println(S_M_S_value_raw);
}

//obsluga ledow - GOTOWE
void led_work(void) {
  if (led_mode == 0)  // RGB OFF
  {
    digitalWrite(status_led_r_pin, HIGH);
    digitalWrite(status_led_g_pin, HIGH);
    digitalWrite(status_led_b_pin, HIGH);
  }

  else if (led_mode == 1)  //R ON
  {
    digitalWrite(status_led_r_pin, LOW);
    digitalWrite(status_led_g_pin, HIGH);
    digitalWrite(status_led_b_pin, HIGH);
  }

  else if (led_mode == 2)  //G ON
  {
    digitalWrite(status_led_r_pin, HIGH);
    digitalWrite(status_led_g_pin, LOW);
    digitalWrite(status_led_b_pin, HIGH);
  }

  else if (led_mode == 3)  //B ON
  {
    digitalWrite(status_led_r_pin, HIGH);
    digitalWrite(status_led_g_pin, HIGH);
    digitalWrite(status_led_b_pin, LOW);
  }

  else {
    led_mode = 0;
  }
}

//tryb 1 sygnalizacji ledow
void led_syg_short(void) {

int led_in = led_mode;  

  for (int u = 0; u <= 10; u++)  //wykonaj petle
  {
    led_mode = 1;
    led_work();
    delay(50);
    led_mode = 0;
    led_work();
    delay(50);
  }
  led_mode = led_in;
  led_work();
}

//tryb 2 sygnalizacji ledow
void led_syg_long(void) {

  int led_in = led_mode;  

  for (int u = 0; u <= 5; u++)  //wykonaj petle
  {
    led_mode = 1;
    led_work();
    delay(150);
    led_mode = 0;
    led_work();
    delay(150);
  }
  led_mode = led_in;
  led_work();
}

//petla obslugi pompy - GOTOWE
void auto_pump(void) {

  if (S_M_S_value <= soil_min) {
    tank_check();  //przed uruchomieniem pompy sprawdz stan wody
  }
  if (((S_M_S_value <= soil_min) && (alarm == 0) && (pump_cycle < pump_cycle_per_space)) || (pump_master_sw == 1))  //warunki do wykonania
  {
    for (int i = 0; i >= pump_max; i++)  //rozpedz pompe i utrzymaj 5000 sekund
    {
      analogWrite(pump_pin, i);
      delay(1);
    }
    analogWrite(pump_pin, pump_max);
    delay(pump_time);

    for (int i = pump_max; i <= 0; i--)  //wylacz pompe
    {
      analogWrite(pump_pin, i);
      delay(1);
    }
    analogWrite(pump_pin, 0);

    //jesli cykl nie zostal wymuszony przez strone to zwieksz cykl pompy
    if (pump_master_sw == 0) {
      pump_cycle++;  //dodaj jeden cykl wlaczenia pompy
    }
    pump_master_sw = 0;
  } else {
    analogWrite(pump_pin, 0);
  }

  //zabezpieczenie przed przelaniem
  accual_time = millis();  //aktualizacja czasu pracy

  //zapisz czas pierwszego cyklu
  if ((pump_cycle == 1) && (pump_save == 0)) {
    first_time_pump = accual_time;
    pump_save = 1;
  }

  //sprawdz warunek czasu
  if (accual_time - first_time_pump >= (pump_min_cycle_space * 60 * 1000)) {
    pump_cycle = 0;
    pump_save = 0;
  }
}

//-------------------------------------------------------------------------------------------------------------------------

//strona glowna wifi - DODAC PODMIANKE WARTOSCI
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>ASU</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  </head><body>
  <h1>Automatyczna stacja uprawy</h> 
  <h2>Odczyt:</h2> 
  <p>Aktualny poziom wilgotnosci: %V_wet_sensor% [&#37;]</p> 
  <p>Ilosc wody w zbiorniku: %V_water_level% </p>   
  <h2>Zapis:</h2> 
  <form action="/get">
    Minimalny poziom wilgotnosc gleby [&#37;]: <input type="number" step="1" name="N_wet_set" value="%V_wet_set%" min="0" max="100" required><br><br>  
    Uruchom pompe jednorazowo <input type="checkbox" name="N_pump_work_force" value="%V_pump_work_force%" ><br><br> 
    Czas pracy pompy [s] (1-10s): <input type="number" step="1" name="N_pump_time_set" value="%V_pump_time_set%" min="1" max="10" required><br><br>    
    Dopuszczalna ilosc zalaczen pompy na dobe (1-10): <input type="number" step="1"name="N_pump_cycle_set" value="%V_pump_cycle_set%" min="1" max="10" required><br><br>    
    <h3>Aktualne ustawienia sieci</h3>
    Ustaw nowa siec <input type="checkbox" name="wifi_set_value" value="%WIFI_SET_VALUE%" ><br><br>
    Nazwa (Max. 16 znakow) <input type="text" maxlength="16" name="wifi_name" value="%Vwifi_name%"  required><br><br>
    Haslo (Max. 16 znakow) <input type="text" maxlength="16" name="wifi_pass" value="%Vwifi_password%"  ><br><br><center>
    Zapisz dane do pamieci <input type="checkbox" name="memory_save_value" value="%MEMORY_SAVE_VALUE%" ><br><br>
    <input type="submit" value="WPROWADZ NOWE USTAWIENIA"><br><br>
  </form> 
</body></html>)rawliteral";

//obsluga żadan OK
void notFound(AsyncWebServerRequest* request) {
  request->send(404, "text/plain", "Not found");
}

//wartosci wrzucane na strone - OK
void int_to_text_conv (void)
{
  //wartosc czujnika wilgotnosci
  text_wet_sensor = String(S_M_S_value, DEC);

  //wartosc czujnika poziomu wody
  if (water_level == 1) {
    text_water_level = "Dostateczna";
  } else {
    text_water_level = "Niedostateczna - dolej wody do zbiornika";
  }

  //wartosc minimalnej ustawionej wilgotnosci
  text_wet_set = String(data_wet_set, DEC);

  //wartosc czasu pracy pompy
  text_pump_time_set = String(data_pump_time_set, DEC);

  //ilosc cykli pracy pompy
  text_pump_cycle_set = String(data_pump_cycle_set, DEC);

  //wifi------------------------------------------------------------
  text_name_wifi = String(buff_name_wifi);
  text_pass_wifi = String(buff_pass_wifi);
}

//wartosci odczytywane ze strony - UAKTUALNIC KONWERSJE
void text_to_int_conv(void) {

  //wartosci zbierane ze strony------------------------------------------------------------
  data_wet_set = text_wet_set.toInt();
  data_pump_time_set = text_pump_time_set.toInt();
  data_pump_cycle_set = text_pump_cycle_set.toInt();

  //nadpisanie ustawien sprzetu
  soil_min = data_wet_set;                     //minimalna wilgotnosc gleby w %
  pump_time = 1000 * data_pump_time_set;       // czas pracy pompy w ms
  pump_cycle_per_space = data_pump_cycle_set;  //ilosc zadzialan na cykl

  //wifi------------------------------------------------------------
  text_name_wifi.toCharArray(buff_name_wifi, wifi_size);
  text_pass_wifi.toCharArray(buff_pass_wifi, wifi_size);
}

AsyncWebServer server(80);

//funkcja przetwarzajaca zmienne - OK
String processor(const String& var) {

  int_to_text_conv();

  //parametry pracy podlewania---------------
  if (var == "V_wet_sensor") {  //czujnik wilgotnosci - ok
    return text_wet_sensor;
  } else if (var == "V_water_level") {  //poziom wody - ok
    return text_water_level;
  } else if (var == "V_wet_set") {  //minimalny poziom wilgotnosci ok
    return text_wet_set;
  } else if (var == "V_pump_time_set") {  //czas pracy pompy - ok
    return text_pump_time_set;
  } else if (var == "V_pump_cycle_set") {  //czas pracy pompy - ok
    return text_pump_cycle_set;
  }

  //nazwa sieci i haslo---------------
  else if (var == "Vwifi_name") {
    return buff_1_name_text_wifi;
  } else if (var == "Vwifi_password") {
    return buff_1_pass_text_wifi;
  }

  return String();
}

//funkcja obslugi internetu - OK
void start_web(void) {
  // Send web page to client
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", index_html, processor);
  });

  server.on("/get", HTTP_GET, [](AsyncWebServerRequest* request) {
    //ustawiona minimalna wilgotnosc czujnika - OK
    if (request->hasParam(id_wet_set)) {
      text_wet_set = request->getParam(id_wet_set)->value();
    }

    //zapis wartosci uruchomienia pompy - OK
    if (request->hasParam(id_pump_work_force)) {
      text_pump_work_force = request->getParam(id_pump_work_force)->value();
      text_pump_work_force = "true";
      data_pump_work_force = pump_master_sw = 1;  //pompa dziala - pole wyboru zaznaczone
    } else {
      text_pump_work_force = "false";             //inaczej nie dziala
      data_pump_work_force = pump_master_sw = 0;  //pompa nie dziala - pole wybory odznaczone
    }

    //ustawiony czas pracy pompy - OK
    if (request->hasParam(id_pump_time_set)) {
      text_pump_time_set = request->getParam(id_pump_time_set)->value();
    }

    //ustawiona ilosc cykli pracy pompy - OK
    if (request->hasParam(id_pump_cycle_set)) {
      text_pump_cycle_set = request->getParam(id_pump_cycle_set)->value();
    }

    //nadpisywanie WiFi - OK
    if (request->hasParam(wifi_set_id)) {
      wifi_set_text = request->getParam(wifi_set_id)->value();
      wifi_set_text = "true";
      wifi_set_active = 1;  //nadpisz dane sieci
    } else {
      wifi_set_text = "false";  //inaczej nie dziala
      wifi_set_active = 0;      //nie nadpisuj danych sieci
    }

        //nazwa sieci i haslo------------------------------------------------------------
    if (request->hasParam(id_name_wifi)) {
      text_name_wifi = request->getParam(id_name_wifi)->value();
      buff_1_name_text_wifi = text_name_wifi;
    }
    if (request->hasParam(id_pass_wifi)) {
      text_pass_wifi = request->getParam(id_pass_wifi)->value();
      buff_1_pass_text_wifi = text_pass_wifi;
    }

    //nadpisywanie EEPROM - OK
    if (request->hasParam(memory_save_id)) {
      memory_save_text = request->getParam(memory_save_id)->value();
      memory_save_text = "true";
      memory_save_active = 1;  //nadpisz eeprom
    } else {
      memory_save_text = "false";  //inaczej nie dziala
      memory_save_active = 0;      //nie nadpisuj eepromu
    }    


    text_to_int_conv();  //konwersja tesktu ze strony na liczby
    debug_serial();      //UART debug

    request->send(200, "text/html", "Zapisano<br><a href=\"/\">Powrot</a>");
  });
  server.onNotFound(notFound);
  server.begin();
}

//obsuga wifi z przycisku
void network_admin(void) {

  if ((sw_short == 1) && (connect_status == 0))  //rozglaszaj siec
  {
     WiFi.softAP(buff_name_wifi, buff_pass_wifi);
    connect_status = 1;
    //led_mode = 3;
    tank_check(); 

  } else if ((sw_short == 0) && (connect_status == 1))  //nie rozglaszaj sieci
  {
    WiFi.softAPdisconnect(true);
    connect_status = 0;
    //led_mode = 2;
    tank_check(); 
  }

  //status diody led
      //przywroc stary status LED
    if(connect_status == 1) 
    {
    led_mode = 3; 
    } 
    else if (connect_status == 0)
    {
    led_mode = 2;            
    }

}

//Reads a string out of memory
String read_string(int l, int p){
  String temp;
  for (int n = p; n < l+p; ++n)
    {
     if(char(EEPROM.read(n))!=';'){
      if(isWhitespace(char(EEPROM.read(n)))){
          //do nothing
        }else temp += String(char(EEPROM.read(n)));
      
     }else n=l+p;
     
    }
  return temp;
}  

//Write data to memory
void write_to_Memory(String s,String p){
 s+=";";
 write_EEPROM(s,data_size);
 p+=";";
 write_EEPROM(p,(wifi_size+data_size));
 EEPROM.commit();
}

//write to memory
void write_EEPROM(String x,int pos){
  
  for(int n=pos;n<x.length()+pos;n++){

    if((EEPROM.read(n)) != (x[n-pos]))
    {   
     EEPROM.write(n,x[n-pos]);
    }
  }
} 

//petla zapisu do pamieci eeprom - DO NAPISANIA
void save_EEPROM(void) {
  if (memory_save_active == 1) {

    //zapis do tablicy danych zmiennych z okna
    data_buff[0]=data_wet_set;
    data_buff[1]=data_pump_time_set;
    data_buff[2]=data_pump_cycle_set; 

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
    //dodawanie nazwy wifi i hasla wifi
    write_to_Memory(buff_1_name_text_wifi, buff_1_pass_text_wifi);

    memory_save_active = 0;
    led_syg_short();
  } 
  
  else {
  }
}


//funkcja ładowania danych z pamieci eeprom i przekazywania ich do tablic
void load_EEPROM(void) {
  //wczytaj dane z pamieci eeprom
  //przyporzadkuj dane do sterowania przekaznikami
  for (int i = 0; i <= data_size - 1; i++) {

    buff_memory_1[i] = EEPROM.read(i);    //do bufora danych
    data_buff[i] = buff_memory_1[i]; //do bufora pamieci
    delay(10);                            //opoznienie odczytu    
    
  }

    //przepisanie tablic odczytu do zmiennych 
    data_wet_set=data_buff[0];
    data_pump_time_set=data_buff[1];
    data_pump_cycle_set =data_buff[2];     

//odczyt ustawien wifi
buff_1_name_text_wifi = read_string (wifi_size, data_size);
buff_1_pass_text_wifi = read_string (wifi_size, wifi_size  + data_size);

}

//aktualizacja hasla wifi
void wifi_save(void) {
  if (wifi_set_active == 1) {
    delay(100);
    WiFi.softAPdisconnect(true);

    delay(100);
    WiFi.softAP(buff_name_wifi, buff_pass_wifi);

    delay(100);
    led_syg_short();
    wifi_set_active = 0;
  } else {
  }
}

//reset nazwy - ok
void wifi_reset(void) {
  if (sw_long == 1) {
    WiFi.softAPdisconnect(true);
    delay(100);
    led_syg_short();
    WiFi.softAP(ssid_default, password_default);
    delay(100);
    sw_long = 0;
    sw_short = 1;
    connect_status = 1;
  }
}

//reset urzadzenia - ok
void device_reset (void)
{
  if(sw_reset== 1) {
  led_syg_long();
  delay(100);
  ESP.restart();
  sw_reset= 0;
  }
}

//zatrzymanie urzadzenia
void device_stop (void)
{
  while(sw_stop==1){
  led_syg_short();
  device_reset(); //reset urzadzenia
  digitalWrite(status_led_r_pin, HIGH);
  digitalWrite(status_led_g_pin, HIGH);
  digitalWrite(status_led_b_pin, HIGH);
  digitalWrite(water_level_vcc_pin, LOW);
  digitalWrite(pump_pin, LOW);
  }
  device_reset(); //reset urzadzenia
}

//debug
void debug_serial (void) {

  Serial.println("wifi data jako string ze strony");
  Serial.println(text_name_wifi);
  Serial.println(text_pass_wifi);

  Serial.println("wifi data jako tablica danych");
  Serial.println(buff_name_wifi);
  Serial.println(buff_pass_wifi);

  Serial.println("nadpisz WiFi");
  Serial.println(wifi_set_active);

  Serial.println("nadpisz EEPROM");
  Serial.println(memory_save_active);

  Serial.println("pompa");
  Serial.println(data_pump_work_force);
}

void setup() {

  Serial.begin(115200);

  //HARDWARE

  //inicjalizacja biblioteki easy button
  button.begin();
  button.onPressedFor(3000, LongPress);
  button.onPressed(ShortPress);
  button.onSequence(3, 5000, StopPress);
  button.onSequence(5, 5000, ResetPress);

  if (button.supportsInterrupt()) {
    button.enableInterrupt(buttonISR);
  }

  //konfiguracja portów - wyjścia
  pinMode(status_led_r_pin, OUTPUT);
  pinMode(status_led_g_pin, OUTPUT);
  pinMode(status_led_b_pin, OUTPUT);
  pinMode(water_level_vcc_pin, OUTPUT);
  pinMode(pump_pin, OUTPUT);
  //konfiguracja portów - wejścia
  pinMode(soil_sensor_pin, INPUT);
  pinMode(water_level_pin, INPUT);

  //stany początkowe
  digitalWrite(status_led_r_pin, HIGH);
  digitalWrite(status_led_g_pin, HIGH);
  digitalWrite(status_led_b_pin, HIGH);
  digitalWrite(water_level_vcc_pin, LOW);
  digitalWrite(pump_pin, LOW);

  //EEPROM
  EEPROM.begin(eeprom_size);  //inicjalizacja pamięci EEPROM
  delay(100);
  load_EEPROM();  //ładowanie pamięci EEPROM
  delay(100);

  //konwertowanie danych z eepromu na dane do wczytania sieci wifi
  buff_1_name_text_wifi.toCharArray(buff_name_wifi, wifi_size);
  buff_1_pass_text_wifi.toCharArray(buff_pass_wifi, wifi_size);
  //przypisanie wczytanych danych z eeprom
  soil_min = data_wet_set;                     //minimalna wilgotnosc gleby w %
  pump_time = 1000 * data_pump_time_set;       // czas pracy pompy w ms
  pump_cycle_per_space = data_pump_cycle_set;  //ilosc zadzialan na cykl

  //WIFI
  WiFi.softAP(buff_name_wifi, buff_pass_wifi);
  //WiFi.softAP(ssid_default, password_default);
  IPAddress myIP = WiFi.softAPIP();
  MDNS.begin(mDSN_name, WiFi.softAPIP());
  start_web();  //ustawienie strony internetowej
  MDNS.addService("http", "tcp", 80);
  connect_status = 1;  //ustanowiono polaczenie

  led_syg_long(); //zakomunikuj start
  tank_check(); //sprawdz stan wody przy uruchomieniu 
}

void loop() {

  //odczyt z czujnikow i wejsc
  soil_status(); //odczyt sensora wilgotnosci
  button.update(); //sprawdź status przycisku
  MDNS.update();  //aktualizacja mDNS

  //konwertowanie na strone i do pamieci eeprom
  save_EEPROM(); //zapisz wprowadzone dane do pamieci eeprom
  wifi_save();  //nowe ustawienia wifi
  wifi_reset(); //reset wifi do ustawien początkowych 
  
  device_stop(); //zatrzymanie urzadzenia
  
  //obsluga wyjsc
  auto_pump();    //sterowanie pompa
  network_admin(); //obsluga rozglaszania sieci
  led_work();

  delay(100);
}

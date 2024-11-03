//Adres: 192.168.4.1 

//biblioteki wifi
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

//konfiguracja pinów
#define sw_pin D1 //przelacznik
#define status_led_pin D4 //dioda statusu
#define pump_pin D6 //tranzystor pompy
#define soil_sensor_pin A0  //odczyt wartosci czujnik gleby
#define water_level_pin D2  //poziom wody 

//zmienne obsługi pamieci eeprom
const int EEPROM_START = 0; //numer komorki od ktorej algorytm ma zaczac poszukiwanie 
const int EEPROM_SIZE = 1; //ilosc pamieci w buforze
const int EEPROM_SIZE_CYCLE_MAX = 4000; //zadeklarowana pamieć EEPROM //bylo 100
int val_buff[EEPROM_SIZE]; //bufor wartosci
int eeprom_buff[EEPROM_SIZE]; //bufor pamieci
int i = 0;  //zmienna do rezerwowania miejsc w pamięci mikrokontrolera na dane, 0 oznacza jedno miejsce 
int last_value_eeprom = 0; //funkcja do przeszukiwania pozycji pamięci EEPROM 
int last_id_eeprom =0 ; //zmienna zapisu ostatniej komórki pamięci EEPROM

//zmienne obslugi WiFi Acesspoint
const char *ssid = "AutoSzklarnia_v1";  //nazwa
const char *password = "winogrona"; //haslo
int val_A=0;  //wartosc parametru pierwszej zmiennej
int connect_status = 0; //zmienna statusu polaczenia 

String value_A = "0";
String value_B = "Dostateczna";
String value_1 = "0";
String value_2 = "0";

const char* input_parameter_A = "value";
const char* input_parameter_B = "value";
const char* input_parameter_1 = "set_1_value";
const char* input_parameter_2 = "set_2_value";

String input_message_1;
String input_message_2;


//zmienne hardwarowe do obsługi programu
//-----------------------------------------------------------------------------------------------------------------------------
int soil_min = 0; //minimalna wilgotnosc gleby w % >>-- WARTOSCI DLA WPROWADZANIA --<<
int pump_master_sw = 0; //zmienna do wymuszenia wlaczenia pompy
//-----------------------------------------------------------------------------------------------------------------------------
int S_M_S_value = 0;  //wartość czujnika gleby
int S_M_S_value_raw = 0;  //wartosc czujnika gleby bez przelcizenia
int alarm = 0; //alarm 
int sw_short = 1; //stan przełącznika przy krotkim wciśnięciu
int sw_long = 1; //stan przełącznika przy długim wcisnieciu
int pump_max = 254; //maksymalna moc pompy
unsigned long pump_time = 3000; // czas pracy pompy w ms
int pump_cycle = 0; // cykle pompy na dobe
int status_led = 0; //stan kontrolki statusu
int water_level = 0; //poziom wody
int pump_save = 0;  //zmienna do zapisywania pierwszego cyklu zadzialania pompy 
unsigned long accual_time = 0; //zmienna do obliczania biezacego czasu 
unsigned long first_time_pump = 0;  //zmienna pierwszego cyklu zadzialania
int pump_min_cycle_space = 480; //ilosc minut na jeden cykl 8*60
int pump_cycle_per_space = 3; //ilosc zadzialan na cykl

// instancja biblioteki przelacznikow
EasyButton button(sw_pin) ;

// funkcje przyciskow
void LongPress()
{
  sw_long=!sw_long;
}

void ShortPress()
{
  sw_short=!sw_short;
}

void buttonISR()
{
  button.read();
}

//PĘTLE WŁASNE - HARDWARE

//sprawdzenie ilości wody w zbiorniku
void tank_check (void)
{
  water_level = digitalRead(water_level_pin);//odczyt poziomu wody

  if (water_level != 1) //jesli brak wody to wlacz alarm
  {
    alarm = 1;
  }
  else
  {
    alarm=0;
  }
  
}

//sprawdzanie poziomu wilgotności gleby
void soil_status (void)
{
  int min_moi = 130; //minimalna wilgotnosc gleby - czujnik w powietrzu
  int max_moi = 65; //maksymalna wilgotnosc gleby - czujnik w wodzie

  //obliczanie wartosci do skalowania
  S_M_S_value=analogRead(soil_sensor_pin);
  S_M_S_value_raw=S_M_S_value;
  S_M_S_value=map(S_M_S_value, min_moi ,max_moi, 0, 100);
  S_M_S_value=constrain(S_M_S_value, 0, 100);

}

//obsluga statusu alarmu
void alarm_status (void)
{

  if (alarm !=0)//alarm włączony
    {
  digitalWrite(status_led_pin, HIGH);
  delay(100);
  digitalWrite(status_led_pin, LOW);
  delay(100);      
    }

  else   //alarm wyłączony
    {
  digitalWrite(status_led_pin, HIGH);      
    }
}

//petla obslugi pompy
void auto_pump (void)
{
  if (((S_M_S_value <= val_buff[i]) && (alarm == 0) && (pump_cycle<pump_cycle_per_space)) || (pump_master_sw== 1))  //warunki do wykonania 
  {
    for (int i=0; i>=pump_max; i++) //rozpedz pompe i utrzymaj 5000 sekund
    {
      analogWrite(pump_pin,i);
      delay(1);
    }
    analogWrite(pump_pin, pump_max );
     delay(pump_time);
     
     for (int i=pump_max; i<=0; i--) //wylacz pompe
    {
      analogWrite(pump_pin,i);
      delay(1);
    }
     analogWrite(pump_pin, 0 );

      //jesli cykl nie zostal wymuszony przez strone to zwieksz cykl pompy
     if(pump_master_sw ==0)
     {
     pump_cycle++; //dodaj jeden cykl wlaczenia pompy
     }
     pump_master_sw = 0;
  }
  else
  {
   analogWrite(pump_pin, 0 ); 
  }

  
  //zabezpieczenie przed przelaniem 
   accual_time = millis(); //aktualizacja czasu pracy
  
  //zapisz czas pierwszego cyklu 
  if((pump_cycle == 1) && (pump_save == 0))
  {
    first_time_pump = accual_time;
    pump_save = 1;
  }
  
  //sprawdz warunek czasu
  if(accual_time - first_time_pump >= (pump_min_cycle_space*60*1000))
  {
    pump_cycle = 0;
    pump_save = 0; 
  }
}

//-------------------------------------------------------------------------------------------------------------------------

//strona glowna wifi
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>ASU</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  </head><body>
  <h1>Automatyczna stacja uprawy</h> 
  <h2>Odczyt:</h2> 
  <p>Aktualny poziom wilgotnosci: %FUN_A_VALUE% [&#37;]</p>
  <p>Ilosc wody w zbiorniku: %FUN_B_VALUE% </p> 
  <h2>Zapis:</h2> 
  <form action="/get">
    Minimalny poziom wilgotnosc gleby [&#37;]: <input type="number" step="1" name="set_1_value" value="%FUN_1_VALUE%" min="0" max="100" required><br><br>
    <h2>On/Off</h2> 
    Uruchom pompe jednorazowo <input type="checkbox" name="set_2_value" value="%FUN_2_VALUE%" ><br><br>
    <input type="submit" value="Zapisz">
  </form> 
</body></html>)rawliteral";

//obsluga żadan
void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

AsyncWebServer server(80);

//funkcja przetwarzajaca zmienne
String processor(const String& var){
  //Serial.println(var);
  if(var == "FUN_A_VALUE"){
    return value_A;
  }
  if(var == "FUN_B_VALUE"){
    return value_B;
  }
  else if(var == "FUN_1_VALUE"){
    return value_1;
  }
  
  return String();
}

//zmiana typow zmiennych
void convert(void)
{
 value_A=String(val_A,DEC);
          
}

//zapis do pamieci EEPROM
void save_EEPROM (void)
{
  int upd_adr_id =0;
  int upd_adr_val =0;
  
  int upd_val_id =0;
  int upd_val_val =0;  
  
  if(val_buff[i]!=eeprom_buff[i]) //zapis do pamieci
  {
  eeprom_buff[i]= val_buff[i];  //zapis do bufora awaryjnego
  
  if(last_id_eeprom < (EEPROM_SIZE_CYCLE_MAX-2-i))  //jesli nie doszlismy jeszcze do konca tablic to wykonaj ponizsza instrukcje, wielkosc tablicy liczymy od 0    xxxx<ostatnie 2 miejsca,
  {
  //aktualizacja wartosci adresu
  last_id_eeprom = last_id_eeprom+2+i;
  upd_adr_id = last_id_eeprom; 
  upd_adr_val = EEPROM.read(last_id_eeprom);
  upd_adr_val = upd_adr_val +1;
  //aktualizacja wartosci wartosci 
  upd_val_id = last_id_eeprom+1+i;
  upd_val_val = val_buff[i];
  //Serial.println("nieprzewinieto---------");
  }
  
  else  //jesli doszlismy do konca tablicy to zacznij nadpisywac ja od poczatku
  {
  //aktualizacja wartosci adresu
  upd_adr_id = EEPROM_START; 
  last_id_eeprom = upd_adr_id;
  upd_adr_val = EEPROM.read(EEPROM_START);
  upd_adr_val = upd_adr_val +1;
  
  //aktualizacja wartosci wartosci
  upd_val_id = 1+i;
  upd_val_val = val_buff[i];
  //Serial.println("przewinieto--------------");  
  }

  last_value_eeprom = upd_adr_val; //aktualizacja globalnej zmiennej wartosci adresu

  //STRUKTORA ZAPISU ADRES, WARTOSC
  EEPROM.write(upd_adr_id, upd_adr_val); //adres, wartosc - zapisz najnowszy adres w kolejnej komorce ale dodaj jeden zapis w cyklu
  delay(5); //opoznienie do zapisu 
  EEPROM.commit();//zapisywanie  
  
  EEPROM.write(upd_val_id, upd_val_val); //adres, wartosc - zapisz wartosc przyporzadkowana do adresu
  delay(5); //opoznienie do zapisu
  EEPROM.commit();//zapisywanie 

 }
}


//funkcja ładowania danych z pamieci eeprom - GOTOWE
void load_EEPROM (void)
{ 
  eeprom_buff[i]=EEPROM.read(last_id_eeprom+1+i);   //pamiec z ostatnia zapisana zmienna w komorce obok
  val_buff[i]=eeprom_buff[i];
  
  value_1=String(val_buff[i],DEC);  //zmiana wartosci int na string
}

//funkcja przeszukiwania ostatniego zapisu w EEPROM - POPRAWIC, JEST BLAD 
void load_last_EEPROM (void)
{ 
  int current_id = EEPROM_START;
  int current_value =EEPROM_START;
  
  last_value_eeprom = EEPROM_START;  //przy zalozeniu ze szukamy najwiekszej wartosci, w przypadku szukania najmniejszej nalezy zastosowac zamiast 0 EEPROM_SIZE_CYCLE_1
   
  int first_value_adr=EEPROM.read(EEPROM_START); //bylo wpisane 0
  int last_value_adr=EEPROM.read(EEPROM_SIZE_CYCLE_MAX-2); // bylo wpisane 99 , wartosc ostatniej komorki adresu STRUKTURA PAMIECI: ADRES, WARTOSC

  if(first_value_adr == last_value_adr)  //jesli ostatni zapis rowna sie pierwszemu to zacznij od poczatku 
  {
      Serial.println("marginalny tryb ladowania");
      last_id_eeprom = EEPROM_SIZE_CYCLE_MAX-2;              //wartosc numerku komorki, bylo 99, wartosc ostatniej komorki adresu STRUKTURA PAMIECI: ADRES, WARTOSC
      current_value=EEPROM.read(last_id_eeprom);     //wartosc nadpisan komorki, bylo 99, wartosc ostatniej komorki adresu STRUKTURA PAMIECI: ADRES, WARTOSC
     
     
      Serial.println(last_value_eeprom);
      Serial.println(last_id_eeprom);
      delay(5000);
  }
  else
  {  
   Serial.println("calkowity tryb ladowania");
  for (current_id; current_id <=(EEPROM_SIZE_CYCLE_MAX-2); (current_id = current_id+2+i))  //dla kazdego wyrazu z zadeklarowanej tablicy sprawdz warunek, bylo 98
  {  
   
    current_value = EEPROM.read(current_id);  //odczytaj biezaca wartosc
    delay(5);
            
      if(current_value>=last_value_eeprom)    //znajdz ostatni obszar pamieci o najwiekszej liczbie zapisow 
      {
      last_value_eeprom = current_value;     //zapisz wartosc liczby zapisow komorki 
      last_id_eeprom = current_id;          //zapisz numer ostatniej zapisanej komorki 
      
      Serial.println("wartosc komorki id- ilosc wykonanych zapisow");
      Serial.println(last_value_eeprom);
      Serial.println("numer ostatniej komorki id");
      Serial.println(last_id_eeprom);
      delay(5000);
      }
   }
  }
}

//funkcja obslugi internetu
void start_web(void)
{
    // Send web page to client
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200,  "text/html", index_html, processor);
  });

  // Receive an HTTP GET request at <ESP_IP>/get?threshold_input=<inputMessage>&enable_arm_input=<inputMessage2>
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
    // GET threshold_input value on <ESP_IP>/get?threshold_input=<inputMessage>
    //zapis wartosci granicy wlaczenia czujnika
    if (request->hasParam(input_parameter_1)) {
      input_message_1 = request->getParam(input_parameter_1)->value();
      value_1=input_message_1;
      val_buff[i] = value_1.toInt();   //zapis parametru wilgotnosci ze strony do zmiennej 
    } 
      //zapis wartosci uruchomienia pompy
         if (request->hasParam(input_parameter_2)) {
       input_message_2 = request->getParam(input_parameter_2)->value();
        input_message_2 = "true";
        value_2=input_message_2;
        pump_master_sw = 1; //pompa dziala - pole wyboru zaznaczone 
      }
     else {
        input_message_2 = "false";   //inaczej nie dziala
        value_2=input_message_2;
        pump_master_sw = 0; //pompa nie dziala - pole wybory odznaczone
      } 

    request->send(200, "text/html", "Zapisano<br><a href=\"/\">Powrot</a>");
  });
  server.onNotFound(notFound);
  server.begin();
}

void setup() {
  
  Serial.begin(19200);

  //INIT - HARDWARE
  //konfiguracja portów - wejścia
  pinMode(soil_sensor_pin, INPUT);
  pinMode(water_level_pin, INPUT);

  //inicjalizacja biblioteki easy button
  button.begin();
  //button.onPressedFor(2000, LongPress);
  button.onPressed( ShortPress);
  
  if (button.supportsInterrupt())
  {
    button.enableInterrupt(buttonISR);
  }

  
  //konfiguracja portów - wyjścia
  pinMode(status_led_pin, OUTPUT);
  pinMode(pump_pin, OUTPUT);

  //stany początkowe 
  digitalWrite(status_led_pin, HIGH);
  digitalWrite(pump_pin, LOW);

  delay(200); //opóźnienie startu

  digitalWrite(status_led_pin, LOW);

  //INIT WIFI
  //uruchomienie AP
  WiFi.softAP(ssid, password);
  IPAddress myIP = WiFi.softAPIP();
  start_web(); //ustawienie strony internetowej
  connect_status = 1; //ustanowiono polaczenie 
  
  delay(1000);
  EEPROM.begin(EEPROM_SIZE_CYCLE_MAX); //inicjalizacja pamięci EEPROM
  delay(1000);
  Serial.println("ladowanie pamiec eeprom"); //debug
  load_last_EEPROM(); //wczytaj ostatni zapis pamieci 
  delay(1000);
    
  Serial.println("zaladowano pamiec eeprom 1");
  load_EEPROM();  //ładowanie pamięci EEPROM
  delay(1000);

  Serial.println("zaladowano pamiec eeprom 2"); //debug
  Serial.println("gotowy");
      
}

//petla debugu i wyswietlania wartosci 
void debug_serial (void)
{
    //EEPROM WIFI
    Serial.println("wartosc przechowywana");
    Serial.println(val_buff[i]);
    Serial.println("adres ostatniej zapisanej komorki pamieci");
    Serial.println(last_id_eeprom);    
    Serial.println("ilosc wykonanych cykli pamieci");
    Serial.println(last_value_eeprom);

    //HARDWARE
   Serial.println("przelacznik dlugo");
   Serial.println(sw_long);
   Serial.println("przelacznik krotko");
   Serial.println(sw_short);
   Serial.println("czujnik gleby");
   Serial.println(S_M_S_value);
   Serial.println("alarm");
   Serial.println(alarm);
   Serial.println("cykle pompy");
   Serial.println(pump_cycle);

   Serial.println("status sieci");
   Serial.println(connect_status);

   Serial.println("czujnik gleby surowy");
   Serial.println(S_M_S_value_raw);
}

//petla konwertowania wyswietlanych (odczytywanych) wartosci ze strony na wartosci programu - hardwarowe 
void value_conv (void)
{
  val_A=S_M_S_value; //wartosc czujnika wilgotnosci 

  //wartosc czujnika poziomu wody
  if(water_level ==1)
  {
    value_B = "Dostateczna";
  }
  else
  {
    value_B = "Niedostateczna - dolej wody do zbiornika"; 
  }

  
}

//obsuga wifi z przycisku
void network_admin(void)
{
  if((sw_short == 1) && (connect_status == 0)) //rozglaszaj siec
  {
  WiFi.softAP(ssid, password);
  connect_status = 1;
  led_mode_1();
  
  }
  else if ((sw_short == 0) && (connect_status == 1))  //nie rozglaszaj sieci
  {
   WiFi.softAPdisconnect (true);
   connect_status = 0;
   led_mode_1();
   
  }
}

//dioda powiadomienia - tryb 1
void led_mode_1 (void)
{
  for(int w=0; w<=3; w++)
  {
  digitalWrite(status_led_pin, HIGH);
  delay(50);
  digitalWrite(status_led_pin, LOW);
  delay(50); 
  }
}

//dioda powiadomienia - tryb 2
void led_mode_2 (void)
{
  for(int w=0; w<=3; w++)
  {
  digitalWrite(status_led_pin, HIGH);
  delay(300);
  digitalWrite(status_led_pin, LOW);
  delay(300); 
  }
}



void loop() {

  //odczyt z czujnikow i wejsc
  soil_status(); //odczyt sensora wilgotnosci
  tank_check (); //sprawdź ilość wody w zbiorniku  
  alarm_status(); //sprawdzanie statusu alarmu
  button.update(); //sprawdź status przycisku 

  //konwertowanie na strone i do pamieci eeprom
  value_conv(); //konwertuj wartosci czujnikow na te ze strony
  convert();  //konwertowanie wartosci 
  save_EEPROM(); //zapisz wprowadzone dane do pamieci eeprom
  
  //obsluga wyjsc 
  auto_pump();    //sterowanie pompa 
  network_admin(); //obsluga rozglaszania sieci
  debug_serial(); //WYSWIETLANIE WARTOSCI DEBUGU
  delay(1000);
}

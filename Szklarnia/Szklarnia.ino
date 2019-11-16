
// Wyswietlacz w pwm 2,3,4,5,6,7
// tak jak na kursie: https://simple-circuit.com/arduino-bmp280-sensor-lcd/
// czujnik wilgotności, teperatury i ciśnienia - podlaczony w SDA i SCL - pin 20, 21, oraz zasilanie 5v
//

/*
Podlaczenia arduino do plytki:
pin     |   PIN     |   PIN
Płytka  |   Arduino |   Sprzęt
--------|-----------|-------------------------------------------------------------------------
PIN 23  |   22      |   Osobnym kablem laczyc z ledem  - Dioda Let imitujaca swiatlo - zarowke
        |   23      |   Podlaczenie do elektrozamka poki co osobno
                        Podlaczenie: pin 43 do VCC a pozostałe GND i IN do GND

PIN 35  |   46      |   Mostek H - 7 - wejście określające kierunek obrotów pierwszego silnika
PIN 36  |   47      |   Mostek H - 2 - wejście określające kierunek obrotów pierwszego silnika
PIN 37  |   49      |   Czujnik Poziomu wody - druga czesc czujnika do masy  
PIN 38  |   48      |   RFID - Reset (RFID - RC522)
PIN 39  |   51      |   RFID - MOSI
PIN 40  |   50      |   RFID - MISO
PIN 41  |   53      |   RFID - SDA
PIN 42  |   52      |   RFID - SCK
PIN 43  |   21  SCL |   SCL - Barometr (BME/BMP280) & czujnik Swiatła TSL2561
PIN 44  |   20  SDA |   SDA - Barometr (BME/BMP280) & czujnik Swiatła TSL2561
PIN 45  |   2       |   Wyświetlacz LED - RS - wybór rejestrów (komenda, dane)
PIN 46  |   3       |   Wyświetlacz LED - E - zezwolenie na zapis do rejestrów
PIN 47  |   4       |   Wyświetlacz LED - D4 - dane
PIN 48  |   5       |   Wyświetlacz LED - D5 - dane
PIN 49  |   6       |   Wyświetlacz LED - D6 - dane
PIN 50  |   7       |   Wyświetlacz LED - D7 - dane
PIN 51  |   8   PWM |   Mostek H - 1 - wejście ENABLE określające prędkość pierwszego silnika
PIN 52  |   A0      |   Czujnik wilgotności gleby - A0

dodatkowo Masa, Zasilanie 5V, oraz 3,3V   

*/

/////////////// TODO
//pierwszy if czy jest dostęp karta
// drugi if czy jest woda w zbiorniku jak nie to wyswietlic napisów
// jesli jest woda wyswietlamy dane:
// co 20 sekund sprawdzac czujnik wilgotnosci gleby

#include "Seeed_BME280.h"
#include <Wire.h>
#include <LiquidCrystal.h> // dolaczenie pobranej biblioteki I2C dla LCD
#include "RTClib.h"
#include <Adafruit_TSL2561_U.h>
#include <SPI.h>     //RFID
#include <MFRC522.h> // RFID

LiquidCrystal lcd(2, 3, 4, 5, 6, 7);
//RTC_DS1307 czas; // TODO co to i czy potrzebne

BME280 bme280; // pressure, temperature, altitude, humidityAir
// swiatlo
sensors_event_t event;                                                              // do czujnika światła
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345); // swiatlo chyba

#define SILNIK_PWM 8        //mostek nóżka 1
#define SILNIK_Kierunek1 46 // mostek nóżka 2 lub 7 sprawdzic na pompie który kierunek
#define SILNIK_Kierunek2 47 // mostek nóżka 2 lub 7 sprawdzic na pompie który kierunek
// #define PortCzujkiSwiatla A5
#define PortCzujkiWilgotnosciGleby A0

#define DiodaJakoZarowka 22 // dioda ktora robi za żarowke
#define ElektrozamekPin 43  // Pin d sterowania elektrozamkiem

#define CZUJNIK_WODY_W_ZBIORNIKU (49) // do czujnika czy jest woda w zbiorniku z wodą

#define SS_PIN 53  // RFID
#define RST_PIN 48 // RFID

MFRC522 rfid_mfrc522(SS_PIN, RST_PIN); // Instance of the class for FRID

bool isAuthorized = false;    // TODO usun juz nie uzywne
unsigned long rememberedTime; // = millis();
unsigned long timeForElectroMagneticLock;
bool isElectroMagneticUnLock = false;

unsigned long timeForDisplay;
unsigned long timeOfLastRefreshLed; // ostatnia chwila kiedy odswiezylismy dane na wyswietlaczu

bool isResetDisplay = false;
// unsigned long nowTime;
int counterPWMForPump = 0;
int MAX_VALUE_FOR_COUNTER_PWM = 0;

//
//for test
unsigned long timeTime;

//
char text[14];
float temperature;
float pressure;
float altitude_;
float humidityAir;
float illuminance;
int waterSensor = 0;  // czujnik czy jest woda w zbiorniku do podlania kwiatow by pompa nie chodzila na sucho
float humidityGround; // do zrobienia

void setup()
{
    Serial.begin(9600);

    /// czujnik PortCzujkiSwiatla

    if (!tsl.begin())
    {
        /* There was a problem detecting the TSL2561 ... check your connections */
        Serial.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
        while (1)
            ;
    }
    displayLightSensorDetails();
    ///////////////////
    // czujnik wody
    pinMode(CZUJNIK_WODY_W_ZBIORNIKU, INPUT_PULLUP);

    //
    Wire.begin();
    // czas.begin();
    lcd.begin(20, 4);

    Serial.println(F("Arduino + BMP280"));

    if (!bme280.init())
    {
        Serial.println("Device error!");
    }

    setDisplayConstText();
    // swiatlo

    // pinMode(PortCzujkiSwiatla, INPUT);

    // czujnik gleby
    pinMode(PortCzujkiWilgotnosciGleby, INPUT);

    // silnik
    pinMode(SILNIK_PWM, OUTPUT); //Sygnał PWM silnika nr 1

    pinMode(SILNIK_Kierunek1, OUTPUT); //Sygnały sterujące kierunkiem obrotów silnika nr 1
    pinMode(SILNIK_Kierunek2, OUTPUT);

    pinMode(DiodaJakoZarowka, OUTPUT);
    pinMode(ElektrozamekPin, OUTPUT);
    digitalWrite(ElektrozamekPin, LOW);

    digitalWrite(SILNIK_Kierunek1, LOW); //Silnik nr 1 - obroty w lewo
    digitalWrite(SILNIK_Kierunek2, HIGH);
    timeTime = millis();
    timeForElectroMagneticLock = timeTime;
    timeForDisplay = timeTime;
    timeOfLastRefreshLed = timeTime;

    // RFID
    SPI.begin(); // Init SPI bus
    pinMode(SS_PIN, OUTPUT);
    digitalWrite(SS_PIN, LOW);

    rfid_mfrc522.PCD_Init(); // Init MFRC522
    pinMode(SS_PIN, OUTPUT);
    digitalWrite(SS_PIN, HIGH);

    Serial.println("RFID reading UID");
}

void loop()
{
    // !!! użyć funkci do czasu milisecond tick coś tam
    // i za jjej pomoasprawdzac czy zwiekrzyła sie wartosśc o 100
    // jeśli tak to wykonujemy zdarzenie uruchamiay pompke i (zwiekrajac o 100 czas nastepnej kontroli)
    // hd 44780 - sterownik wyswetlacza hitachi
    // - jedna strona jak sie to urządzenie programuje
    // na 8 bitahc ustawionea jest jedna jedynka
    //ik\jest kilka rozkazów
    //rozkazz numer 1 ----
    // jak ustaw
    // dokumentacja https://www.sparkfun.com/datasheets/LCD/HD44780.pdf str 40
    //
    //vczujnik wilgotnosci
    // wsadzic do suchej ziemi i do błota i mamy wterdy 2 skrajne warości i możemy wtedy zrobić preskalowanie na wilgotność gleby
    //
    //

    // if (!isAuthorized)
    // {
    //     NapisStartowy();
    //     SprawdzRFID();
    // }
    // else
    {

        //light
        getIlluminance();
        getDataIsWaterTankFull();
        getDataFromBme280();
        getHumidityGround();

        if (!waterSensor)
        {
            NapisBrakWody();
        }

        // gdy wilgotnosc gleby poniżej 40 % wlacz pompe i podlej gdy za dyza wylacz pompe
        if (humidityGround < 46.0)
        {
            startPomp();
        }
        if (humidityGround > 48.0)
        {
            stopPomp();
        }

        // Obsluga żarówki
        // TODO ustawic progowe swiecenia zerowki(diody) empirycznie

        if (illuminance < 40.0)
        {
            digitalWrite(DiodaJakoZarowka, HIGH);
        }
        else
        {
            digitalWrite(DiodaJakoZarowka, LOW);
        }

        // tylko do testow do wolniejszego wyswietlania potem skasuj nnow i if oraz zmienna timeTime
        // jak skasuje to to odkomentuj ShowDataDisplay()
        // unsigned long nnow = millis();
        // if ((nnow - timeTime) > 400UL)
        // {
        //     sprintf(text, "%u%%", (int)(humidityGround));
        //     lcd.setCursor(9, 2);
        //     lcd.print(text);
        //     Serial.print("humidityGround ");
        //     Serial.println(humidityGround);
        //     timeTime = nnow;

        //     sprintf(text, "%u", (int)(counterPWMForPump));
        //     lcd.setCursor(9, 3);
        //     lcd.print(text);
        // }
        // analogWrite(SILNIK_PWM, counterPWMForPump);
        // Serial.print("pwm : ");
        // Serial.println(counterPWMForPump);
        // resetuje wyswietlacz po zmianach na niezmiennych
        if (isResetDisplay)
        {
            if ((millis() - timeForDisplay) >= 5000UL)
            {
                isResetDisplay = false;
                setDisplayConstText();
            }
        }

        if (!isResetDisplay)
        {
            unsigned long now = millis();
            if ((now - timeOfLastRefreshLed) >= 1000UL)
            {
                timeOfLastRefreshLed = now;
                ShowDataDisplay();
            }
        }
        // Serial.println(isResetDisplay);

        //ShowDataConsol();

        SprawdzRFID();
        if (isElectroMagneticUnLock)
        {
            if (millis() - timeForElectroMagneticLock >= 5000UL)
            {
                closeLock();
            }
        }
    }
    delay(100); // wait 2 seconds
}

void NapiszPrzywitanie(const char *uzytkownik)
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Dzien dobry");

    sprintf(text, "%s", uzytkownik);
    lcd.setCursor(1, 2);
    lcd.print(text);
    timeForDisplay = millis();
    isResetDisplay = true;

    //delay(4000); // wait 2 seconds TODO WYWAL czekanie i uruchom program
    //setDisplayConstText();
}

void NapiszBrakAutoryzacji()
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Brak uprawnien");
    lcd.setCursor(5, 1);
    lcd.print("Do wejscia");
    timeForDisplay = millis();
    isResetDisplay = true;

    //delay(2000); // wait 2 seconds
    // NapisStartowy();
}

void NapisBrakWody()
{
    if (!isResetDisplay)
    {
        lcd.clear();
        lcd.setCursor(0, 1);
        lcd.print("Brak wody");
        lcd.setCursor(7, 2);
        lcd.print("w zbiorniku");
        timeForDisplay = millis();
        isResetDisplay = true;
    }
}

void NapisStartowy()
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Przyloz karte");
}

bool SprawdzRFID()
{
    digitalWrite(SS_PIN, LOW);
    bool isInDatabase = false;
    if (rfid_mfrc522.PICC_IsNewCardPresent())
    {
        if (rfid_mfrc522.PICC_ReadCardSerial())
        {
            // wypiswanie danych karty na port szeregowy
            // Serial.print("Tag UID:");
            // for (byte i = 0; i < rfid_mfrc522.uid.size; i++)
            // {
            //     Serial.print(rfid_mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
            //     Serial.print(rfid_mfrc522.uid.uidByte[i], HEX);
            // }
            // wypiswanie danych karty na port szeregowy

            isInDatabase = PorownajKarteZBaza(rfid_mfrc522.uid.uidByte);
            if (isInDatabase)
            {
                openLock();
            }
            Serial.println();
            rfid_mfrc522.PICC_HaltA();
        }
    }
    digitalWrite(SS_PIN, HIGH);
    return isInDatabase;
}

void openLock()
{
    digitalWrite(ElektrozamekPin, HIGH);
    timeForElectroMagneticLock = millis();
    isElectroMagneticUnLock = true;
}

void closeLock()
{
    digitalWrite(ElektrozamekPin, LOW);
    isElectroMagneticUnLock = false;
}

bool PorownajKarteZBaza(uint8_t karta[])
{
    // baza uzykownikow i ich kart
    uint8_t user1[10] = {225, 108, 146, 50};
    uint8_t user2[10] = {253, 34, 125, 137};
    uint8_t user3[10] = {201, 66, 169, 72};

    if (user1[0] == karta[0])
    {
        if (user1[1] == karta[1])
        {
            if (user1[2] == karta[2])
            {
                if (user1[3] == karta[3])
                {
                    Serial.print(" Zarejestrowano: Uzytkownik 1 ");
                    const char *user = "Uzytkownik 1";
                    NapiszPrzywitanie(user);
                    isAuthorized = true;
                    return true;
                }
            }
        }
    }
    else

        if (user2[0] == karta[0])
    {
        if (user2[1] == karta[1])
        {
            if (user2[2] == karta[2])
            {
                if (user2[3] == karta[3])
                {
                    Serial.print(" Zarejestrowano: Uzytkownik 2 ");
                    const char *user = "Uzytkownik 2";
                    NapiszPrzywitanie(user);
                    isAuthorized = true;
                    return true;
                }
            }
        }
    }
    else

        if (user3[0] == karta[0])
    {
        if (user3[1] == karta[1])
        {
            if (user3[2] == karta[2])
            {
                if (user3[3] == karta[3])
                {
                    Serial.print(" Zarejestrowano: Uzytkownik 3 ");
                    const char *user = "Uzytkownik 3";
                    NapiszPrzywitanie(user);
                    isAuthorized = true;
                    return true;
                }
            }
        }
    }

    Serial.print(" Nie ma uzytkownika w bazie ");
    NapiszBrakAutoryzacji();
    return false;
}

void startPomp()
{
    // TO DO tylko do testow zmieniam by szybciej ruszylo potem usun tego if
    if (counterPWMForPump == 0)
    {
        //counterPWMForPump = 130;
    }
    //

    if (counterPWMForPump < MAX_VALUE_FOR_COUNTER_PWM)
    {
        unsigned long now = millis();
        if ((now - rememberedTime) >= 5UL) // ważne żeby z dopikiem UL - bo unsigned long
        {
            rememberedTime = now;
            counterPWMForPump += 5;
            if (counterPWMForPump > MAX_VALUE_FOR_COUNTER_PWM)
            {
                counterPWMForPump = MAX_VALUE_FOR_COUNTER_PWM;
            }
            analogWrite(SILNIK_PWM, counterPWMForPump); //Spokojne rozpędzanie silnika
        }
    }
}

void stopPomp()
{
    //if (counterPWMForPump == 255)
    //{
    counterPWMForPump = 0;
    analogWrite(SILNIK_PWM, counterPWMForPump);
    Serial.println("ZATRZYMUJE POMPE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    //}
}

void getDataIsWaterTankFull()
{
    waterSensor = digitalRead(CZUJNIK_WODY_W_ZBIORNIKU);
    ///return static_cast<bool>(waterSensor);
}

void getHumidityGround()
{
    float value = analogRead(PortCzujkiWilgotnosciGleby);
    Serial.println(value);
    humidityGround = calculateHumidityGround(value);

    // gdy poza zakresem tak na wszelki wypadek gdyby zle pomiary lub gdy wyjdzie za zakres
    if (humidityGround < 0)
    {
        humidityGround = 0;
    }
    else if (humidityGround > 100)
    {
        humidityGround = 100;
    }
}

float calculateHumidityGround(float value)
{
    // można za pomocą funkcji arduino :
    // map(value, minvalue, maxvalue, minoutput, maxoutput)
    // gotowa funja z arduino ktora obliczy procent wilgotnosci
    // http://tinybrd.pl/czujnik_wigloci/

    // bez podlaczenia czegokolwiek wynik jest 262 :-)
    // min na Ani czujniku 479 min na moim 642
    // max na Ani czujniku 324 max na moim 427

    // na Ani czujniku wzór jest:
    //float returnValue = ((-100 * value) + 47900) / 155.0;

    // na moim czujniku wzór jest :
    // humidityGround = (( -100 * value) + 64200) / 255.0;

    // gdy dla testow samo powietrze i woda:
    // min - 1026
    // max - 300
    float returnValue = ((-100 * value) + 102600) / 726.0;

    return returnValue;
}

void setDisplayConstText()
{ // wyswietlnie stalych niezmiennych napisów
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp:");
    lcd.setCursor(0, 1);
    lcd.print("Pres:");
    lcd.setCursor(0, 2);
    lcd.print("Humidity:");
}

void displayLightSensorDetails(void)
{
    sensor_t sensor;
    tsl.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print("Sensor:       ");
    Serial.println(sensor.name);
    Serial.print("Driver Ver:   ");
    Serial.println(sensor.version);
    Serial.print("Unique ID:    ");
    Serial.println(sensor.sensor_id);
    Serial.print("Max Value:    ");
    Serial.print(sensor.max_value);
    Serial.println(" lux");
    Serial.print("Min Value:    ");
    Serial.print(sensor.min_value);
    Serial.println(" lux");
    Serial.print("Resolution:   ");
    Serial.print(sensor.resolution);
    Serial.println(" lux");
    Serial.println();
    Serial.println("------------------------------------");
    Serial.println("");
    delay(500);
}

void getDataFromBme280()
{
    // get temperature, pressure and altitude from library
    temperature = bme280.getTemperature();     // get temperature
    pressure = bme280.getPressure();           // get pressure
    altitude_ = bme280.calcAltitude(pressure); // get altitude (this should be adjusted to your local forecast)
    humidityAir = bme280.getHumidity();
}

void getIlluminance()
{
    /* Get a new sensor event */
    // czujnik swiatla
    tsl.getEvent(&event);

    /* Display the results (light is measured in lux) */
    if (event.light)
    {
        illuminance = event.light;
    }
    else
    {
        /* If event.light = 0 lux the sensor is probably saturated
       and no reliable data could be generated! */
        Serial.println("Sensor overload");
    }
}

void ShowDataDisplay()
{
    // print data on the LCD screen
    // 1: print temperature
    sprintf(text, "%d.%02u%cC  ", (int)temperature, (int)(temperature * 100) % 100, 223);
    lcd.setCursor(5, 0);
    lcd.print(text);
    // 2: print pressure
    sprintf(text, "%u.%02u hPa ", (int)(pressure / 100), (int)((uint32_t)pressure % 100));
    lcd.setCursor(5, 1);
    lcd.print(text);

    // 3: print humidity of Air
    sprintf(text, "%u%%", (int)(humidityAir));
    lcd.setCursor(9, 2);
    lcd.print(text);

    // na wyswietlaczy wyswietla hi lub lo jak jest lub nie ma wody
    if (waterSensor == LOW)
    {
        sprintf(text, "Lo");
        lcd.setCursor(0, 3);
        lcd.print(text);
    }
    else
    {
        sprintf(text, "HI ");
        lcd.setCursor(0, 3);
        lcd.print(text);
    }

    sprintf(text, "%uLux", (int)(illuminance));
    lcd.setCursor(3, 3);
    lcd.print(text);
}

void ShowDataConsol()
{
    Serial.println();

    Serial.print("Czujnik wody w zbbiorniku = ");
    Serial.print(waterSensor);

    Serial.println();
    // print data on the serial monitor software
    // 1: print temperature
    Serial.print("Temperature = ");
    Serial.print(temperature);
    Serial.println(" °C");
    // 2: print pressure
    Serial.print("Pressure    = ");
    Serial.print(pressure / 100);
    Serial.println(" hPa");
    // 3: print altitude
    Serial.print("Approx Altitude = ");
    Serial.print(altitude_);
    Serial.println(" m");

    Serial.print("Humidity of Air: ");
    Serial.print(humidityAir);
    Serial.println("%");

    Serial.print("humidityGround: ");
    Serial.print(humidityGround);
    Serial.println("%");

    Serial.print(illuminance);
    Serial.println(" lux");

    //Serial.println(analogRead(PortCzujkiSwiatla));
    Serial.println("...");
    Serial.println("...");

    Serial.println(); // start a new line
}

//TODO
//1. sprawdzic pomiary gleby w suchej i elnej wody doniczce
//2. Srawdzić max i mnimum wartosci w czujniku światła
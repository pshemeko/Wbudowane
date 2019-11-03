
// Wyswietlacz w pwm 2,3,4,5,6,7
// tak jak na kursie: https://simple-circuit.com/arduino-bmp280-sensor-lcd/
// czujnik wilgotności, teperatury i ciśnienia - podlaczony w SDA i SCL - pin 20, 21, oraz zasilanie 5v
//

#include "Seeed_BME280.h"
#include <Wire.h>
#include <LiquidCrystal.h> // dolaczenie pobranej biblioteki I2C dla LCD
#include "RTClib.h"
#include <Adafruit_TSL2561_U.h>

LiquidCrystal lcd(2, 3, 4, 5, 6, 7);
RTC_DS1307 czas;

BME280 bme280;
// swiatlo
sensors_event_t event; // do czujnika światła
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

#define SILNIK_PWM 10       //mostek nóżka 1
#define SILNIK_Kierunek1 22 // mostek nóżka 2 lub 7 sprawdzic na pompie który kierunek
#define SILNIK_Kierunek2 23 // mostek nóżka 2 lub 7 sprawdzic na pompie który kierunek
#define PortCzujkiSwiatla A5
#define PortCzujkiWilgotnosciGleby A0

#define CZUJNIK_WODY_W_ZBIORNIKU (40) // do czujnika czy jest woda w zbiorniku z wodą

unsigned long rememberedTime; // = millis();
// unsigned long nowTime;
int counterPWMForPump = 0;
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
    czas.begin();
    lcd.begin(20, 4);

    Serial.println(F("Arduino + BMP280"));

    if (!bme280.init())
    {
        Serial.println("Device error!");
    }

    setDisplayConstText();
    // swiatlo

    pinMode(PortCzujkiSwiatla, INPUT);

    // czujnik gleby
    pinMode(PortCzujkiWilgotnosciGleby, INPUT);

    // silnik
    pinMode(SILNIK_PWM, OUTPUT); //Sygnał PWM silnika nr 1

    pinMode(SILNIK_Kierunek1, OUTPUT); //Sygnały sterujące kierunkiem obrotów silnika nr 1
    pinMode(SILNIK_Kierunek2, OUTPUT);

    digitalWrite(SILNIK_Kierunek1, LOW); //Silnik nr 1 - obroty w lewo
    digitalWrite(SILNIK_Kierunek2, HIGH);
    timeTime = millis();
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

    //light
    getIlluminance();
    getDataIsWaterTankFull();
    getDataFromBme280();
    getHumidityGround();

    // gdy wilgotnosc gleby poniżej 40 % wlacz pompe i podlej gdy za dyza wylacz pompe
    if (humidityGround < 40.0)
    {
        startPomp();
    }
    if (humidityGround > 60.0)
    {
        stopPomp();
    }

    // tylko do testow do wolniejszego wyswietlania potem skasuj nnow i if oraz zmienna timeTime
    // jak skasuje to to odkomentuj ShowDataDisplay()
    unsigned long nnow = millis();
    if ((nnow - timeTime) > 400UL)
    {
        sprintf(text, "%u%%", (int)(humidityGround));
        lcd.setCursor(9, 2);
        lcd.print(text);
        Serial.print("humidityGround ");
        Serial.println(humidityGround);
        timeTime = nnow;
    }

    //ShowDataDisplay();

    delay(1000); // wait 2 seconds
}

void startPomp()
{
    unsigned long now = millis();
    // TO DO tylko do testow zmieniam by szybciej ruszylo potem usun tego if
    if (counterPWMForPump == 0)
    {
        counterPWMForPump = 170;
    }
    //

    if (counterPWMForPump < 255)
    {
        if ((now - rememberedTime) >= 50UL) // ważne żeby z dopikiem UL - bo unsigned long
        {
            rememberedTime = now;
            counterPWMForPump += 5;
            if (counterPWMForPump > 255)
            {
                counterPWMForPump = 255;
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

bool getDataIsWaterTankFull()
{
    waterSensor = digitalRead(CZUJNIK_WODY_W_ZBIORNIKU);
    return static_cast<bool>(waterSensor);
}

void getHumidityGround()
{
    float value = analogRead(PortCzujkiWilgotnosciGleby);

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
    Serial.print("humidityGround");
    Serial.print(humidityGround);
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

    ShowDataConsol();
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
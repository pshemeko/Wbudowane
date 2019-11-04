#include <SPI.h>
#include <MFRC522.h>
#define SS_PIN 53
#define RST_PIN 49

MFRC522 mfrc522(SS_PIN, RST_PIN); // Instance of the class
void setup()
{
    Serial.begin(9600);
    SPI.begin();        // Init SPI bus
    mfrc522.PCD_Init(); // Init MFRC522
    Serial.println("RFID reading UID");
}
void loop()
{
    if (mfrc522.PICC_IsNewCardPresent())
    {
        if (mfrc522.PICC_ReadCardSerial())
        {
            Serial.print("Tag UID:");
            for (byte i = 0; i < mfrc522.uid.size; i++)
            {
                Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
                Serial.print(mfrc522.uid.uidByte[i], HEX);
            }
            Serial.print("dupa: ");
            Serial.println(mfrc522.sensor_id);
            mfrc522.Uid.uidByte;
            Serial.println();
            mfrc522.PICC_HaltA();
        }
    }
}
/*
 * Numery urzadzen co mam
karta -> Tag UID: FD 22 7D 89
karta -> Tag UID: E1 6C 92 32
brelok -> Tag UID: C9 42 A9 48
*/
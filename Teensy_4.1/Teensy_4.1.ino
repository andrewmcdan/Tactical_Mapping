// Things to do:
// need to implement reading / writing  from / to the feather
// need to implement a user interface on the OLED
// Need to implement the encoder
// All implementations need to be non-blocking
// need to communicate with the GPS
// if the GPS is not available, use bluetooth
// need to implement the bluetooth

// 1.0.0 - Initial release

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GPS.h>
#include <TimeLib.h>
#include <Timezone.h>
#include <Encoder.h>
// adafruit bluefruit library
#include <Adafruit_BLE.h>
#include <Adafruit_BluefruitLE_UART.h>


#define LED 13
#define OLED_SDA 0
#define OLED_SCL 0

#define GPS_RX 21
#define GPS_TX 20
#define GPS_RST 18
#define GPS_INT 17

#define ENCODER_A 14
#define ENCODER_B 15
#define ENCODER_SW 16

#define BT_RX 22
#define BT_TX 23
#define BT_RST 25
#define BT_INT 26

#define FEATHER_IRQ_IN 12
#define FEATHER_IRQ_OUT 11 

SSD1306 display(128, 64, &Wire, OLED_SDA, OLED_SCL);
Adafruit_GPS GPS(&Serial1);
Timezone tz;

BLESerial bleSerial(BT_RX, BT_TX, BT_RST, BT_INT);

Encoder encoder(ENCODER_A, ENCODER_B);

setup(){
    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH);
    pinMode(GPS_RST, OUTPUT);
    digitalWrite(GPS_RST, HIGH);
    pinMode(GPS_INT, INPUT);
    pinMode(ENCODER_SW, INPUT);
    digitalWrite(ENCODER_SW, HIGH);
    pinMode(BT_RST, OUTPUT);
    digitalWrite(BT_RST, HIGH);
    pinMode(BT_INT, INPUT);
    pinMode(FEATHER_IRQ_IN, INPUT);
    pinMode(FEATHER_IRQ_OUT, OUTPUT);
    digitalWrite(FEATHER_IRQ_OUT, HIGH);
    // initialize the serial ports
    Serial.begin(115200);
    Serial1.begin(9600);
    Serial2.begin(9600);
    Serial.println("Starting...");
    Serial.println("Done.");
    digitalWrite(LED, LOW);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.println("Starting...");
    display.display();
    Serial1.begin(9600);
    GPS.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    GPS.sendCommand(PGCMD_ANTENNA);
    tz.setLocation(F("America/New_York"));
    bleSerial.setDeviceName("Teensy_4.1");
    bleSerial.begin();
    Serial.println("Done.");
    digitalWrite(LED, HIGH);
}

loop(){
    digitalWrite(LED, HIGH);
    delay(1000);
    digitalWrite(LED, LOW);
    delay(1000);
}


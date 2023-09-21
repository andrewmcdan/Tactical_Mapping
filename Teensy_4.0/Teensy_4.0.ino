// Things to do:
// need to implement reading / writing  from / to the feather
// need to implement a user interface on the OLED
// Need to implement the encoder
// All implementations need to be non-blocking
// need to communicate with the GPS
// if the GPS is not available, use bluetooth
// need to implement the bluetooth

// 1.0.0 - Initial release

//#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GPS.h>
#include "QuadEncoder.h"
// adafruit bluefruit library
#include <Adafruit_BLE.h>
#include <Adafruit_BluefruitLE_UART.h>
#include <Bounce2.h>
#include <climits>

#define MODULE_STARTUP_WAIT_TIME 5

#define LED_PIN 13

















#define OLED_SCREEN_WIDTH 128
#define OLED_SCREEN_HEIGHT 32
#define OLED_RESET -1
#define OLED_ADDRESS 0x3C

#define ENCODER_A_PIN 4
#define ENCODER_B_PIN 5
#define ENCODER_SW_PIN 6

#define GPS_EN_PIN 3
#define GPS_FIX_PIN 2

#define BT_MODE_PIN 23

#define FEATHER_IRQ_IN_PIN 10
#define FEATHER_IRQ_OUT_PIN 11
#define FeatherSerial Serial2

#define DEBUG_ENABLE_PIN 22

Adafruit_SSD1306 display(OLED_SCREEN_WIDTH, OLED_SCREEN_HEIGHT, &Wire, OLED_RESET); // OLED display
Adafruit_GPS GPS(&Serial1); // GPS module
Adafruit_BluefruitLE_UART bleSerial(Serial3, BT_MODE_PIN); // bluetooth module
QuadEncoder encoder(1, ENCODER_A_PIN, ENCODER_B_PIN, 1); // encoder using encoder object 1 on pins 4 and 5 with pullup
Button encoderButton = Button(); // encoder button

bool GPS_available = false; // this is the flag that determines if the GPS is available or not
bool BT_available = false; // this is the flag that determines if the bluetooth is available or not
//TODO: remove debug flag
bool debugEnabled = false; // this is the flag that determines if debug mode is enabled or not

uint8_t thisDeviceAddress = 0; // this is the address of this device
String displayTextBuffer[4]; // this is the buffer for the display text
uint8_t GPSserialBuffer[512]; // this is the buffer for the GPS serial data
uint8_t FeatherSerialBuffer[256]; // this is the buffer for the feather serial data
uint32_t myID; // 
long encoderPosition = 0;

class GPSLocation {
public:
    int32_t latitude;
    int32_t longitude;
    unsigned long age;

    GPSLocation() {
        age = millis();
    }

    void updateAge() {
        age = millis();
    }

    void updateLocation(int32_t lat, int32_t lon) {
        latitude = lat;
        longitude = lon;
        updateAge();
    }
};

GPSLocation myLocation;

void setup() {
    randomSeed(analogRead(0));
    myID = random(0, uint32_t(-1));
    // initialize the IRQ pins
    pinMode(FEATHER_IRQ_IN_PIN, INPUT);
    pinMode(FEATHER_IRQ_OUT_PIN, OUTPUT);
    digitalWrite(FEATHER_IRQ_OUT_PIN, HIGH);
    // Give the other modules time to start up
    pinMode(LED_PIN, OUTPUT);
    uint8_t waitCount_ = 0;
    // Give the other modules time to start up
    while (waitCount_ < MODULE_STARTUP_WAIT_TIME) {
        digitalWrite(LED_PIN, HIGH);
        delay(500);
        digitalWrite(LED_PIN, LOW);
        delay(500);
        waitCount_++;
    }
    // clear the displayTextBuffer
    displayTextBuffer[0] = "";
    displayTextBuffer[1] = "";
    displayTextBuffer[2] = "";
    displayTextBuffer[3] = "";
    // Setup and read the debug enable pin
    pinMode(DEBUG_ENABLE_PIN, INPUT_PULLUP);
    if (digitalRead(DEBUG_ENABLE_PIN) == LOW) {
        debugEnabled = true;
    }
    // initialize the serial port
    if (debugEnabled) {
        Serial.begin(115200);
        while (!Serial) delay(1); // wait for serial port to connect.
        Serial.println("Booting...");
    }
    // initialize the feather serial port
    FeatherSerial.addMemoryForRead(FeatherSerialBuffer, sizeof(FeatherSerialBuffer));
    FeatherSerial.begin(115200);
    // initialize the OLED
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
        Serial.println("SSD1306 allocation failed");
    }
    // setup the display
    display.setTextWrap(false);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    // print the startup message
    printTextToDisplay("Starting...");
    delay(150);
    // initialize the pins
    printTextToDisplay("Initializing pins...");
    delay(150);
    digitalWrite(LED_PIN, HIGH);
    pinMode(ENCODER_SW_PIN, INPUT_PULLUP);
    // pinMode(ENCODER_A_PIN, INPUT_PULLUP);
    // pinMode(ENCODER_B_PIN, INPUT_PULLUP);
    encoder.setInitConfig();  //
    //encoder.EncConfig.revolutionCountCondition = DISABLE;
    //encoder.EncConfig.enableModuloCountMode = ENABLE;
    //encoder.EncConfig.positionModulusValue = 4; 
    encoder.init();
    
    pinMode(GPS_EN_PIN, OUTPUT);
    digitalWrite(GPS_EN_PIN, HIGH);
    pinMode(GPS_FIX_PIN, INPUT);
    digitalWrite(LED_PIN, LOW);
    encoderButton.attach(ENCODER_SW_PIN, INPUT_PULLUP);
    encoderButton.interval(5);
    encoderButton.setPressedState(LOW);
    printTextToDisplay("Pins initialized.");
    delay(150);

    // initialize the GPS
    printTextToDisplay("Initializing GPS...");
    delay(150);
    Serial1.addMemoryForRead(GPSserialBuffer, sizeof(GPSserialBuffer)); // 512 byte buffer for incoming GPS data
    GPS.begin(9600);

    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_GLLONLY); // only output GLL sentences (location)
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
    GPS.sendCommand(PGCMD_ANTENNA); // request for updates on antenna status
    printTextToDisplay("Waiting for GPS...");
    delay(1000);
    if (GPS.available()) {
        GPS_available = true;
        printTextToDisplay("GPS available.");
        Serial.println("GPS available.");
    } else {
        printTextToDisplay("GPS not available.");
        Serial.println("GPS not available.");
    }
    printTextToDisplay("GPS done.");
    //delay(1000);

    // initialize the bluetooth
    printTextToDisplay("Initializing Bluetooth...");
    if (bleSerial.begin(true)) {
        BT_available = true;
        printTextToDisplay("Bluetooth available.");
        Serial.println("Bluetooth available.");
        bleSerial.echo(false);
        Serial.println("Requesting Bluefruit info:");
        printTextToDisplay("Requesting Bluefruit info...");
        bleSerial.info();
    } else {
        printTextToDisplay("Bluetooth not available.");
        Serial.println("Bluetooth not available.");
    }
    printTextToDisplay("Bluetooth done.");
    delay(1000);
    // wait for the Feather to boot
    Serial.println("Give the Feather time to startup...");
    printTextToDisplay("Feather booting...");
    //delay(1000);
    bool featherBooted = false;
    while (!featherBooted) {
        printTextToDisplay("Waiting for Feather...");
        Serial.println("Waiting for Feather to boot...");
        digitalWrite(FEATHER_IRQ_OUT_PIN, LOW);
        delay(100);
        uint16_t timeout = 0;
        while (digitalRead(FEATHER_IRQ_IN_PIN) == HIGH) {
            timeout++;
            delay(1);
            if (timeout > 15000) {
                Serial.println("Timeout waiting for Feather to boot.");
                printTextToDisplay("Timeout waiting for");
                printTextToDisplay("Feather to boot.");
                break;
            }
        }
        digitalWrite(FEATHER_IRQ_OUT_PIN, HIGH);
        delay(1000);
        // wait for Feather to send startup message
        timeout = 0;
        while (!FeatherSerial.available()) {
            timeout++;
            delay(1);
            if (timeout > 5000) {
                Serial.println("Timeout waiting for Feather to boot.");
                printTextToDisplay("Timeout waiting for");
                printTextToDisplay("Feather to boot.");
                break;
            }
        }
        String messageFromFeather = FeatherSerial.readStringUntil('\n');
        // parse the message from the Feather
        if (messageFromFeather.startsWith("addr=")) {
            thisDeviceAddress = messageFromFeather.substring(5).toInt();
            Serial.print("This device address: ");
            Serial.println(thisDeviceAddress);
            printTextToDisplay("Address: " + String(thisDeviceAddress));
            featherBooted = true;
        } else if (messageFromFeather.startsWith("x")) {
            Serial.print("Error from Feather: ");
            Serial.println(messageFromFeather.substring(6));
            printTextToDisplay("Error: " + messageFromFeather.substring(6));
            printTextToDisplay("Shutting down.");
            Serial.println("Shutting down.");
            while (true) {
                delay(1000);
            }
        } else if (messageFromFeather.length() == 0) {
            Serial.println("No message from Feather.");
            printTextToDisplay("No message from Feather.");
        } else {
            Serial.print("Unknown message from Feather: ");
            Serial.println(messageFromFeather);
            printTextToDisplay("Unknown message: " + messageFromFeather);
        }
    }
    delay(100);

    Serial.println("Startup done.");
    printTextToDisplay("Startup done.");
    digitalWrite(LED_PIN, HIGH);
}

void loop() {
    // digitalWrite(LED_PIN, HIGH);
    // delay(1500);
    // digitalWrite(LED_PIN, LOW);
    // delay(1000);
    //printTextToDisplay("Looping...");
    // TODO: handle GPS updates
    // This section may be complete?
    if (GPS_available) {
        if (GPS.available())GPS.read();
        if (GPS.newNMEAreceived()) {
            printTextToDisplay("New NMEA received.");
            GPS.parse(GPS.lastNMEA());
            Serial.print("Fix: ");
            Serial.println((int) GPS.fix);
            if (GPS.fix) {
                printTextToDisplay("GPS fix.");
                Serial.print("Location : ");
                Serial.print(GPS.latitude_fixed / 10000000.0, 8);
                Serial.print(", ");
                Serial.println(GPS.longitude_fixed / 10000000.0, 8);
                printTextToDisplay("Updating myLocation.");
                myLocation.updateLocation(GPS.latitude_fixed, GPS.longitude_fixed); // update myLocation with new GPS data
            }
        }
    }
    // TODO: handle bluetooth updates
    if (BT_available) {

    }
    // TODO: handle encoder updates
    long newEncoderPosition = encoder.read();
    if (newEncoderPosition != encoderPosition) {
        if (abs(newEncoderPosition) - abs(encoderPosition) > 1) {
            printTextToDisplay("Encoder update.");
            Serial.print("Encoder position: ");
            Serial.println(newEncoderPosition);
            if (newEncoderPosition > encoderPosition) {
                // encoder moved clockwise
                // TODO: call menu up function
                printTextToDisplay("Encoder moved clockwise.");
                encoder.write(0);
            } else if (newEncoderPosition < encoderPosition || (newEncoderPosition == LONG_MAX && encoderPosition == LONG_MIN)) {
                // encoder moved counterclockwise
                // TODO: call menu down function
                printTextToDisplay("Encoder moved counterclockwise.");
                encoder.write(0);
            }
            encoderPosition = newEncoderPosition = 0;
        }
    }

    encoderButton.update();
    if (encoderButton.pressed()) {
        // TODO: call menu select function
        printTextToDisplay("Encoder button pressed.");
    }

    // TODO: handle data from Feather
    // create a static char array to hold data coming from the Feather
    static char messageFromFeather[512];
    static uint16_t messageFromFeatherLen = 0;
    while(FeatherSerial.available()){
        messageFromFeather[messageFromFeatherLen++] = FeatherSerial.read();
        if(messageFromFeatherLen==512)messageFromFeatherLen = 0;
    }
    if(messageFromFeather[messageFromFeatherLen - 1] == '\n'){
        Serial.print("Message from Feather: ");
        messageFromFeather[messageFromFeatherLen - 1] = '\0';
        Serial.println(String(messageFromFeather));
        messageFromFeatherLen = 0;
        // TODO: parse the message from the Feather
        uint16_t index = 0;
        String mesType = "";
        while(messageFromFeather[index] != ':'){
            mesType += messageFromFeather[index++];
        }
        index++; // skip the colon
        if(mesType.startsWith("vbat")){
            // message is a battery voltage message
            // TODO: parse the battery voltage message
            float vbat = String(messageFromFeather).substring(index).toFloat();
            Serial.print("Battery voltage: ");
            Serial.println(vbat);
            // TODO: if battery voltage is too low shut down the device
        }else if(mesType.startsWith("meshStatusUpdate")){
            // message is a meshStatusUpdate message
            // TODO: parse the meshStatusUpdate message
        }else if(mesType.startsWith("repeaterActionRequest")){
            // message is a repeaterActionRequest message
            // TODO: parse the repeaterActionRequest message
        }else if(mesType.startsWith("message")){
            // message is a "message" message
            // TODO: parse the "message" message
        }
    }
    


    // TODO: send new location to Feather / other nodes

}

// print text to the display and shift the text up
void printTextToDisplay(String text) {
    displayTextBuffer[0] = displayTextBuffer[1]; // shift the text up
    displayTextBuffer[1] = displayTextBuffer[2]; // shift the text up
    displayTextBuffer[2] = displayTextBuffer[3]; // shift the text up
    displayTextBuffer[3] = text; // add the new text to the bottom
    display.clearDisplay(); // clear the display
    display.setCursor(0, 0); // set the cursor to the top left
    display.println(displayTextBuffer[0]); // print the text to the display line by line from the top 
    display.println(displayTextBuffer[1]);
    display.println(displayTextBuffer[2]);
    display.println(displayTextBuffer[3]);
    display.display(); // display the text
}

// void printStatusLineToDisplay(String text) {
//     //display.clearDisplay(); // clear the display
//     display.setCursor(0, 0); // set the cursor to the top left
//     display.print("-                        "); // print a blank line
//     display.setCursor(0, 0); // set the cursor to the top left
//     display.print("-");
//     display.print(text); 
//     display.display();
// }

// TODO: Write a function to send a message to the Feather

// Sends a message to the Feather to be sent to the mesh
void sendMeshMessageToFeather(uint8_t* data, uint16_t len){
    // set the IRQ pin low
    digitalWrite(FEATHER_IRQ_OUT_PIN, LOW);
    // wait for the IRQ in pin to go low
    while(digitalRead(FEATHER_IRQ_IN_PIN)==HIGH){
        delay(1);
    }
    // send the data
    FeatherSerial.write(data, len);
    // wait for the IRQ in pin to go high
    while(digitalRead(FEATHER_IRQ_IN_PIN)==LOW){
        delay(1);
    }
    // set the IRQ pin high
    digitalWrite(FEATHER_IRQ_OUT_PIN, HIGH);
    // delay for a bit just to be safe
    delay(50);
}

// Composes a message to be sent to the mesh
String composeMeshMessage(){
    return "";
}
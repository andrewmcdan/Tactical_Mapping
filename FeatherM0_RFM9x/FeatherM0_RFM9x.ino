///TODO: need to read a pin to determine if the module should be active or not.
///      if the pin is low, then the module should be active. If the pin is high, then the module should be inactive.
///      this will allow the module to be turned off when not in use, saving power and allowing the battery to charge faster.
///      the pin should be read in the setup() function and then the module should be turned on or off based on the value of the pin.
///      off will basically just mean sleeping, but the module will still be powered.

///TODO: if the battery level gets too low, the module should be turned off to prevent the battery from being damaged.

///TODO: need to be able to disconnect the battery from the module when it gets too low with a way to reconnect when power
///      is applied to the USB port.


#include <Arduino.h>
#include <SPI.h>
#include <RH_RF95.h>
//#include <RHDatagram.h>
#include <RHRouter.h>
#include <RHMesh.h>
#include <vector>
#include <cstdint>
#include <cstddef>

#define MODULE_STARTUP_WAIT_TIME 5

// Feather M0 w/Radio
#define RFM95_CS    8
#define RFM95_INT   3
#define RFM95_RST   4
#define RF95_FREQ 915.0 // Mhz. Match this with the version of your radio module!

// Teensy connection
#define TEENSY_IRQ_IN 12 // this is the pin that the Teensy will use to tell the Feather that it has data to send
#define TEENSY_IRQ_OUT 11 // this is the pin that the Feather will use to tell the Teensy that it has data to send

// Radio address pins (these are the pins that will be used to set the address of the radio)
#define ADDR_PIN_0 14
#define ADDR_PIN_1 15
#define ADDR_PIN_2 16
#define ADDR_PIN_3 17
#define ADDR_PIN_4 18
#define ADDR_PIN_5 19

#define VBATPIN A7 // this is the pin that the Feather will use to read the battery voltage

#define DEBUG_ENABLE_PIN 6 // this is the pin that the Feather will use to determine if debug mode should be enabled
#define DEBUG_LONE_MODULE 13 // this is the pin that the Feather will use to determine if it is the only module

#define MESH_BEACON_LENGTH 2 // the length of the mesh status packet
#define MESH_BEACON_PACKET_TYPE 0x0f
#define MESSAGE_PACKET_TYPE 0xf0
#define REPEATER_ACTION_REQUEST_PACKET_TYPE 0xa0

// calculate free RAM
extern "C" char *sbrk(int i);
size_t freeRAM() {
  char stack_dummy = 0;
  return &stack_dummy - sbrk(0);
}

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

RHMesh *meshManager; // this is the mesh manager that will be used to manage the mesh

// This class defines the array that will be used to store incoming messages
// and provides named access to certain parts of the array / packet.
class radioRecvMesArray{
private:
    uint8_t data[RH_RF95_MAX_MESSAGE_LEN];
public:
    uint8_t& messageType = data[0];
    uint8_t& destinationAddress = data[1];
    uint8_t& originAddress = data[2];
    uint8_t& hopCount = data[3];

    uint8_t& operator[](size_t index){
        if(index < sizeof(data)){
            return data[index];
        }else{
            return data[0];
        }
        
    }

    operator uint8_t*(){
        return data;
    }
};

class SerialDebug{
private:
    bool debugEnabled;
public:
    SerialDebug(bool debugEnabled){
        this->debugEnabled = debugEnabled;
        if(debugEnabled){
            Serial.begin(115200);
            while (!Serial) delay(1); // wait for serial port to connect. Needed for native USB
        }
    }
    // default constructor
    SerialDebug(){}

    template <typename T>
    void print(const T &data) {
        if (debugEnabled) {
            Serial.print(data);
        }
    }

    template <typename T>
    void println(const T &data) {
        if (debugEnabled) {
            Serial.println(data);
        }
    }

    template <typename T>
    void print(const T &data, int base) {
        if (debugEnabled) {
            Serial.print(data, base);
        }
    }

    template <typename T>
    void println(const T &data, int base) {
        if (debugEnabled) {
            Serial.println(data, base);
        }
    }
};

radioRecvMesArray recvBuf; // this is the buffer that the radio will use to store incoming messages
uint8_t recvBufLen = RH_RF95_MAX_MESSAGE_LEN; // this is the length of the buffer that the radio will use to store incoming messages
uint8_t fromAddr; // this is the address of the node that sent the message
uint8_t dataOutBuf[RH_RF95_MAX_MESSAGE_LEN]; // this is the buffer that the radio will use to store outgoing messages
//uint8_t maxAddrFound = 1; // this is the highest address that has been found on the mesh
uint8_t toAddr; // this is the address that the message is being sent to
uint8_t recvFromId; // this is the id of the node that sent the message
uint8_t recvFlags; // these are the flags that were set when the message was sent
uint8_t address; // this is the address of this node
uint8_t routes[64]; // this is the array that will store the routes to the nodes
unsigned long loopStart = millis(); // this is the time that the loop started
unsigned long loopEnd = millis(); // this is the time that the loop ended
unsigned long timeSinceMeshUpdate = millis(); // this is the time since the last mesh update was received
unsigned long timeSinceLastBattUpdate = millis(); // this is the time since the last battery update was sent
uint8_t serialTeensyRecvBuf[1024]; // this is the buffer that will store incoming data from the Teensy
uint16_t serialTeensyRecvBufLen = sizeof(serialTeensyRecvBuf); // this is the length of the buffer that will store incoming data from the Teensy
bool loneModule = false; // this is the flag that determines if this is the only module or not
SerialDebug debugSerial;

void setup() {
    bool debugEnabled = false; // this is the flag that determines if debug mode is enabled or not
    pinMode(LED_BUILTIN, OUTPUT);
    uint8_t waitCount_ = 0;
    // Give the other Teensy time to start up so that 
    // the IRQ pin is not toggled before the Teensy is ready
    while(waitCount_ < MODULE_STARTUP_WAIT_TIME){
        digitalWrite(LED_BUILTIN, HIGH);
        delay(500);
        digitalWrite(LED_BUILTIN, LOW);
        delay(500);
        waitCount_++;
    }
    String dataToSendToTeensy = "";
    // setup the radio address pins
    pinMode(ADDR_PIN_0, INPUT);
    pinMode(ADDR_PIN_1, INPUT);
    pinMode(ADDR_PIN_2, INPUT);
    pinMode(ADDR_PIN_3, INPUT);
    pinMode(ADDR_PIN_4, INPUT);
    pinMode(ADDR_PIN_5, INPUT);

    // Setup and read the debug enable pin
    pinMode(DEBUG_ENABLE_PIN, INPUT);
    if(digitalRead(DEBUG_ENABLE_PIN) == LOW) {
        debugEnabled = true;
        // set address to random value between 0 and 63, inclusive
        address = random(0, 64);
    }else{
        // set address using address pins
        address = (digitalRead(ADDR_PIN_0) << 0) | (digitalRead(ADDR_PIN_1) << 1) | (digitalRead(ADDR_PIN_2) << 2) | (digitalRead(ADDR_PIN_3) << 3) | (digitalRead(ADDR_PIN_4) << 4) | (digitalRead(ADDR_PIN_5) << 5);
    }

    // Setup and read the lone module pin
    pinMode(DEBUG_LONE_MODULE, INPUT);
    if(digitalRead(DEBUG_LONE_MODULE) == LOW) {
        loneModule = true;
    }
    // Setup the rest of the pins
    pinMode(TEENSY_IRQ_IN, INPUT);
    pinMode(TEENSY_IRQ_OUT, OUTPUT);
    digitalWrite(TEENSY_IRQ_OUT, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);
    delay(100);
    // setup the serial ports
    Serial1.begin(115200);
    debugSerial = SerialDebug(debugEnabled);
    delay(100);
    debugSerial.println("Reset radio...");
    // manual reset
    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(100);
    debugSerial.println("Radio reset complete.");
    debugSerial.println("Init radio using mesh manager...");
    meshManager = new RHMesh(rf95, address);
    if (!meshManager->init()) {
        debugSerial.println("LoRa radio init failed... stopping.");
        dataToSendToTeensy = dataToSendToTeensy + "x1";
        while (1);
    }
    debugSerial.println("LoRa radio init OK!");
    if (!rf95.setFrequency(RF95_FREQ)) {
        debugSerial.println("setFrequency failed... stopping");
        //Serial1.println("x2");
        dataToSendToTeensy = dataToSendToTeensy + "x2";
        while (1);
    }
    debugSerial.print("Freq set to: ");
    debugSerial.println(RF95_FREQ);
    debugSerial.println("Adjusting modem mode...");
    if (!rf95.setModemConfig(RH_RF95::ModemConfigChoice::Bw125Cr45Sf128)) {//< Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Default medium range
        debugSerial.println("Modem mode set failed... stopping.");
        //Serial1.println("x3");
        dataToSendToTeensy = dataToSendToTeensy + "x3";
        while (1);
    }
    debugSerial.println("Successfully set modem config");
    debugSerial.print("Setting tramsit power...");
    rf95.setTxPower(23, false);
    debugSerial.println(" complete.");
    
    // reset the routes array
    for(uint8_t i = 0; i < 256; i++){
        routes[i] = 255;
    }

    debugSerial.println("Radio setup complete. Continuing...");

    // instantiate the mesh using the maxAddrFound as the address for the first node.
    // The first node in the vector will be this device.

    if(!loneModule){
        // next thing to do is to wait for the Teensy to finish setup. This is done by
        // waiting for the IRQ pin to go low.

        // if the Teensy gets through its setup, it will set the IRQ pin to low and
        // wait for the Feather to set its IRQ pin to low.
        debugSerial.println("Give the Teensy time to start...");
        //delay(1000); // give the Teensy time to start up
        digitalWrite(TEENSY_IRQ_OUT, LOW);
        delay(100);
        debugSerial.println("Waiting for Teensy to finish setup...");
        while (digitalRead(TEENSY_IRQ_IN) == HIGH) {
            delay(100);
        }
        while (digitalRead(TEENSY_IRQ_IN) == LOW) {
            delay(100);
        }
        digitalWrite(TEENSY_IRQ_OUT, HIGH);
        delay(1000);
        Serial1.println(dataToSendToTeensy); // send to the Teensy the data that was collected during setup
        delay(1000);
        debugSerial.println("Teensy setup complete. Continuing.");
    }
    debugSerial.println("Feather setup complete. Continuing.");
    debugSerial.println("Free memory: " + String(freeRAM()));
}

void loop() {
    //Serial.println("Loop Start\nFree memory: " + String(freeRAM()));
    loopStart = millis();
    uint8_t dataOutBufLen = 0;
    // next thing to do is to send the battery voltage every 30 seconds to the Teensy
    if(millis() - timeSinceLastBattUpdate > 30000){
        debugSerial.println("Loop Start\nFree memory: " + String(freeRAM()));
        timeSinceLastBattUpdate = millis(); // reset the timer
        float measured_vbat = analogRead(VBATPIN); // read the battery voltage
        measured_vbat *= 2;    // we divided by 2, so multiply back
        measured_vbat *= 3.3;  // Multiply by 3.3V, our reference voltage
        measured_vbat /= 1024; // convert to voltage

        if(measured_vbat < 3.30){ // if the battery voltage is below 3.4V, stop the module and turn off the battery
            debugSerial.println("Battery voltage is low: " + String(measured_vbat) + " Stopping.");
            Serial1.println("x2");
            sendDataToTeensy("vbat:low");
            //digitalWrite(BATT_RELAY_PIN, LOW); // turn off the battery
            while(1){}
        }
        debugSerial.print("VBat: " ); // debug
        debugSerial.println(measured_vbat); // debug
        if(!sendDataToTeensy("vbat:" + String(measured_vbat))){ // send the data to the Teensy
            debugSerial.println("Error sending vbat data to Teensy"); // debug
        }
    }



    uint16_t serialTeensyDataInBufLen = 0;

    if(digitalRead(TEENSY_IRQ_IN) == LOW){
        debugSerial.println("Data available from Teensy");
        digitalWrite(TEENSY_IRQ_OUT, LOW);
        // keep trying to read data from the Teensy until the Teensy sets the IRQ pin to high
        while(digitalRead(TEENSY_IRQ_IN) == LOW){
            // read the data from the Teensy for as long as the Teensy is sending data
            while(Serial1.available()){
                serialTeensyRecvBuf[serialTeensyDataInBufLen++] = Serial1.read();
            }
        }
        digitalWrite(TEENSY_IRQ_OUT, HIGH);
    }

    // check the irq pin to see if the Teensy has an interrupt
    // if so, read the data from the serial port and interpret it
    // The Teensy should only ever send data to the Feather that the
    // Feather needs to send to the mesh.

    if(serialTeensyDataInBufLen > 0){
        debugSerial.println("Teensy sent some data");
        debugSerial.println("Free memory: " + String(freeRAM()));
        debugSerial.println("Data received from Teensy: ");
        
        // Now we need to interpret the data
        // the data is sent as a packet with the following format:
        // messages to be sent to the mesh:
        // message:len=length:data=data
        // where:
        // length - the length of the data as a string
        // data - the data of the message as raw bytes
        // commands for the Feather:
        // command:[command string]

        // first we need to check if the data is a command or a message
        char typeBuf[8];
        for(uint8_t i = 0; i < 7; i++){
            typeBuf[i] = serialTeensyRecvBuf[i];
        }
        typeBuf[7] = '\0';
        String type = String(typeBuf);
        if(type == "message"){
            // split the serialTeensyRecvBuf array into a length string
            String lengthStr = "";
            uint8_t indexOfData = 0;
            // first we find the index of the first '='
            while(serialTeensyRecvBuf[indexOfData] != '='){
                indexOfData++;
            }
            indexOfData++; // this should now be the start of the length string

            // now add the data to the length string until we find the ':' character
            while(serialTeensyRecvBuf[indexOfData] != ':'){
                lengthStr += (char)serialTeensyRecvBuf[indexOfData];
                indexOfData++;
            }
            // read until we get to the next '=' character
            while(serialTeensyRecvBuf[indexOfData] != '='){
                indexOfData++;
            }
            indexOfData++; // this should now be the start of the data

            for(uint8_t i = 0; i < indexOfData; i++){
                Serial.print((char)serialTeensyRecvBuf[i]);
            }
            Serial.println("..."); // print an ellipsis to show that there is more data

            uint16_t length = lengthStr.toInt();

            // now send the data to the mesh
            debugSerial.println("Sending data to mesh...");
            // TODO: fix this
            // if(!radio.sendto((uint8_t *)serialTeensyRecvBuf + indexOfData, length, RH_BROADCAST_ADDRESS)){
            //     Serial.println("Failed to send data to mesh");
            // }
        }else if(type == "command"){
            String command = "";
            for(uint8_t i = 8; i < serialTeensyDataInBufLen; i++){
                command += (char)serialTeensyRecvBuf[i];
            }
            // now we need to check what the command is
            if(command.substring(0, 5) == "addr="){
                // the command is to set the address of the Feather
                address = command.substring(6).toInt();
                debugSerial.println("Address: " + String(address));
                debugSerial.println("Setting address...");
                // TODO: need to set the address of the radio
                //radio.setThisAddress(address);
                debugSerial.println("Address set.");
            }
        }
    }

    //debugSerial.println("Loop End\nFree memory: " + String(freeRAM()));
    // wait for a bit before looping again 
    loopEnd = millis();
    //if(loopEnd - loopStart < 100){
    //    delay(100 - (loopEnd - loopStart));
    //}
}

// function to send data to the Teensy over the serial port 
// returns true if the data was sent successfully, false otherwise
bool sendDataToTeensy(String data){
    //       The Teensy can use a large buffer for the serial port which will allow us to send data without using interrupts
    debugSerial.println("Sending data to Teensy..."); // debug
    Serial1.println(data); // send the data to the Teensy
    debugSerial.println("Data sent to Teensy"); // debug
    // return true to indicate that we successfully sent the data
    return true;
}

// a version of the sendDataToTeensy function that takes a uint8_t array and a length
bool sendDataToTeensy(uint8_t *data, uint8_t len){ // a version of the sendDataToTeensy function that takes a uint8_t array and a length
    debugSerial.println("Sending data to Teensy..."); // debug message
    Serial1.write(data, len); // send the data to the Teensy
    Serial1.println(); // send a newline character to the Teensy
    debugSerial.println("Data sent to Teensy"); // debug message
    return true; // return true to indicate that we successfully sent the data
}

///TODO: need to read a pin to determine if the module should be active or not.
///     if the pin is low, then the module should be active. If the pin is high, then the module should be inactive.
///     this will allow the module to be turned off when not in use, saving power and allowing the battery to charge faster.
///     the pin should be read in the setup() function and then the module should be turned on or off based on the value of the pin.
///     off will basically just mean sleeping, but the module will still be powered.

///TODO: if the battery level gets too low, the module should be turned off to prevent the battery from being damaged.

///TODO: need to be able to disconnect the battery from the module when it gets too low with a way to reconnect when power
///     is applied to the USB port.



#include <Arduino.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <RHDatagram.h>

#define MODULE_STARTUP_WAIT_TIME 15

// Feather M0 w/Radio
#define RFM95_CS    8
#define RFM95_INT   3
#define RFM95_RST   4
#define RF95_FREQ 915.0 // Mhz. Match this with the version of your radio module!

// Teensy connection
#define TEENSY_IRQ_IN 12 // this is the pin that the Teensy will use to tell the Feather that it has data to send
#define TEENSY_IRQ_OUT 11 // this is the pin that the Feather will use to tell the Teensy that it has data to send

#define VBATPIN A7 // this is the pin that the Feather will use to read the battery voltage

#define BATT_RELAY_PIN 10 // this is the pin that the Feather will use to turn the battery relay on and off

#define DEBUG_ENABLE_PIN 6 // this is the pin that the Feather will use to determine if debug mode should be enabled

#define MESH_STATUS_LENGTH 2 // the length of the mesh status packet
#define MESH_STATUS_PACKET_TYPE 0x0f
#define MESSAGE_PACKET_TYPE 0xf0
#define REPEATER_ACTION_REQUEST_PACKET_TYPE 0xa0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

RHDatagram radio(rf95); // starts datagram manager with default address of 0

uint8_t recvBuf[RH_RF95_MAX_MESSAGE_LEN]; // this is the buffer that the radio will use to store incoming messages
uint8_t recvBufLen = sizeof(recvBuf); // this is the length of the buffer that the radio will use to store incoming messages
uint8_t fromAddr; // this is the address of the node that sent the message
uint8_t dataOutBuf[RH_RF95_MAX_MESSAGE_LEN]; // this is the buffer that the radio will use to store outgoing messages
uint8_t maxAddrFound = 1; // this is the highest address that has been found on the mesh
uint8_t toAddr; // this is the address that the message is being sent to
uint8_t recvFromId; // this is the id of the node that sent the message
uint8_t recvFlags; // these are the flags that were set when the message was sent
unsigned long loopStart = millis(); // this is the time that the loop started
unsigned long loopEnd = millis(); // this is the time that the loop ended
unsigned long timeSinceMeshUpdate = millis(); // this is the time since the last mesh update was received
unsigned long timeSinceLastBattUpdate = millis(); // this is the time since the last battery update was sent
//TODO: remove debug flag
bool debugEnabled = false; // this is the flag that determines if debug mode is enabled or not

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    uint8_t waitCount_ = 0;
    // Give the other modules time to start up
    while(waitCount_ < MODULE_STARTUP_WAIT_TIME){
        digitalWrite(LED_BUILTIN, HIGH);
        delay(500);
        digitalWrite(LED_BUILTIN, LOW);
        delay(500);
        waitCount_++;
    }
    String dataToSendToTeensy = "";
    // Setup and read the debug enable pin
    pinMode(DEBUG_ENABLE_PIN, INPUT);
    if(digitalRead(DEBUG_ENABLE_PIN) == LOW) {
        debugEnabled = true;
    }
    // Setup the rest of the pins
    pinMode(BATT_RELAY_PIN, OUTPUT);
    digitalWrite(BATT_RELAY_PIN, HIGH); 
    pinMode(TEENSY_IRQ_IN, INPUT);
    pinMode(TEENSY_IRQ_OUT, OUTPUT);
    digitalWrite(TEENSY_IRQ_OUT, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);
    delay(100);
    // setup the serial ports
    Serial1.begin(115200);
    if(debugEnabled){
        Serial.begin(115200);
        while (!Serial) delay(1); // wait for serial port to connect. Needed for native USB
        Serial.println("Booting...");
    }
    delay(100);
    Serial.println("Reset radio...");
    // manual reset
    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(100);
    Serial.println("Radio reset complete.");
    Serial.println("Init radio using datagram manager...");
    if (!radio.init()) {
        Serial.println("LoRa radio init failed... stopping.");
        dataToSendToTeensy = dataToSendToTeensy + "x1";
        while (1);
    }
    Serial.println("LoRa radio init OK!");
    if (!rf95.setFrequency(RF95_FREQ)) {
        Serial.println("setFrequency failed... stopping");
        //Serial1.println("x2");
        dataToSendToTeensy = dataToSendToTeensy + "x2";
        while (1);
    }
    Serial.print("Freq set to: ");
    Serial.println(RF95_FREQ);
    Serial.println("Adjusting modem mode...");
    if (!rf95.setModemConfig(RH_RF95::ModemConfigChoice::Bw125Cr45Sf128)) {//< Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Default medium range
        Serial.println("Modem mode set failed... stopping.");
        //Serial1.println("x3");
        dataToSendToTeensy = dataToSendToTeensy + "x3";
        while (1);
    }
    Serial.println("Successfully set modem config");
    Serial.print("Setting tramsit power...");
    rf95.setTxPower(23, false);
    Serial.println(" complete.");
    Serial.println("Start to find other nodes...");
    if (!radio.waitAvailableTimeout(30000)) { // if after 30 seconds nothing is received, assume there is no mesh
        Serial.println("No other nodes found. Setting address = 1");
        //Serial1.println("addr=1");
        dataToSendToTeensy = dataToSendToTeensy + "addr=1";
        radio.setThisAddress(1);
    } else {
        // we need to get a mesh status update in order to set this node's address, so keep looping until one comes in
        Serial.println("Other nodes found. Waiting for mesh status update...");
        bool meshStatusRecvd = false;
        while (!meshStatusRecvd) {
            radio.recvfrom(recvBuf, &recvBufLen, &fromAddr, &toAddr, &recvFromId, &recvFlags);
            Serial.println("Received data");
            Serial.print("From: 0x");
            Serial.println(fromAddr, HEX);
            Serial.print("To: 0x");
            Serial.println(toAddr, HEX);
            Serial.print("ID: 0x");
            Serial.println(recvFromId, HEX);
            Serial.print("Flags: 0x");
            Serial.println(recvFlags, HEX);

            if (recvBufLen == MESH_STATUS_LENGTH && recvBuf[0] == MESH_STATUS_PACKET_TYPE && toAddr == RH_BROADCAST_ADDRESS) {
                Serial.print("Max address found: 0x");
                Serial.println(recvBuf[1], HEX);
                maxAddrFound = recvBuf[1] + 1;
                //Serial1.println("addr=" + String(maxAddrFound));
                dataToSendToTeensy = dataToSendToTeensy + "addr=" + String(maxAddrFound);
                radio.setThisAddress(maxAddrFound);
                meshStatusRecvd = true;
            }else{
                Serial.println("Received packet was not a mesh status update. Ignoring.");
                Serial.println("Waiting for another packet...");
            }
            if(!meshStatusRecvd){
                // waiting for another packet
                // This has a long timeout because in the case where this code gets executed,
                // this node is simply waiting for a mesh Status update.
                if(!radio.waitAvailableTimeout(60000)){
                    Serial.println("No more packets received after 60 seconds. Stopping.");
                    //Serial1.println("x4");
                    dataToSendToTeensy = dataToSendToTeensy + "x4";
                    while(1){}
                }
            }
        }
    }
    Serial.println("Radio setup complete. Continuing...");

    // next thing to do is to wait for the Teensy to finish setup. This is done by
    // waiting for the IRQ pin to go low.

    // if the Teensy gets through its setup, it will set the IRQ pin to low and
    // wait for the Feather to set its IRQ pin to low.
    Serial.println("Give the Teensy time to start...");
    delay(10000); // give the Teensy time to start up
    digitalWrite(TEENSY_IRQ_OUT, LOW);
    delay(100);
    Serial.println("Waiting for Teensy to finish setup...");
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
    Serial.println("Teensy setup complete. Continuing.");
    Serial.println("Feather setup complete. Continuing.");
}

void loop() {
    loopStart = millis();
    // first thing to do is send out a broadcast that indicates basic mesh status.
    // each node should do this so that in the event a segment of the mesh gets seperated,
    // and another node connects to the seperated section, all nodes will connect with
    // no overlapping addresses.
    // maxAddrFound - the highest addr of all the nodes
    uint8_t dataOutBufLen = 0;
    if(millis() - timeSinceMeshUpdate > 5000){ // Only send out a mesh update every 5 seconds
        timeSinceMeshUpdate = millis(); // reset the timer
        dataOutBuf[dataOutBufLen++] = MESH_STATUS_PACKET_TYPE; // mesh status flag
        dataOutBuf[dataOutBufLen++] = maxAddrFound; // the max address found
        Serial.println("Broadcasting mesh update..."); // debug
        radio.sendto(dataOutBuf, MESH_STATUS_LENGTH, RH_BROADCAST_ADDRESS); // send the packet
        dataOutBufLen = 0; // reset the dataOutBufLen
    }

    // next thing to do is to send the battery voltage every 30 seconds to the Teensy
    if(millis() - timeSinceLastBattUpdate > 30000){
        timeSinceLastBattUpdate = millis(); // reset the timer
        float measured_vbat = analogRead(VBATPIN); // read the battery voltage
        measured_vbat *= 2;    // we divided by 2, so multiply back
        measured_vbat *= 3.3;  // Multiply by 3.3V, our reference voltage
        measured_vbat /= 1024; // convert to voltage

        if(measured_vbat < 3.30){ // if the battery voltage is below 3.4V, stop the module and turn off the battery
            Serial.println("Battery voltage is low: " + String(measured_vbat) + " Stopping.");
            Serial1.println("x2");
            sendDataToTeensy("vbat:low");
            digitalWrite(BATT_RELAY_PIN, LOW); // turn off the battery
            while(1){}
        }
        Serial.print("VBat: " ); // debug
        Serial.println(measured_vbat); // debug
        if(!sendDataToTeensy("vbat:" + String(measured_vbat))){ // send the data to the Teensy
            Serial.println("Error sending vbat data to Teensy"); // debug
        }
    }

    // then receive any messages on the waves...
    // if a message is received, send it to the Teensy.
    // There is a 1 second timeout on the waitAvailableTimeout function so if the
    // message is not received, the code will just keep looping.
    if(radio.available()){
    //if(radio.waitAvailableTimeout(500)){
        radio.recvfrom(recvBuf, &recvBufLen, &fromAddr, &toAddr, &recvFromId, &recvFlags); // receive the data
        Serial.print("Received data from: 0x"); // debug
        Serial.println(fromAddr, HEX);
        Serial.print("To: 0x");
        Serial.println(toAddr, HEX);
        Serial.print("ID: 0x");
        Serial.println(recvFromId, HEX);
        Serial.print("Flags: 0x");
        Serial.println(recvFlags, HEX);
        if (recvBufLen == MESH_STATUS_LENGTH && recvBuf[0] == MESH_STATUS_PACKET_TYPE && toAddr == RH_BROADCAST_ADDRESS) { // if the packet is a mesh status update
            Serial.println("Mesh status update received. Updating max addr"); // debug
            Serial.print("Max address: 0x");
            Serial.println(recvBuf[1], HEX);
            // if the new max addr is greater than the current max addr, update it
            if(recvBuf[1] > maxAddrFound){
                maxAddrFound = recvBuf[1];
            }
            
            // need to send an update to the Teensy about RSSI and SNR
            int16_t rssi = rf95.lastRssi(); // get the RSSI of the last packet received
            int snr = rf95.lastSNR(); // get the SNR of the last packet received
            Serial.print("RSSI: "); // debug
            Serial.println(rssi);
            Serial.print("SNR: ");
            Serial.println(snr);
            // send the message to the Teensy
            // the message is sent as a packet with the following format:
            // meshStatusUpdate:rssi=rssi:snr=snr
            // where:
            // rssi - the RSSI of the last packet received
            // snr - the SNR of the last packet received
            // the message is sent to the Teensy as a packet.
            // the Teensy will decide what to do with the message.
            Serial.println("Sending mesh status update to Teensy...");
            if(!sendDataToTeensy("meshStatusUpdate:rssi=" + String(rssi) + ":snr=" + String(snr))){ // send the data to the Teensy
                Serial.println("Error sending mesh status update to Teensy");
            }
        }else if(recvBuf[0] == (MESH_STATUS_PACKET_TYPE & REPEATER_ACTION_REQUEST_PACKET_TYPE)  && toAddr == RH_BROADCAST_ADDRESS){ // repeater action request packet received
            // send the message to the Teensy
            // the message is sent as a packet with the following format:
            // repeaterActionRequest:from=from:to=to:id=id:flags=flags:data=data
            // where:
            // from - the from address
            // to - the to address
            // id - the id of the message
            // flags - the flags of the message
            // data - the data of the message
            // the message is sent to the Teensy as a packet.
            // the Teensy will decide what to do with the message.
            Serial.println("Repeater action request packet received. Sending to Teensy...");
            if(!sendDataToTeensy("repeaterActionRequest:from=" + String(fromAddr) + ":to=" + String(toAddr) + ":id=" + String(recvFromId) + ":flags=" + String(recvFlags) + ":data=")){
                Serial.println("Failed to send data to Teensy");
            }
            if(!sendDataToTeensy(recvBuf, recvBufLen)){
                Serial.println("Failed to send data to Teensy");
            }
        }else if(recvBuf[0] == MESSAGE_PACKET_TYPE && toAddr == RH_BROADCAST_ADDRESS){ // message packet received
            // send the message to the Teensy
            // the message is sent as a packet with the following format:
            // message:len=length:from=from:to=to:id=id:flags=flags:data=data
            // where:
            // length - the length of the data
            // from - the from address
            // to - the to address
            // id - the id of the message
            // flags - the flags of the message
            // data - the data of the message
            // the message is sent to the Teensy as a packet.
            // the Teensy will decide what to do with the message.
            Serial.println("Message type packet received. Sending to Teensy...");
            if(!sendDataToTeensy("message:len=" + String(recvBufLen) + ":from=" + String(fromAddr) + ":to=" + String(toAddr) + ":id=" + String(recvFromId) + ":flags=" + String(recvFlags) + ":data=")){
                Serial.println("Failed to send data to Teensy");
            }
            if(!sendDataToTeensy(recvBuf, recvBufLen)){
                Serial.println("Failed to send data to Teensy");
            }
        }
    }

    // check the irq pin to see if the Teensy has an interrupt
    // if so, read the data from the serial port and interpret it

    if(digitalRead(TEENSY_IRQ_IN) == LOW){
        Serial.println("Teensy has an interrupt");
        Serial1.println("ready");
        uint8_t dataInBuf[RH_RF95_MAX_MESSAGE_LEN + 32];
        uint16_t dataInBufLen = 0;
        uint16_t waitCount = 0;
        // wait for Teensy to be ready and to send some data
        while(Serial1.available() || digitalRead(TEENSY_IRQ_IN) == LOW){
            // if the Teensy is ready, read the data
            while(!Serial1.available()){
                waitCount++;
                delay(10);
                if(waitCount > 100){
                    waitCount = 0;
                    // if we get here there was a timeout waiting for data and both micros should abort
                    Serial.println("Timeout waiting for Teensy data");
                    Serial1.println("xxxxxxx");
                    break;
                }
            }
            if(Serial1.available()){
                dataInBuf[dataInBufLen++] = Serial1.read();
            }
        }
        if(dataInBufLen > 0){
            Serial.println("Data received from Teensy: ");
            
            // Now we need to interpret the data
            // the data is sent as a packet with the following format:
            // message:len=length:data=data
            // where:
            // length - the length of the data as a string
            // data - the data of the message as raw bytes

            // split the dataInBuf array into a length string
            String lengthStr = "";
            uint8_t indexOfData = 0;
            while(dataInBuf[indexOfData] != '='){
                indexOfData++;
            }
            indexOfData++;
            while(dataInBuf[indexOfData] != ':'){
                lengthStr += dataInBuf[indexOfData];
                indexOfData++;
            }
            indexOfData++;
            while(dataInBuf[indexOfData] != '='){
                indexOfData++;
            }
            indexOfData++; // this should now be the start of the data

            for(uint8_t i = 0; i < indexOfData; i++){
                Serial.print((char)dataInBuf[i]);
            }
            Serial.println("..."); // print an ellipsis to show that there is more data

            uint16_t length = lengthStr.toInt();

            // now send the data to the mesh
            Serial.println("Sending data to mesh...");
            if(!radio.sendto((uint8_t *)dataInBuf + indexOfData, length, RH_BROADCAST_ADDRESS)){
                Serial.println("Failed to send data to mesh");
            }
        }
    }

    // wait for a second before looping again 
    loopEnd = millis();
    if(loopEnd - loopStart < 100){
        delay(100 - (loopEnd - loopStart));
    }
}

bool sendDataToTeensy(String data){
    Serial.println("Sending data to Teensy..."); // debug
    String dataInStr = ""; // the data received from the Teensy
    uint16_t waitCount = 0; // the number of times we have waited for the Teensy to be ready
    digitalWrite(TEENSY_IRQ_OUT, LOW); // set the Teensy IRQ pin to low to tell the Teensy that we are ready to send data
    // wait for Teensy to be ready
    while(!Serial1.available()){ // wait for the Teensy to send "ready"
        delay(10); // wait 10ms
        waitCount++; // increment the wait count
        if(waitCount > 100){ // if we have waited for 1 second
            digitalWrite(TEENSY_IRQ_OUT, HIGH); // set the Teensy IRQ pin to high to tell the Teensy that we are not ready to send data
            return false; // return false to indicate that we failed to send the data
        }
    }
    
    while(Serial1.available()){ // while there is data available from the Teensy
        dataInStr += Serial1.read(); // read the data and add it to the dataInStr string
        if(dataInStr.length() > 5){ // if the dataInStr string is longer than 5 characters
            digitalWrite(TEENSY_IRQ_OUT, HIGH);
            return false; // return false to indicate that we failed to send the data
        }
    }
    if(dataInStr == "ready"){
        // if we get here, the Teensy is ready to receive data
        Serial1.println(data); // send the data to the Teensy
        Serial.println("Data sent to Teensy"); // debug
    }
    // set the Teensy IRQ pin to high to tell the Teensy that we are not ready to send data
    digitalWrite(TEENSY_IRQ_OUT, HIGH);
    // return true to indicate that we successfully sent the data
    return true;
}

bool sendDataToTeensy(uint8_t *data, uint8_t len){ // a version of the sendDataToTeensy function that takes a uint8_t array and a length
    Serial.println("Sending data to Teensy...");
    String dataInStr = "";
    uint16_t waitCount = 0;
    digitalWrite(TEENSY_IRQ_OUT, LOW);
    // wait for Teensy to be ready
    while(!Serial1.available()){
        delay(10);
        waitCount++;
        if(waitCount > 100){
            digitalWrite(TEENSY_IRQ_OUT, HIGH);
            return false;
        }
    }
    while(Serial1.available()){
        dataInStr += Serial1.read();
        if(dataInStr.length() > 5){
            digitalWrite(TEENSY_IRQ_OUT, HIGH);
            return false;
        }
    }
    if(dataInStr == "ready"){
        Serial1.write(data, len);
        Serial.println("Data sent to Teensy");
    }
    digitalWrite(TEENSY_IRQ_OUT, HIGH);
    return true;
}

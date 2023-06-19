#include <arduino.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <RHDatagram.h>


// Feather M0 w/Radio
#define RFM95_CS    8
#define RFM95_INT   3
#define RFM95_RST   4
#define RF95_FREQ 915.0

// Teensy connection
#define TEENSY_IRQ_IN 12
#define TEENSY_IRQ_OUT 13

#define MESH_STATUS_LENGTH 2
#define MESH_STATUS_PACKET_TYPE 0x0f
#define MESSAGE_PACKET_TYPE 0xf0
#define REPEATER_ACTION_REQUEST_PACKET_TYPE 0xa0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

RHDatagram radio(rf95); // starts datagram manager with default address of 0

uint8_t recvBuf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t recvBufLen = sizeof(recvBuf);
uint8_t fromAddr;
uint8_t dataOutBuf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t maxAddrFound = 1;
uint8_t toAddr;
uint8_t recvFromId;
uint8_t recvFlags;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(TEENSY_IRQ_IN, INPUT);
    pinMode(TEENSY_IRQ_OUT, OUTPUT);
    digitalWrite(TEENSY_IRQ_OUT, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);
    delay(100);
    Serial.begin(115200);
    Serial1.begin(115200);
    while (!Serial) delay(1);
    Serial1.println("booting");
    Serial.println("Booting...");
    delay(100);
    Serial.println("Reset radio...");
    // manual reset
    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(100);
    Serial.println("Radio reset complete.");
    Serial.println("Init radio using datagram manager...");
    while (!radio.init()) {
        Serial.println("LoRa radio init failed... stopping.");
        Serial1.println("x1");
        while (1);
    }
    Serial.println("LoRa radio init OK!");
    if (!rf95.setFrequency(RF95_FREQ)) {
        Serial.println("setFrequency failed... stopping");
        Serial1.println("x2");
        while (1);
    }
    Serial.print("Freq set to: ");
    Serial.println(RF95_FREQ);

    Serial.println("Adjusting modem mode...");
    if (!rf95.setModemConfig(RH_RF95::ModemConfigChoice::Bw125Cr45Sf128)) {//< Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Default medium range
        Serial.println("Modem mode set failed... stopping.");
        Serial1.println("x3l");
        while (1) {}
    }
    Serial.println("Successfully set modem config");
    Serial.print("Setting tramsit power...");
    rf95.setTxPower(23, false);
    Serial.println(" complete.");
    Serial.println("Start to find other nodes...");
    if (!radio.waitAvailableTimeout(30000)) { // if after 30 seconds nothing is received, assume there is no mesh
        Serial.println("No other nodes found. Setting address = 1");
        Serial1.println("addr=1");
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
                Serial1.println("addr=" + String(maxAddrFound));
                radio.setThisAddress(maxAddrFound);
                meshStatusRecvd = true;
            }
            if(!meshStatusRecvd){
                // waiting for another packet
                // This has a long timeout because in the case where this code gets executed,
                // this node is simply waiting for a mesh Status update.
                if(!radio.waitAvailableTimeout(60000)){
                    Serial.println("No mesh status update received after 60 seconds. Stopping.");
                    Serial1.println("x4");
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
    digitalWrite(TEENSY_IRQ_OUT, LOW);
    Serial.println("Waiting for Teensy to finish setup...");
    while (digitalRead(TEENSY_IRQ_IN) == HIGH) {}
    digitalWrite(TEENSY_IRQ_OUT, HIGH);
    Serial.println("Setup complete complete. Continuing.");
}

void loop() {
    unsigned long loopStart = millis();
    // first thing to do is send out a broadcast that indicates basic mesh status.
    // each node should do this so that in the event a segment of the mesh gets seperated,
    // and another node connects to the seperated section, all nodes will connect with
    // no overlapping addresses.
    // maxAddrFound - the highest addr of all the nodes
    uint8_t dataOutBufLen = 0;
    dataOutBuf[dataOutBufLen++] = MESH_STATUS_PACKET_TYPE; // mesh status flag
    dataOutBuf[dataOutBufLen++] = maxAddrFound;
    radio.sendto(dataOutBuf, MESH_STATUS_LENGTH, RH_BROADCAST_ADDRESS);
    dataOutBufLen = 0; // reset the dataOutBufLen

    // then receive any messages on the waves...
    // if a message is received, send it to the Teensy.
    // There is a 1 second timeout on the waitAvailableTimeout function so if the
    // message is not received, the code will just keep looping.
    if(radio.waitAvailableTimeout(1000)){
        radio.recvfrom(recvBuf, &recvBufLen, &fromAddr, &toAddr, &recvFromId, &recvFlags);
        Serial.print("Received data from: 0x");
        Serial.println(fromAddr, HEX);
        Serial.print("To: 0x");
        Serial.println(toAddr, HEX);
        Serial.print("ID: 0x");
        Serial.println(recvFromId, HEX);
        Serial.print("Flags: 0x");
        Serial.println(recvFlags, HEX);
        if (recvBufLen == MESH_STATUS_LENGTH && recvBuf[0] == MESH_STATUS_PACKET_TYPE && toAddr == RH_BROADCAST_ADDRESS) {
            Serial.println("Mesh status update received. Updating max addr");
            Serial.print("Max address: 0x");
            Serial.println(recvBuf[1], HEX);
            // if the new max addr is greater than the current max addr, update it
            if(recvBuf[1] > maxAddrFound){
                maxAddrFound = recvBuf[1];
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

    if(digitalRead(TEENSY_IRQ_IN) == HIGH){
        Serial.println("Teensy has an interrupt");
        Serial1.println("ready");
        uint8_t dataInBuf[RH_MAX_MESSAGE_LEN];
        uint8_t dataInBufLen = 0;
        uint16_t waitCount = 0;
        // wait for Teensy to be ready and to send some data
        while(Serial1.available() || digitalRead(TEENSY_IRQ_IN) == HIGH{
            // if the Teensy is ready, read the data
            while(!Serial1.available()){
                waitCount++;
                delay(10);
                if(waitCount > 1000){
                    // if we get here there was a timeout waiting for data and both micros should abort
                    Serial.println("Timeout waiting for Teensy data");
                    Serial1.println("xxxxxxx");
                }
            }
            if(Serial1.available()){
                dataInBuf[dataInBufLen++] = Serial1.read();
            }
        }
        if(dataInBufLen > 0){
            Serial.println("Data received from Teensy: ");
            Serial.println(dataInBuf, dataInBufLen);
            // Now we need to interpret the data

        }
    }

    // wait for a second before looping again 
    unsigned long loopEnd = millis();
    if(loopEnd - loopStart < 1000){
        delay(1000 - (loopEnd - loopStart));
    }
}

bool sendDataToTeensy(String data){
    Serial.println("Sending data to Teensy...");
    String dataInStr = "";
    uint16_t waitCount = 0;
    digitalWrite(TEENSY_IRQ_OUT, LOW);
    // wait for Teensy to be ready
    while(!Serial1.available()){
        delay(10);
        waitCount++;
        if(waitCount > 1000){
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
        Serial1.println(data);
        Serial.println("Data sent to Teensy");
    }
    digitalWrite(TEENSY_IRQ_OUT, HIGH);
    return true;
}

bool sendDataToTeensy(uint8_t *data, uint8_t len){
    Serial.println("Sending data to Teensy...");
    String dataInStr = "";
    uint16_t waitCount = 0;
    digitalWrite(TEENSY_IRQ_OUT, LOW);
    // wait for Teensy to be ready
    while(!Serial1.available()){
        delay(10);
        waitCount++;
        if(waitCount > 1000){
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

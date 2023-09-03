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

#define NODE_MAX_STALENESS (1000 * 60 * 5) // 5 minutes

// calculate free RAM
extern "C" char *sbrk(int i);
size_t freeRAM() {
  char stack_dummy = 0;
  return &stack_dummy - sbrk(0);
}

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

RHDatagram radio(rf95); // starts datagram manager with default address of 0

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

radioRecvMesArray recvBuf; // this is the buffer that the radio will use to store incoming messages
uint8_t recvBufLen = RH_RF95_MAX_MESSAGE_LEN; // this is the length of the buffer that the radio will use to store incoming messages
uint8_t fromAddr; // this is the address of the node that sent the message
uint8_t dataOutBuf[RH_RF95_MAX_MESSAGE_LEN]; // this is the buffer that the radio will use to store outgoing messages
//uint8_t maxAddrFound = 1; // this is the highest address that has been found on the mesh
uint8_t toAddr; // this is the address that the message is being sent to
uint8_t recvFromId; // this is the id of the node that sent the message
uint8_t recvFlags; // these are the flags that were set when the message was sent
uint8_t address; // this is the address of this node
uint8_t routes[256]; // this is the array that will store the routes to the nodes
int16_t rssi[256]; // this is the array that will store the RSSI of the last packet received from the node
unsigned long loopStart = millis(); // this is the time that the loop started
unsigned long loopEnd = millis(); // this is the time that the loop ended
unsigned long timeSinceMeshUpdate = millis(); // this is the time since the last mesh update was received
unsigned long timeSinceLastBattUpdate = millis(); // this is the time since the last battery update was sent
uint8_t serialTeensyRecvBuf[1024]; // this is the buffer that will store incoming data from the Teensy
uint16_t serialTeensyRecvBufLen = sizeof(serialTeensyRecvBuf); // this is the length of the buffer that will store incoming data from the Teensy
bool debugEnabled = false; // this is the flag that determines if debug mode is enabled or not
bool loneModule = false; // this is the flag that determines if this is the only module or not

class Node{
private:
    // the address of the node
    int nodeAddress;
    // the next hop to the destination
    int routeNextHop;
    int routeHopCount;
    bool isDirectlyConnected;
    unsigned long staleness; // the number of milliseconds since the last message was received from this node
    // when stalenness reaches a certain value, the routeNextHop and routeHopCount should be reset

    int16_t lastRSSI;
    int lastSNR;
public:
    // default constructor
    Node(){
        routeNextHop = -1;
        isDirectlyConnected = false;
        staleness = 0;
    }
    // constructor that takes the address of the node
    Node(int addr){
        nodeAddress = addr;
        routeNextHop = -1;
        isDirectlyConnected = false;
        staleness = 0;
    }
    // this function will return the address of the node
    int getNodeAddress(){
        return nodeAddress;
    }
    // this function will return the next hop to the destination
    int getNextHop(){
        return routeNextHop;
    }

    int getHopCount(){
        return routeHopCount;
    }
    void setHopCount(int hopCount){
        routeHopCount = hopCount;
        updateStaleness();
    }
    // this function will update the routeNextHop variable
    void updateNextHop(int nextHop){
        routeNextHop = nextHop;
        updateStaleness();
    }
    bool getIsDirectlyConnected(){
        return isDirectlyConnected;
    }
    void setIsDirectlyConnected(bool isDirectlyConnected_){
        isDirectlyConnected = isDirectlyConnected_;
        if(isDirectlyConnected){
            routeNextHop = -1;
            routeHopCount = 0;
        }
        updateStaleness();
    }

    void updateStaleness(){
        staleness = millis();
    }

    unsigned long getStaleness(){
        return millis() - staleness;
    }

    bool isStale(){
        return millis() - staleness > NODE_MAX_STALENESS;
    }

    void updateRSSI(int16_t rssi){
        lastRSSI = rssi;
        updateStaleness();
    }

    void updateSNR(int snr){
        lastSNR = snr;
        updateStaleness();
    }

    int16_t getLastRSSI(){
        return lastRSSI;
    }

    int getLastSNR(){
        return lastSNR;
    }
};

class Mesh{
private:
    std::vector<Node> nodes;
public:
    // default constructor
    Mesh(){
    }
    // constructor that takes the address of the first node
    Mesh(int addr){
        nodes.push_back(Node(addr));
    }
    // this function will add a node to the mesh
    // returns the number of nodes in the mesh or -1 if the node is already in the mesh
    int addNode(int addr){
        if(getNodeIndex(addr) != -1){
            // if the node is already in the mesh, return -1
            return -1;
        }
        nodes.push_back(Node(addr));
        //  return the size of the vector
        return nodes.size() - 1;
    }
    // this function will return the address of the node at the given index
    int getNodeAddress(int index){
        return nodes.at(index).getNodeAddress();
    }
    // this function will return the index of the node with the given address
    // or -1 if the node is not in the mesh
    int getNodeIndex(int addr){
        for(size_t nodeInd = 0; nodeInd < nodes.size(); nodeInd++){
            if(nodes.at(nodeInd).getNodeAddress() == addr){
                return nodeInd;
            }
        }
        return -1;
    }
    // this function will remove a node from the mesh
    int removeNode(int index){
        nodes.erase(nodes.begin() + index);
        return nodes.size();
    }
    // this function takes the address of a node and returns the next hop to that node if
    // this node is on the route to that node or -1 if there isn't a route to that node.
    // Returns -2 if the destination is directly connected to this node.
    int getNextHop(int destination){
        // find the next hop to the destination
        for(size_t nodeInd = 0; nodeInd < nodes.size(); nodeInd++){
            if(nodes.at(nodeInd).getNodeAddress() == destination){
                return nodes.at(nodeInd).getIsDirectlyConnected()?-2:nodes.at(nodeInd).getNextHop();
            }
        }
        // if we get here, the destination is not in the routeToNode array
        return -1;
    }

    Node getNode(int index){
        return nodes.at(index);
    }
};


// instantiate the mesh using the maxAddrFound as the address for the first node.
// The first node in the vector will be this device.
static Mesh mesh;

void setup() {
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
        // set address to random value between 1 and 255
        address = random(1, 255);
    }else{
        // set address using address pins
        address = (digitalRead(ADDR_PIN_0) << 0) | (digitalRead(ADDR_PIN_1) << 1) | (digitalRead(ADDR_PIN_2) << 2) | (digitalRead(ADDR_PIN_3) << 3) | (digitalRead(ADDR_PIN_4) << 4) | (digitalRead(ADDR_PIN_5) << 5));
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
    
    // TODO: rewrite the radio setup here because we don't need to find other nodes when we set the address using the address pins

    if (!radio.waitAvailableTimeout(45000)) { // if after 45 seconds nothing is received, assume there is no mesh
        Serial.println("No other nodes found. Setting address = 1");
        //Serial1.println("addr=1");
        dataToSendToTeensy = dataToSendToTeensy + "addr=1";
        radio.setThisAddress(address);
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

            if (recvBufLen == MESH_BEACON_LENGTH && recvBuf[0] == MESH_BEACON_PACKET_TYPE && toAddr == RH_BROADCAST_ADDRESS) {
                Serial.print("Max address found: 0x");
                Serial.println(recvBuf[1], HEX);
                maxAddrFound = recvBuf[1] + 1;
                Serial1.println("addr=" + String(address));
                dataToSendToTeensy = dataToSendToTeensy + "addr=" + String(address);
                radio.setThisAddress(address);
                meshStatusRecvd = true;
            }else{
                Serial.println("Received packet was not a mesh status update. Ignoring.");
                Serial.println("Waiting for another packet...");
            }
        }
    }

    Serial.println("Radio setup complete. Continuing...");

    // instantiate the mesh using the maxAddrFound as the address for the first node.
    // The first node in the vector will be this device.
    mesh = Mesh(address);

    if(!loneModule){
        // next thing to do is to wait for the Teensy to finish setup. This is done by
        // waiting for the IRQ pin to go low.

        // if the Teensy gets through its setup, it will set the IRQ pin to low and
        // wait for the Feather to set its IRQ pin to low.
        Serial.println("Give the Teensy time to start...");
        //delay(1000); // give the Teensy time to start up
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
    }
    Serial.println("Feather setup complete. Continuing.");
    Serial.println("Free memory: " + String(freeRAM()));
}

void loop() {
    //Serial.println("Loop Start\nFree memory: " + String(freeRAM()));
    loopStart = millis();
    // first thing to do is send out a broadcast that indicates basic mesh status.
    // each node should do this so that in the event a segment of the mesh gets seperated,
    // and another node connects to the seperated section, all nodes will connect with
    // no overlapping addresses.
    // maxAddrFound - the highest addr of all the nodes
    uint8_t dataOutBufLen = 0;
    if(millis() - timeSinceMeshUpdate > 15000){ // Only send out a mesh beacon every 15 seconds
        timeSinceMeshUpdate = millis(); // reset the timer
        dataOutBuf[dataOutBufLen++] = MESH_BEACON_PACKET_TYPE; // mesh status flag
        dataOutBuf[dataOutBufLen++] = 0; // the max address found
        Serial.println("Broadcasting mesh Beacon..."); // debug
        radio.sendto(dataOutBuf, MESH_BEACON_LENGTH, RH_BROADCAST_ADDRESS); // send the packet
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
            //digitalWrite(BATT_RELAY_PIN, LOW); // turn off the battery
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
        if (recvBufLen == MESH_BEACON_LENGTH && recvBuf.messageType == MESH_BEACON_PACKET_TYPE && toAddr == RH_BROADCAST_ADDRESS) { // if the packet is a mesh status update
            Serial.println("Mesh status update received. Updating max addr"); // debug
            Serial.print("Max address: 0x");
            Serial.println(recvBuf[1], HEX);
            
            int16_t rssi = rf95.lastRssi(); // get the RSSI of the last packet received
            int snr = rf95.lastSNR(); // get the SNR of the last packet received

            // add the node to the mesh
            if(mesh.addNode(fromAddr) == -1){
                Serial.println("Address already in the mesh.");
            }else{
                Serial.println("Node added to mesh");
            }
            int nodeInd = mesh.getNodeIndex(fromAddr);
            if(nodeInd != -1){
                mesh.getNode(nodeInd).updateRSSI(rssi);
                mesh.getNode(nodeInd).updateSNR(snr);
                mesh.getNode(nodeInd).setIsDirectlyConnected(true);
            }
            

            // if the new max addr is greater than the current max addr, update it
            if(recvBuf[1] > maxAddrFound){
                maxAddrFound = recvBuf[1];
            }
            
            // need to send an update to the Teensy about RSSI and SNR
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
            if(!sendDataToTeensy("meshStatusUpdate:rssi=" + String(rssi) + ";snr=" + String(snr))){ // send the data to the Teensy
                Serial.println("Error sending mesh status update to Teensy");
            }
        }else if(recvBuf.messageType == (MESH_BEACON_PACKET_TYPE & REPEATER_ACTION_REQUEST_PACKET_TYPE)  && toAddr == RH_BROADCAST_ADDRESS){ // repeater action request packet received
            int16_t rssi = rf95.lastRssi(); // get the RSSI of the last packet received
            int snr = rf95.lastSNR(); 
            int nodeInd = mesh.getNodeIndex(fromAddr);
            if(nodeInd != -1){
                mesh.getNode(nodeInd).updateRSSI(rssi);
                mesh.getNode(nodeInd).updateSNR(snr);
                mesh.getNode(nodeInd).setIsDirectlyConnected(true);
            }
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
            if(!sendDataToTeensy("repeaterActionRequest:from=" + String(fromAddr) + ";to=" + String(toAddr) + ";id=" + String(recvFromId) + ";flags=" + String(recvFlags) + ";data=")){
                Serial.println("Failed to send data to Teensy");
            }
            if(!sendDataToTeensy(recvBuf, recvBufLen)){
                Serial.println("Failed to send data to Teensy");
            }
        }else if(recvBuf[0] == MESSAGE_PACKET_TYPE && toAddr == mesh.getNode(0).getNodeAddress()){ // message packet received
            int16_t rssi = rf95.lastRssi(); // get the RSSI of the last packet received
            int snr = rf95.lastSNR(); 
            int nodeInd = mesh.getNodeIndex(fromAddr);
            if(nodeInd != -1){
                mesh.getNode(nodeInd).updateRSSI(rssi);
                mesh.getNode(nodeInd).updateSNR(snr);
                mesh.getNode(nodeInd).setIsDirectlyConnected(true);
            }
            // This message, although sent to this device, may be destined for another device.
            // We need to check the destination address (recvBuf[1]) to see if it is this device or another device.
            // If it is this device, we need to send the message to the Teensy.
            // If it is another device, we need to send the message to the next hop.

            // first we need to check if the message is for this device
            if(recvBuf.destinationAddress == mesh.getNode(0).getNodeAddress()){ // This device
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
                if(!sendDataToTeensy("message:len=" + String(recvBufLen) + ";from=" + String(fromAddr) + ";to=" + String(toAddr) + ";id=" + String(recvFromId) + ";flags=" + String(recvFlags) + ";data=")){
                    Serial.println("Failed to send data to Teensy");
                }
                if(!sendDataToTeensy(recvBuf, recvBufLen)){
                    Serial.println("Failed to send data to Teensy");
                }
            }else{
                // if we get here, the message is not for this device
                // we need to send the message to the next hop
                // first we need to find the next hop
                int nextHop = mesh.getNextHop(recvBuf.destinationAddress);
                if(nextHop == -1){
                    // if we get here, there is no route to the destination
                    // we need to send a message to the Teensy to tell it that there is no route to the destination
                    // the message is sent as a packet with the following format:
                    // noRouteToDest:from=from:to=to:id=id:flags=flags:data=data
                    // where:
                    // from - the from address
                    // to - the to address
                    // id - the id of the message
                    // flags - the flags of the message
                    // data - the data of the message
                    // the message is sent to the Teensy as a packet.
                    // the Teensy will decide what to do with the message.
                    Serial.println("No route to destination. Sending to Teensy...");
                    if(!sendDataToTeensy("noRouteToDest:from=" + String(fromAddr) + ";to=" + String(toAddr) + ";id=" + String(recvFromId) + ";flags=" + String(recvFlags) + ";data=")){
                        Serial.println("Failed to send data to Teensy");
                    }
                    if(!sendDataToTeensy(recvBuf, recvBufLen)){
                        Serial.println("Failed to send data to Teensy");
                    }
                }else if(nextHop == -2){
                    // if we get here, the destination is directly connected to this node
                    // send it to the node over the radio
                    Serial.println("Destination is directly connected to this node. Sending to node...");
                    radio.sendto(recvBuf, recvBufLen, recvBuf.destinationAddress);
                }
            }

            // Once we have completed processing, make sure to we check that the return path exists as a route
            // to the sender. If it does not, we need to add it.

            // First, find the index of the origin node
            int originNodeInd = mesh.getNodeIndex(recvBuf.originAddress);
            // if the origin node is not in the mesh, add it
            if(originNodeInd == -1){
                Serial.println("Origin node not in mesh. Adding...");
                mesh.addNode(recvBuf.originAddress);
                originNodeInd = mesh.getNodeIndex(recvBuf.originAddress);
            }
            // Now we update the nextHop and hopCount of the origin node
            mesh.getNode(originNodeInd).updateNextHop(fromAddr);
            mesh.getNode(originNodeInd).setHopCount(recvBuf.hopCount + 1);
            
            
        }
    }

    uint16_t serialTeensyDataInBufLen = 0;

    if(digitalRead(TEENSY_IRQ_IN) == LOW){
        Serial.println("Data available from Teensy");
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
        Serial.println("Teensy sent some data");
        Serial.println("Free memory: " + String(freeRAM()));
        Serial.println("Data received from Teensy: ");
        
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
        String type = String((char*)serialTeensyRecvBuf,7);
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
                lengthStr += serialTeensyRecvBuf[indexOfData];
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
            Serial.println("Sending data to mesh...");
            if(!radio.sendto((uint8_t *)serialTeensyRecvBuf + indexOfData, length, RH_BROADCAST_ADDRESS)){
                Serial.println("Failed to send data to mesh");
            }
        }else if(type == "command"){
            String command = String((char*)serialTeensyRecvBuf + 8, serialTeensyDataInBufLen - 8);
            // now we need to check what the command is
            if(command.substring(0, 5) == "addr="){
                // the command is to set the address of the Feather
                address = command.substring(6).toInt();
                Serial.println("Address: " + String(address));
                Serial.println("Setting address...");
                radio.setThisAddress(address);
                Serial.println("Address set.");
        }
    }

    //Serial.println("Loop End\nFree memory: " + String(freeRAM()));
    // wait for a bit before looping again 
    loopEnd = millis();
    if(loopEnd - loopStart < 100){
        delay(100 - (loopEnd - loopStart));
    }
}

// function to send data to the Teensy over the serial port 
// returns true if the data was sent successfully, false otherwise
bool sendDataToTeensy(String data){
    //       The Teensy can use a large buffer for the serial port which will allow us to send data without using interrupts
    Serial.println("Sending data to Teensy..."); // debug
    Serial1.println(data); // send the data to the Teensy
    Serial.println("Data sent to Teensy"); // debug
    // return true to indicate that we successfully sent the data
    return true;
}

// a version of the sendDataToTeensy function that takes a uint8_t array and a length
bool sendDataToTeensy(uint8_t *data, uint8_t len){ // a version of the sendDataToTeensy function that takes a uint8_t array and a length
    Serial.println("Sending data to Teensy..."); // debug message
    Serial1.write(data, len); // send the data to the Teensy
    Serial1.println(); // send a newline character to the Teensy
    Serial.println("Data sent to Teensy"); // debug message
    return true; // return true to indicate that we successfully sent the data
}

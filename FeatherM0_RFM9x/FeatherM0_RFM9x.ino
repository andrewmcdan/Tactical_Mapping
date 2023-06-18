#include <arduino.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <RHDatagram.h>


// #elif defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)  // Feather M0 w/Radio
#define RFM95_CS    8
#define RFM95_INT   3
#define RFM95_RST   4
#define RF95_FREQ 915.0

#define MESH_STATUS_LENGTH 2
#define MAX_DIRECT_CONNECT_STALENESS 253

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

RHDatagram datagramManager(rf95); // starts datagram manager with default address of 0

uint8_t recvBuf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t recvBufLen = sizeof(recvBuf);
uint8_t fromAddr;
uint8_t data[256];
uint8_t maxAddrFound = 1;
uint8_t directConnectNodes[256]; // addresses of all nodes directly connected to this node
uint8_t directConnectNodesStaleness[256];
uint8_t directConnectNodesCount = 0;
uint8_t toAddr;
uint8_t recvFromId;
uint8_t recvFlags;

void setup() {
  for(uint8_t i = 0; i < 256; i++){
    directConnectNodesStaleness[i] = 255;
  }
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  while (!Serial) delay(1);
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
  while (!datagramManager.init()) {
    Serial.println("LoRa radio init failed... stopping.");
    //Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed... stopping");
    while (1);
  }
  Serial.print("Freq set to: ");
  Serial.println(RF95_FREQ);

  Serial.println("Adjusting modem mode...");
  if(!rf95.setModemConfig(RH_RF95::ModemConfigChoice::Bw125Cr45Sf2048)){
    Serial.println("Modem mode set failed... stopping.");
    while(1){}
  }
  Serial.println("Successfully set modem config");

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  Serial.print("Setting tramsit power...");
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
  Serial.println(" complete.");
  Serial.println("Start to find other nodes...");
  if(!datagramManager.waitAvailableTimeout(10000)){ // if after 10 seconds nothing is received, assume there is no mesh
    Serial.println("No other nodes found. Setting address = 1");
    datagramManager.setThisAddress(1);
  }else{
    // we need to get a mesh staus update in order to set this node's address, so keep looping until one comes in
    bool meshStatusRecvd = false;
    while(!meshStatusRecvd){
      datagramManager.waitAvailable(); // waiting for packet
      datagramManager.recvfrom(recvBuf, &recvBufLen, &fromAddr, &toAddr, &recvFromId, &recvFlags);
      Serial.println("Received data");
      Serial.print("From: 0x");
      Serial.println(fromAddr, HEX);
      Serial.print("To: 0x");
      Serial.println(toAddr, HEX);
      Serial.print("ID: 0x");
      Serial.println(recvFromId, HEX);
      Serial.print("Flags: 0x");
      Serial.println(recvFlags, HEX);
      
      if(recvBufLen == MESH_STATUS_LENGTH && recvBuf[0] == 0xff && toAddr == RH_BROADCAST_ADDRESS){
        Serial.print("Max address found: 0x");
        Serial.println(recvBuf[1], HEX);
        maxAddrFound = recvBuf[1] + 1;
        datagramManager.setThisAddress(maxAddrFound);
        meshStatusRecvd = true;
      }
      updateDirectConnectNodes(fromAddr); // This node is clearly connected to whichever node it received the mesh status from, so add it to the direct connected nodes
    }
    
  }
  Serial.println("Setup complete complete. Continuing.");
}

void loop() {
  // first thing to do is send out a broadcast that indicates basic mesh status.
  // each node should do this so that in the event a segment of the mesh gets seperated,
  // and another node connects to the seperated section, all nodes will connect with
  // no overlapping addresses.
  // maxAddrFound - the highest addr of all the nodes
  uint8_t dataLength = 0;
  data[dataLength++] = 0xff; // mesh status flag
  data[dataLength++] = maxAddrFound;
  datagramManager.sendto(data, dataLength, RH_BROADCAST_ADDRESS);
  dataLength = 0;

  // then receive any messages on the waves...

  delay(1000);
}

void updateDirectConnectNodes(uint8_t addr){
  // check if addr is new
  // if new, add to directConnectNodes and reset staleness to 0
  // if not new, reset staleness to 0
  // add 1 to staleness of all nodes.
  // check any node that has staleness of MAX_DIRECT_CONNECT_STALENESS or greater and remove that node from the array shifting all other elements in both arrays
}

bool isDirectConnectNode(uint8_t addr){
  // returns true if the addr is in directConnectNodes array
  return true;
}
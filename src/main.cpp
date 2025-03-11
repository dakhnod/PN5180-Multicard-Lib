#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <MDNS.h>

#define CHECKSUM_NONE      0b000
#define CHECKSUM_RX_PARITY 0b001
#define CHECKSUM_RX_CRC    0b010
#define CHECKSUM_TX_CRC    0b100

#define PIN_RST  5
#define PIN_SS   23
#define PIN_MOSI 19
#define PIN_MISO 22
#define PIN_SCK  21
#define PIN_BUSY 17
#define PIN_IRQ  16

uint8_t stuffing[64];
static AsyncWebServer server(80);

WiFiUDP udp;
MDNS mdns(udp);

bool wifiConnected = false;

EventGroupHandle_t irqEventGroup;

void hardReset();
void transmitSPI(uint8_t *data, int length, uint8_t *response, int response_length);
void loadRFParameters();
void writeRegister(uint8_t address, uint32_t value);
void readRegister(uint8_t address, uint32_t *value);
void setRegisterBits(uint8_t address, uint32_t mask);
void clearRegisterBits(uint8_t address, uint32_t mask);
void sendData(uint8_t data, int length);
void readData(uint8_t *response, int length);
void RFOn();
void RFOff();
void startTransceive();
void onWifiStateChange(arduino_event_id_t event);
void onIRQPin();

void setup() {
  Serial.begin(115200);

  irqEventGroup = xEventGroupCreate();

  Serial.onReceive([](){
    int higherPriorityTaskWokenUp;
    xEventGroupSetBitsFromISR(irqEventGroup, 0b100, &higherPriorityTaskWokenUp);
  }, false);

  memset(stuffing, 0xff, sizeof(stuffing));

  pinMode(PIN_RST,  OUTPUT);
  pinMode(PIN_SS,   OUTPUT);
  pinMode(PIN_BUSY, INPUT);
  pinMode(PIN_IRQ,  INPUT);

  hardReset();

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);

  digitalWrite(PIN_SS, HIGH);


  uint8_t writeReference[] = {0x06, 0x34, 0}; // gear number
  transmitSPI(writeReference, 3, NULL, 0);

  uint8_t writeThreshold[] = {0x06, 0x37, 15}; // threshold
  transmitSPI(writeThreshold, 3, NULL, 0);

  uint8_t writeConfig[] = {0x06, 0x38, 0x01}; // LPCD mode
  transmitSPI(writeConfig, 3, NULL, 0);
  
  WiFi.onEvent(onWifiStateChange);
  WiFi.begin("DoubleD", "DoubleD3141");

  server.on("/index.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "POST /rf_on /rf_off /transceive");
  });

  server.on("/rf_on", HTTP_POST, [](AsyncWebServerRequest *request){
    RFOn();
    request->send(200);
  });

  server.on("/rf_off", HTTP_POST, [](AsyncWebServerRequest *request){
    RFOff();
    request->send(200);
  });

  server.on("/transceive", HTTP_POST, [](AsyncWebServerRequest *request){
    return 0;
  });

  server.onRequestBody([](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
    if(request->url() != "/transceive") {
      return;
    }

    int responseLength = request->getHeader("Response-Length")->toString().toInt();

    uint8_t response[responseLength];

    Serial.printf("len: %d, index: %d, total: %d\n", len, index, total);

    request->send(200, "text/plain", "thanks!");
  });

  server.begin();

  attachInterrupt(PIN_IRQ, onIRQPin, RISING);

  delay(1000);
}

void onIRQPin(){
  BaseType_t higherPriorityTaskWasWoken = false;
  xEventGroupSetBitsFromISR(irqEventGroup, 0b01, &higherPriorityTaskWasWoken);
}

void onBusyPin(){
  BaseType_t higherPriorityTaskWasWoken = false;
  xEventGroupSetBitsFromISR(irqEventGroup, 0b10, &higherPriorityTaskWasWoken);
}

void onWifiStateChange(arduino_event_id_t event) {
  if(event != ARDUINO_EVENT_WIFI_STA_GOT_IP) {
    return;
  }

  Serial.println(WiFi.localIP());
  mdns.begin(WiFi.localIP(), "nfc");
  wifiConnected = true;
}

void hardReset(){
  digitalWrite(PIN_RST, LOW);
  delay(1);
  digitalWrite(PIN_RST, HIGH);
}

void startTransceive(){
  setRegisterBits(0x00, 0x03);
}

void stopTransceive(){
  clearRegisterBits(0x00, 0x07);
}

void configureChecksum(int flags){
  if(flags & CHECKSUM_TX_CRC) {
    setRegisterBits(0x19, 0b001001);
  }else{
    clearRegisterBits(0x19, 0b01);
  }

  if(flags & CHECKSUM_RX_PARITY) {
    writeRegister(0x12, 0b110000001000);
  }else if(flags & CHECKSUM_RX_CRC) {
    writeRegister(0x12, 0b110000001001);
  }else{
    clearRegisterBits(0x12, 0b110000001001);
  }
}

void enableLPCD(uint16_t interval){
  uint8_t command[4] = {0x0B, 0x01};
  memcpy(command + 2, &interval, 2);

  transmitSPI(command, 4, NULL, 0);
}

void printArray(uint8_t *data, int length, std::string label) {
  Serial.printf("%s: ", label.c_str());
  for(int i = 0; i < length; i++) {
    Serial.printf("%02x ", data[i]);
  }
  Serial.println();
}

void sendData(uint8_t *data, int length, uint8_t lastByteBits) {
  uint8_t payload[length + 2] = {0x09, lastByteBits};

  // uint8_t payload[length + 1] = {0x08};

  memcpy(payload + 2, data, length);

  transmitSPI(payload, length + 2, NULL, 0);
}

void readData(uint8_t *response, int length) {
  uint8_t payload[] = {0x0a, 0x00};

  transmitSPI(payload, 2, response, length);
}

void awaitBusyState(int state) {
  xEventGroupClearBits(irqEventGroup, 0x02);
  attachInterrupt(PIN_BUSY, onBusyPin, state ? RISING : FALLING);
  if(digitalRead(PIN_BUSY) != state) {
    xEventGroupWaitBits(irqEventGroup, 0x02, true, false, 1000);
  }
  xEventGroupClearBits(irqEventGroup, 0x02);
  detachInterrupt(PIN_BUSY);
}

void clearIRQ(){
  uint32_t data = 0x000FFFFF;

  writeRegister(0x03, data);
}

void RFOn(){
  uint8_t command[] = {0x16, 0x00};
  transmitSPI(command, 2, NULL, 0);
}

void RFOff(){
  uint8_t command[] = {0x17, 0x00};
  transmitSPI(command, 2, NULL, 0);
}

void loadRFParameters(){
  uint8_t command[] = {0x11, 0x00, 0x80};
  transmitSPI(command, 3, NULL, 0);
}

void readRegister(uint8_t address, uint32_t *response) {
  uint8_t command[] = {0x04, address};

  transmitSPI(command, 2, (uint8_t*)response, 4);
}


void setRegisterBits(uint8_t address, uint32_t mask) {
  uint8_t command[] = {
    0x01,
    address, 
    (uint8_t)(mask >> 0 ),
    (uint8_t)(mask >> 8 ),
    (uint8_t)(mask >> 16),
    (uint8_t)(mask >> 24),
  };

  transmitSPI(command, 6, NULL, 0);
}


void clearRegisterBits(uint8_t address, uint32_t mask) {
  mask = ~mask;
  uint8_t command[] = {
    0x02,
    address, 
    (uint8_t)(mask >> 0 ),
    (uint8_t)(mask >> 8 ),
    (uint8_t)(mask >> 16),
    (uint8_t)(mask >> 24),
  };

  transmitSPI(command, 6, NULL, 0);
}

void writeRegister(uint8_t address, uint32_t value) {
  uint8_t command[] = {
    0x00,
    address, 
    (uint8_t)(value >> 0 ),
    (uint8_t)(value >> 8 ),
    (uint8_t)(value >> 16),
    (uint8_t)(value >> 24),
  };

  transmitSPI(command, 6, NULL, 0);
}

void transmitSPI(uint8_t *data, int length, uint8_t *response, int response_length) {
  awaitBusyState(LOW);

  digitalWrite(PIN_SS, LOW);

  delay(2);

  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  SPI.writeBytes(data, length);

  awaitBusyState(HIGH);

  digitalWrite(PIN_SS, HIGH);

  awaitBusyState(LOW);

  if(response_length == 0) {
    SPI.endTransaction();
    return;
  }

  memset(response, 0xFF, response_length);

  digitalWrite(PIN_SS, LOW);

  SPI.transferBytes(stuffing, response, response_length);

  awaitBusyState(HIGH);

  digitalWrite(PIN_SS, HIGH);

  awaitBusyState(LOW);

  SPI.endTransaction();
  digitalWrite(PIN_SS, HIGH);
}

void prepareIRQ(uint32_t expected) {
  clearIRQ();

  writeRegister(0x01, expected);

  xEventGroupClearBits(irqEventGroup, 0x01);
}

int awaitIRQ(uint32_t expected, unsigned int timeout){
  uint32_t irq_status = 0;
  #if 0
  while((irq_status & expected) == 0) {
    // Serial.printf("IRQ: %x\n", irq_status);
    readRegister(0x02, &irq_status);
    delay(1);
  }
  return irq_status;
  #endif

  // clearIRQ();

  xEventGroupWaitBits(irqEventGroup, 0x01, true, false, timeout);

  readRegister(0x02, &irq_status);

  if(irq_status & expected) {
    return irq_status;
  }
  return -1;
}



int transceive(uint8_t *data, int txLength, int lastByteBits, uint8_t *response, int *responseLength, int *lastReceivedByteBits, int *collisionPosition) {
  stopTransceive();
  startTransceive();

  prepareIRQ(0x01);
  sendData(data, txLength, lastByteBits);
  int irq = awaitIRQ(0x01, 10);
  clearIRQ();

  stopTransceive();

  if(irq == -1) {
    return -1;
  }

  uint32_t rx_state;

  readRegister(0x13, &rx_state);

  *responseLength = rx_state & 0xFF;
  if(lastReceivedByteBits != NULL) {
    *lastReceivedByteBits = (rx_state >> 13) & 0b111;
  }

  if(collisionPosition != NULL) {
    if(rx_state & (1 << 18)) {
      *collisionPosition = (rx_state >> 19) & 0b1111111;
    }else {
      *collisionPosition = -1;
    }
  }

  readData(response, *responseLength);

  return 0;
}

void readEEPROM(uint8_t startAddress, uint8_t *data, uint8_t length) {
  uint8_t readCommand[] = {0x07, startAddress, length};
  transmitSPI(readCommand, 3, data, length);
}

void awaitSerialKeypress() {
  while(Serial.read() != -1);
  xEventGroupClearBits(irqEventGroup, 0b100);
  xEventGroupWaitBits(irqEventGroup, 0b100, true, false, portMAX_DELAY);
}

int transceiveATQ(uint16_t *ATQA) {
  configureChecksum(CHECKSUM_RX_PARITY);

  uint8_t ATQ = 0x26;
  int length;
  int result = transceive(&ATQ, 1, 7, (uint8_t*) ATQA, &length, NULL, NULL);

  if(result == -1) {
    return -1;
  }
  if(length != 2) {
    return -1;
  }

  return 0;
}

void setReceiveBitOffset(int offset) {
  clearRegisterBits(0x12, 0b111 << 6);
  if(offset == 0) {
    return;
  }
  setRegisterBits(0x12, (offset & 0b111) << 6);
}

int selectCard(uint8_t UID[10], int *uidLength){
  uint8_t SEL[7];

  int cascadeLevel = 0;

  SEL[1] = 0x70;

  uint8_t response[7];

  int responseCount;

  *uidLength = 0;

  int collisionPosition;
  
  for(int cascadeLevel = 0; cascadeLevel < 3; cascadeLevel++) {
    SEL[0] = 0x93 + (cascadeLevel * 2);
    SEL[1] = 0x20;

    configureChecksum(CHECKSUM_RX_PARITY);
    int result = transceive(SEL, 2, 0, response, &responseCount, NULL, &collisionPosition);

    if(collisionPosition != -1) {
      Serial.printf("Collision position: %d\n", collisionPosition);
      return -1;
    }
  
    if(result == -1) {
      return -1;
    }

    memcpy(SEL + 2, response, 5);

    #if 1
    uint8_t partialResponse[7];
    int bits = 6;
    SEL[1] = 0x20 | bits;
    int lastReceivedByteBits;
    setReceiveBitOffset(bits);
    result = transceive(SEL, 3, bits, partialResponse, &responseCount, &lastReceivedByteBits, NULL);
    setReceiveBitOffset(0);

    if(result == -1) {
      return -1;
    }

    Serial.printf("response: %d, last bits: %d\n", result, lastReceivedByteBits);
    printArray(partialResponse, responseCount, "partial response");
    #endif

    SEL[1] = 0x70;
    uint8_t SAK;

    configureChecksum(CHECKSUM_TX_CRC | CHECKSUM_RX_CRC);
    result = transceive(SEL, 7, 0, &SAK, &responseCount, NULL, NULL);
    Serial.printf("SAK: %02x\n", SAK);

    if(result == -1) {
      return -1;
    }

    if((SAK & 0x04) == 0x00) {
      memcpy(UID + (*uidLength), response, 4);
      *uidLength += 4;
      break;
    }

    memcpy(UID + (*uidLength), response + 1, 3);
    *uidLength += 3;
  }

  return 0;
}

void loop() {
  if(wifiConnected) {
    mdns.run();
  }

  #if 01
  awaitSerialKeypress();
  #endif

  #if 0
  loadRFParameters();
  while(true){
    uint32_t agcConfig;
    readRegister(0x26, &agcConfig);

    Serial.printf("DPC gear: %d, AGC value: %d\n", (agcConfig >> 10) & 0b1111, agcConfig & 0b1111111111);

    delay(100);
  }
  #endif

  #if 0
  loadRFParameters();
  RFOn();
  while(true){
    uint32_t rxInfo;
    readRegister(0x1D, &rxInfo);

    Serial.printf("AGC gear: %d, AGC value: %d\n", (rxInfo >> 20) & 0b1111, rxInfo & 0b1111111111);

    delay(100);
  }
  #endif

  #if 0
  uint32_t agcConfig;
  readRegister(0x26, &agcConfig);

  writeRegister(0x26, agcConfig);
  #endif

  #if 0
  prepareIRQ(1 << 19);

  Serial.printf("waiting for LPCD...\n");

  enableLPCD(100);

  int irq = awaitIRQ(1 << 19, 9999999);
  Serial.printf("detected card IRQ %x\n", irq);
  #endif

  Serial.println("Reading card...");

  loadRFParameters();

  uint16_t ATQA;

  RFOn();

  int result = transceiveATQ(&ATQA);

  if(result < 0) {
    Serial.println("ATQA error");
    // RFOff();
    // return;
  }

  Serial.printf("ATQA: %x\n", ATQA);

  uint8_t uid[10];
  int uidLength;

  // enableTXCRC();

  result = selectCard(uid, &uidLength);

  if(result < 0) {
    Serial.println("select error");
  }

  printArray(uid, uidLength, "UID");

  RFOff();
}
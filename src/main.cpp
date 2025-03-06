#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <MDNS.h>

#define PIN_RST  5
#define PIN_SS   23
#define PIN_MOSI 19
#define PIN_MISO 22
#define PIN_SCK  21
#define PIN_BUSY 17

uint8_t stuffing[64];
static AsyncWebServer server(80);

WiFiUDP udp;
MDNS mdns(udp);

bool wifiConnected = false;

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
void disableRXCRC();
void disableTXCRC();
void onWifiStateChange(arduino_event_id_t event);

void setup() {
  Serial.begin(115200);

  memset(stuffing, 0xff, sizeof(stuffing));

  pinMode(PIN_RST,  OUTPUT);
  pinMode(PIN_SS,   OUTPUT);
  pinMode(PIN_BUSY, INPUT);

  hardReset();

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);

  digitalWrite(PIN_SS, HIGH);

  loadRFParameters();

  
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
  delay(100);
  digitalWrite(PIN_RST, HIGH);
}

void startTransceive(){
  setRegisterBits(0x00, 0x03);
}

void stopTransceive(){
  clearRegisterBits(0x00, 0x07);
}

void disableTXCRC(){
  clearRegisterBits(0x19, 0x01);
}

void disableRXCRC(){
  clearRegisterBits(0x12, 0x01);
}

void enableTXCRC(){
  setRegisterBits(0x19, 0b001001);
}

void enableRXParity(){
  writeRegister(0x12, 0b110000001000);
}

void enableRXCRC(){
  writeRegister(0x12, 0b110000001001);
}

void printArray(uint8_t *data, int length, std::string label) {
  Serial.printf("%s: ", label.c_str());
  for(int i = 0; i < length; i++) {
    Serial.printf("%x ", data[i]);
  }
  Serial.println();
}

void sendData(uint8_t *data, int length) {
  uint8_t payload[length + 2] = {0x09, 0x00};

  if(length == 1) {
    payload[1] = 0x07;
  }

  // uint8_t payload[length + 1] = {0x08};

  memcpy(payload + 2, data, length);

  transmitSPI(payload, length + 2, NULL, 0);
}

void readData(uint8_t *response, int length) {
  uint8_t payload[] = {0x0a, 0x00};

  transmitSPI(payload, 2, response, length);
}

void awaitBusyState(int state) {
  while (state != digitalRead(PIN_BUSY)){
    // Serial.printf("Waiting BUSY to becomde %d\n", state);
    // delay(100);
  };
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
    (mask >> 0 ) & 0xFF,
    (mask >> 8 ) & 0xFF,
    (mask >> 16) & 0xFF,
    (mask >> 24) & 0xFF,
  };

  transmitSPI(command, 6, NULL, 0);
}


void clearRegisterBits(uint8_t address, uint32_t mask) {
  mask = ~mask;
  uint8_t command[] = {
    0x02,
    address, 
    (mask >> 0 ) & 0xFF,
    (mask >> 8 ) & 0xFF,
    (mask >> 16) & 0xFF,
    (mask >> 24) & 0xFF,
  };

  transmitSPI(command, 6, NULL, 0);
}

void writeRegister(uint8_t address, uint32_t value) {
  uint8_t command[] = {
    0x00,
    address, 
    (value >> 0 ) & 0xFF,
    (value >> 8 ) & 0xFF,
    (value >> 16) & 0xFF,
    (value >> 24) & 0xFF,
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

int waitForIRQ(int expected){
  uint32_t irq_status;

  for(int i = 0; true; i++) {
    readRegister(0x02, &irq_status);

    if(irq_status & expected) {
      return irq_status;
    } 

    if(i > 90) {
      return -1;
    }

    delay(1);
  }
}



int transceive(uint8_t *data, int txLength, uint8_t *response, int *responseLength) {
  stopTransceive();
  startTransceive();

  clearIRQ();

  sendData(data, txLength);

  int irq = waitForIRQ(0x01);
  clearIRQ();

  stopTransceive();

  if(irq == -1) {
    return -1;
  }

  uint32_t rx_state;

  readRegister(0x13, &rx_state);

  *responseLength = rx_state & 0xFF;

  readData(response, *responseLength);

  return 0;
}

void loop() {
  if(wifiConnected) {
    mdns.run();
  }

  if(!Serial.available()) {
    return;
  }

  while(Serial.read() != -1);

  Serial.println("Sending command...");

  uint8_t ATQ = 0x26;
  uint8_t response[9];

  int responseCount;

  RFOn();

  disableRXCRC();
  disableTXCRC();

  int result = transceive(&ATQ, 1, response, &responseCount);

  Serial.print("ATQA: ");
  if(result == -1) {
    Serial.println("Read timeout");
    RFOff();
    return;
  }

  printArray(response, responseCount, "ATQA");

  // enableTXCRC();

  uint8_t SEL[7];

  int cascadeLevel = 0;
  int uidLength;
  uint8_t uid[10];

  SEL[1] = 0x70;
  
  for(int cascadeLevel = 0; cascadeLevel < 3; cascadeLevel++) {
    SEL[0] = 0x93 + (cascadeLevel * 2);
    SEL[1] = 0x20;

    enableRXParity();
    disableTXCRC();
    result = transceive(SEL, 2, response, &responseCount);
  
    if(result == -1) {
      Serial.println("Read timeout");
      RFOff();
      return;
    }

    printArray(response, responseCount, "SEL");

    SEL[1] = 0x70;

    memcpy(SEL + 2, response, 5);

    enableRXCRC();
    enableTXCRC();
    result = transceive(SEL, 7, response, &responseCount);

    printArray(response, responseCount, "SEL");

    uint8_t SAK = response[0];

    if((SAK & 0x04) == 0x00) {
      Serial.println("SAK: ATSS");
      memcpy(uid + uidLength, response, 4);
      uidLength += 4;
      break;
    }

    memcpy(uid + uidLength, response + 1, 3);
    uidLength += 3;
  }

  printArray(uid, uidLength, "UID");

  RFOff();
}
#include "TinyGPS_GICom.h"
#define GPSBaud 115200
#define RXD2 16
#define TXD2 17
int flag4, flag5;

// Codigos de configuracion GPS
uint8_t offGxGGA[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x23};
uint8_t offGxGLL[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A};
uint8_t offGxGSA[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31};
uint8_t offGxGSV[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x38};
uint8_t UART1GxRMC[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x3F};
uint8_t offGxVTG[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46};
uint8_t cfgPM2[] =  {0xB5, 0x62, 0x06, 0x3B, 0x30, 0x00, 0x02, 0x06, 0x00, 0x00, 0x02, 0x91, 0x43, 0x01, 0x10, 0x27, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x4F, 0xC1, 0x03, 0x00, 0x87, 0x02, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x64, 0x40, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2D, 0x3F};
uint8_t cfgRXM[] =  {0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92}; //Receiver manager: 1-Power save mode
uint8_t baudUART[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x7E};//115200
uint8_t saveCfg[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x31, 0xBF};


TinyGPSPlus gps;



//$EIGPQ,RMC*3A\r\n
byte gps_set_sucess = 0;

void setup() {
  pinMode(4, INPUT);
  Serial.begin(115200);
  Serial2.begin(115200);

  // Wait for hardware to initialize
  delay(1000); Serial2.flush();

  //Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);


  //cfnGPS();
//restartGps();

//HOLA
offGps();

//gps apagado
}



void loop() {

/*
  if (digitalRead(4) == 1 && flag4 == 0) {
    flag4 = 1;
    Serial.print(digitalRead(4)); Serial.println("cnfGPS");

    //Set GPS to backup mode (sets it to never wake up on its own)
    //uint8_t GPSoff[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x4D, 0x3B};

  }
  if (digitalRead(4) == 0) {
    flag4 = 0;
  }

  if (digitalRead(5) == 1 && flag5 == 0) {
    flag5 = 1;
    Serial.print(digitalRead(5)); Serial.println("Send");
    Serial2.write("$EIGPQ,RMC*3A\r\n");//peticion de datos al GPS
  }
  if (digitalRead(5) == 0) {
    flag5 = 0;
  }


  while (Serial2.available() > 0) {
    if (gps.encode(Serial2.read())) {
      displayInfo();
    }
  }


  delay(100);*/
}

// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for (int i = 0; i < len; i++) {
    Serial2.write(MSG[i]);
    Serial.print(MSG[i], HEX);
  }
  Serial.println();
}


void offGps(){
  uint8_t GPSoff[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x4D, 0x3B};
 sendUBX(GPSoff, sizeof(GPSoff)/sizeof(uint8_t));
  }



void restartGps(){
  uint8_t GPSon[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x4C, 0x37};
 sendUBX(GPSon, sizeof(GPSon)/sizeof(uint8_t));
  }

void cfnGPS() {
  sendUBX(offGxGGA, sizeof(offGxGGA) / sizeof(uint8_t)); delay(50);getUBX_ACK(offGxGGA);
  sendUBX(offGxGLL, sizeof(offGxGLL) / sizeof(uint8_t)); delay(50);
  sendUBX(offGxGSA, sizeof(offGxGSA) / sizeof(uint8_t)); delay(50);
  sendUBX(offGxGSV, sizeof(offGxGSV) / sizeof(uint8_t)); delay(50);
  sendUBX(UART1GxRMC, sizeof(UART1GxRMC) / sizeof(uint8_t)); delay(50);
  sendUBX(offGxVTG, sizeof(offGxVTG) / sizeof(uint8_t)); delay(50);
  sendUBX(cfgPM2, sizeof(cfgPM2) / sizeof(uint8_t)); delay(50);
  sendUBX(cfgRXM, sizeof(cfgRXM) / sizeof(uint8_t)); delay(50);
  sendUBX(baudUART, sizeof(baudUART) / sizeof(uint8_t)); delay(50);

  Serial2.flush();
  delay(2);
  Serial2.end();
  delay(500);
  //Start the serial port and wait for it to initialize
  Serial2.begin(115200);
  delay(3000);
  
  sendUBX(saveCfg, sizeof(saveCfg) / sizeof(uint8_t)); delay(50);
  getUBX_ACK(saveCfg);
}


// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
  Serial.print(" * Reading ACK response: ");

  // Construct the expected ACK packet
  ackPacket[0] = 0xB5;  // header
  ackPacket[1] = 0x62;  // header
  ackPacket[2] = 0x05;  // class
  ackPacket[3] = 0x01;  // id
  ackPacket[4] = 0x02;  // length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];  // ACK class
  ackPacket[7] = MSG[3];  // ACK id
  ackPacket[8] = 0;   // CK_A
  ackPacket[9] = 0;   // CK_B

  // Calculate the checksums
  for (uint8_t i = 2; i < 8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }
  while (1) {
    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      Serial.println(" (SUCCESS!)");
      return true;
    }
    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) {
      Serial.println(" (FAILED!)");
      return false;
    }
    // Make sure data is available to read
    if (Serial2.available()) {
      b = Serial2.read();
      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) {
        ackByteID++;
        Serial.print(b, HEX);
      }
      else {
        ackByteID = 0;  // Reset and look again, invalid order
      }
    }
  }
}


void displayInfo() {
  Serial.print(F("Location: "));
  if (gps.location.isValid()) {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid()) {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid()) {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}

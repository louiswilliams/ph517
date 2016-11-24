#include "Arduhdlc.h"

#define HDLC_MAX_FRAME_LEN 16

typedef struct {
  volatile uint32_t rpm;
  volatile uint32_t pulses;
  volatile uint32_t timeOn; 
} EngineFrame;

EngineFrame engineData = {0, 0, 0};

Arduhdlc hdlc = Arduhdlc(&hdlcSendChar, &hdldRecvFrame, HDLC_MAX_FRAME_LEN);

// Send data
void hdlcSendChar(uint8_t data) {
  Serial1.write((char)data);
}

void hdldRecvFrame(const uint8_t* data, uint16_t len) {
  if (len == sizeof(EngineFrame)) {
    memcpy(&engineData, data, sizeof(EngineFrame));
    Serial.print("rpm: ");
    Serial.println(engineData.rpm, DEC);
    Serial.print("pulses: ");
    Serial.println(engineData.pulses, DEC);
    Serial.print("timeOn: " );
    Serial.println(engineData.timeOn, DEC); 
  } else {
    Serial.println("Received data of length " + len);
  }
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
   while (Serial1.available()) {
    hdlc.charReceiver((char) Serial1.read());
  }
}



#include "InputOutput.h"

/*
 class DataInput
 */
bool DataInput::isSwitchPressed(uint8_t i) {
  return modeSwitches & BUTTON(i);
}

// Parse engine data from raw data and store in engineData
// The data comes in little-endian (machine) order
void DataInput::setEngineData(const uint8_t* data, uint16_t length) {
  if (length == sizeof(engine)) {
    memcpy(&engine, data, length);
    Serial.print("rpm: ");
    Serial.println(engine.rpm, DEC);
    Serial.print("pulses: ");
    Serial.println(engine.pulses, DEC);
    Serial.print("timeOn: " );
    Serial.println(engine.timeOn, DEC); 
    // setChar(ENGINE_RPM, engine.rpm);
    // setChar(ENGINE_PULSES, engine.pulses);
    // setChar(ENGINE_TIMEON, engine.timeone);
  } else {
    Serial.println("Received data of unknown length: " + length);
  }
}

/*
 * class DataOutput
 */
bool DataOutput::isLedOn(uint8_t i) {
  return modeLEDs & BUTTON(i);
}

bool DataOutput::setLedOn(uint8_t i, bool enable) {
  if (enable) {
    modeLEDs |= BUTTON(i);
  } else {
    modeLEDs &= (0xFF - BUTTON(i));
  }
}

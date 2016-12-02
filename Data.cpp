#include "InputOutput.h"

/*
 class DataInput
 */
bool DataInput::isSwitchPressed(uint8_t i) const {
  return modeSwitches & BUTTON(i);
}

// Parse engine data from raw data and store in engineData
// The data comes in little-endian (machine) order
void DataInput::setEngineData(const uint8_t* data, uint16_t length) {
  if (length == sizeof(engine)) {
    memcpy(&engine, data, length);
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
bool DataOutput::isLedOn(uint8_t i) const {
  return modeLEDs & BUTTON(i);
}

bool DataOutput::setLedOn(uint8_t i, bool enable) {
  if (enable) {
    modeLEDs |= BUTTON(i);
  } else {
    modeLEDs &= (0xFF - BUTTON(i));
  }
}

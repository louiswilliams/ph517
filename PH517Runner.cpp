/**
  @author Louis Williams
  @date 2016

  PH517 Controls Runner
**/

#include "PH517Runner.h"

PH517Runner::PH517Runner(InputOutput& io) {
  _inputs = {};
  _outputs = {};
  _io = io;
}

bool PH517Runner::step() {

  // Collect data into InputData
  collectInputs(_inputs);
  // Get OutputData from the InputData
  processInputs(_inputs, _outputs);
  // Send OutputData to hardware
  sendOutputs(_outputs);

  // TODO: Beacon to UI Tablet
  delay(10);
  return true;
}

// Collect data from hardware into the inputs structure 
void PH517Runner::collectInputs(DataInputs& inputs) {
  inputs.throttle = _io.readThrottle();
  inputs.brake = _io.readBrake();
  inputs.battTemp = _io.readBattTemp();
  inputs.modeSwitches = _io.readModeSwitches();

  // Serial.println("--Inputs--");
  // Serial.println(" Throttle: " + inputs.throttle);
  // Serial.println(" Brake: " + inputs.brake);
  // Serial.println(" modeSwitches: " + inputs.modeSwitches);  
  // Serial.println(" engine.rpm: " + inputs.engine.rpm);  
  // Serial.println(" engine.fuelRate: " + inputs.engine.fuelRate);  

  _io.getBattData(inputs.batt);
  _io.getMotorData(inputs.motor);
}

// Parse incoming 
void PH517Runner::setEngineData(const uint8_t* data, uint16_t length) {
  _io.getEngineDataFromBuffer(data, length, _inputs.engine);
}

// Process inputs and return outputs from control block
void PH517Runner::processInputs(const DataInputs& inputs, DataOutputs& outputs) {
  outputs.engineServo = map(inputs.throttle, 875, 560, 0, 255);
  outputs.motorAccel = map(inputs.throttle, 875, 560, 0, 4095);

  // Starter
  // Button 1
  if (inputs.modeSwitches & 1) {
    digitalWrite(RELAY_POWER, LOW);
    digitalWrite(BUTTON_LED_1, LOW);
  } else {    
    digitalWrite(RELAY_POWER, HIGH);
    digitalWrite(BUTTON_LED_1, HIGH);
  }
  if (inputs.modeSwitches & 1<<1) {
    digitalWrite(BUTTON_LED_2, LOW);
  } else {    
    digitalWrite(BUTTON_LED_2, HIGH);
  }
  if (inputs.modeSwitches & 1<<2) {
    digitalWrite(BUTTON_LED_3, LOW);
  } else {    
    digitalWrite(BUTTON_LED_3, HIGH);
  }
  if (inputs.modeSwitches & 1<<3) {
    digitalWrite(RELAY_CRANK, LOW);
    digitalWrite(BUTTON_LED_4, LOW);
  } else {    
    digitalWrite(RELAY_CRANK, HIGH);
    digitalWrite(BUTTON_LED_4, HIGH);
  }



  // Serial.println("--Outputs--");
  // Serial.println(" engineServo: " + outputs.engineServo);
  // Serial.println(" motorAccel: " + outputs.motorAccel);  
}

// Actuate values to hardware components
void PH517Runner::sendOutputs(const DataOutputs& outputs) {
  _io.sendEngineAccel(outputs.engineServo);
  _io.sendMotorAccel(outputs.motorAccel);
  // TODO: Send Motor accel/regen
  // TODO: Swtich LEDs
  // TODO: Switch relays
}

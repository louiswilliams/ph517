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

  _io.setChar(ENGINE_RPM, _inputs.engine.rpm);
  _io.setChar(ENGINE_FUEL, _inputs.engine.fuelRate);
}

// Process inputs and return outputs from control block
void PH517Runner::processInputs(const DataInputs& inputs, DataOutputs& outputs) {
  outputs.engineServo = map(inputs.throttle, THROTTLE_MIN, THROTTLE_MAX, 0, 255);
  outputs.motorAccel = map(inputs.throttle, THROTTLE_MIN, THROTTLE_MAX, 0, 4095);

  // White
  if (inputs.modeSwitches & BUTTON(1)) {
    outputs.crankActive = true;
    outputs.modeLEDs &= (0xFF - BUTTON(1));
  } else {    
    outputs.crankActive = false;
    outputs.modeLEDs |= BUTTON(1);
  }
  // Green
  if (inputs.modeSwitches & BUTTON(2)) {
    outputs.modeLEDs &= (0xFF - BUTTON(2));
  } else {
    // If button was just pressed
    if (outputs.modeLEDs & BUTTON(2) == 0) {
      // If reverse is active, deactivate it
      if (outputs.reverseActive) {
        outputs.reverseActive = false;
      } else {
        outputs.modeLEDs |= BUTTON(2);
        outputs.reverseActive = true;
      }
    }
  }
  // Blue
  if (inputs.modeSwitches & BUTTON(3)) {
    outputs.modeLEDs &= (0xFF - BUTTON(3));
  } else {    
    outputs.modeLEDs |= BUTTON(3);
  }
  // Red
  if (inputs.modeSwitches & BUTTON(4)) {
    outputs.enginePoweroffActive = true;
    outputs.modeLEDs &= (0xFF - BUTTON(4));
  } else {    
    outputs.enginePoweroffActive = false;
    outputs.modeLEDs |= BUTTON(4);
  }



  // Serial.println("--Outputs--");
  // Serial.println(" engineServo: " + outputs.engineServo);
  // Serial.println(" motorAccel: " + outputs.motorAccel);  
}

// Actuate values to hardware components
void PH517Runner::sendOutputs(const DataOutputs& outputs) {
  _io.sendEngineAccel(outputs.engineServo);
  _io.sendMotorAccel(outputs.motorAccel);

  // Button LEDs
  if (_outputs.modeLEDs & BUTTON(1)) {
    digitalWrite(BUTTON_LED_1, HIGH);
  } else {
    digitalWrite(BUTTON_LED_1, LOW);
  }
  if (_outputs.modeLEDs & BUTTON(2)) {
    digitalWrite(BUTTON_LED_2, LOW);
  } else {
    digitalWrite(BUTTON_LED_2, HIGH);
  }
  if (_outputs.modeLEDs & BUTTON(3)) {
    digitalWrite(BUTTON_LED_3, LOW);
  } else {
    digitalWrite(BUTTON_LED_3, HIGH);    
  }
  if (_outputs.modeLEDs & BUTTON(4)) {
    digitalWrite(BUTTON_LED_4, HIGH);
  } else {
    digitalWrite(BUTTON_LED_4, LOW);
  }

  // Engine poweroff
  if (_outputs.enginePoweroffActive) {
    digitalWrite(RELAY_POWER, LOW);
  } else {
    digitalWrite(RELAY_POWER, HIGH);
  }
  // Reverse
  if (outputs.reverseActive) {
    digitalWrite(RELAY_MOTORDIR, LOW);
    digitalWrite(RELAY_REVERSE, LOW);
  } else {
    digitalWrite(RELAY_MOTORDIR, HIGH);
    digitalWrite(RELAY_REVERSE, HIGH);
  }
  // Starter
  if (outputs.crankActive) {
    digitalWrite(RELAY_CRANK, LOW);
  } else {
    digitalWrite(RELAY_CRANK, HIGH);
  }


  // TODO: Send Motor accel/regen
  // TODO: Swtich LEDs
  // TODO: Switch relays
}

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
void PH517Runner::collectInputs(DataInput& inputs) {
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

  _io.readEngineData();
  _io.getBattData(inputs.batt);
  _io.getMotorData(inputs.motor);
}

// Parse incoming 
void PH517Runner::engineDataReceived(const uint8_t* data, uint16_t length) {
  _inputs.setEngineData(data, length);

  _io.setChar(ENGINE_RPM, _inputs.engine.rpm);
  _io.setChar(ENGINE_PULSES, _inputs.engine.pulses);
  _io.setChar(ENGINE_TIMEON, _inputs.engine.timeOn);
}

// Process inputs and return outputs from control block
void PH517Runner::processInputs(const DataInput& inputs, DataOutput& outputs) {
  outputs.engineServo = map(inputs.throttle, THROTTLE_MIN, THROTTLE_MAX, 0, 255);
  outputs.motorAccel = map(inputs.throttle, THROTTLE_MIN, THROTTLE_MAX, 0, 4095);

  // White
  if (inputs.isSwitchPressed(1)) {
    outputs.crankActive = true;
    outputs.setLedOn(1, false);
  } else {    
    outputs.crankActive = false;
    outputs.setLedOn(1, true);
  }
  // Green
  if (inputs.isSwitchPressed(2)) {
    if (outputs.reverseActive) {
      // Disable light if active
      outputs.setLedOn(2, false);
    } else {
      // Enable light if inactive
      outputs.setLedOn(2, true);
    }
  } else {

    // If reverse is active and LED is OFF, deactivate reverse
    if (outputs.reverseActive && !(outputs.isLedOn(2))) {
      outputs.reverseActive = false;
      outputs.setLedOn(2, false);
    // If reverse is not active and LED is ON, activate
    } else if (!outputs.reverseActive && (outputs.isLedOn(2))) {
      outputs.setLedOn(2, true);
      outputs.reverseActive = true;
    }
  }
  // Blue
  if (inputs.isSwitchPressed(3)) {
    outputs.setLedOn(3, false);
  } else {    
    outputs.setLedOn(3, true);
  }
  // Red
  if (inputs.isSwitchPressed(4)) {
    outputs.enginePoweroffActive = true;
    outputs.setLedOn(4, false);
  } else {    
    outputs.enginePoweroffActive = false;
    outputs.setLedOn(4, true);
  }



  // Serial.println("--Outputs--");
  // Serial.println(" engineServo: " + outputs.engineServo);
  // Serial.println(" motorAccel: " + outputs.motorAccel);  
}

// Actuate values to hardware components
void PH517Runner::sendOutputs(const DataOutput& outputs) {
  _io.sendEngineAccel(outputs.engineServo);
  _io.sendMotorAccel(outputs.motorAccel);

  // Button LEDs
  if (outputs.isLedOn(1)) {
    digitalWrite(BUTTON_LED_1, HIGH);
  } else {
    digitalWrite(BUTTON_LED_1, LOW);
  }
  if (outputs.isLedOn(2)) {
    digitalWrite(BUTTON_LED_2, HIGH);
  } else {
    digitalWrite(BUTTON_LED_2, LOW);
  }
  if (outputs.isLedOn(3)) {
    digitalWrite(BUTTON_LED_3, HIGH);
  } else {
    digitalWrite(BUTTON_LED_3, LOW);    
  }
  if (outputs.isLedOn(4)) {
    digitalWrite(BUTTON_LED_4, HIGH);
  } else {
    digitalWrite(BUTTON_LED_4, LOW);
  }

  // Engine poweroff
  if (outputs.enginePoweroffActive) {
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

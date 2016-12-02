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

  // Increment step counter
  _numSteps++;

  // Send data if the send delay timer is overdue
  uint32_t now = millis();
  if (now - _lastDebugPrint > DEBUG_PRINT_DELAY) {

    // Step rate (steps/second)
    float rate = _numSteps/((now - _lastDebugPrint)/1000.0);

    Serial.println();
    Serial.print("Rate (Hz): ");
    Serial.println(rate, 1);
    Serial.println("MOTOR");
    Serial.print("  rpm: ");
    Serial.println(_inputs.motor.rpm, DEC); // x1
    Serial.print("  rmsCurrent: ");
    Serial.println(_inputs.motor.rmsCurrent / 10.0, 1); // x10
    Serial.print("  capVoltage: ");
    Serial.println(_inputs.motor.capVoltage / 10.0, 1); // x10
    Serial.print("  statorFreq: ");
    Serial.println(_inputs.motor.statorFreq, DEC); // x1
    Serial.print("  temp: ");
    Serial.println(_inputs.motor.temp - 40, DEC); // [0,240] - 40
    Serial.print("  controllerTemp: ");
    Serial.println(_inputs.motor.controllerTemp - 40, DEC); // [0,240] - 40

    Serial.println("PEDALS");
    Serial.print("  throttle: ");
    Serial.println(_inputs.throttle, DEC);
    Serial.print("  brake: ");
    Serial.println(_inputs.brake, DEC);

    Serial.println("ENGINE");
    Serial.print("  rpm: ");
    Serial.println(_inputs.engine.rpm, DEC);
    Serial.print("  pulses: ");
    Serial.println(_inputs.engine.pulses, DEC);
    Serial.print("  timeOn: " );
    Serial.println(_inputs.engine.timeOn, DEC); 

    _lastDebugPrint = now;
    _numSteps = 0;
  }  
  delay(10);
  return true;
}

// Collect data from hardware into the inputs structure 
void PH517Runner::collectInputs(DataInput& inputs) {
  inputs.throttle = _io.readThrottle();
  inputs.brake = _io.readBrake();
  inputs.battTemp = _io.readBattTemp();
  inputs.modeSwitches = _io.readModeSwitches();

  _io.readEngineData();
  _io.getBattData(inputs.batt);
  _io.getMotorData(inputs.motor);
}

// Parse incoming 
void PH517Runner::engineDataReceived(const uint8_t* data, uint16_t length) {
  _inputs.setEngineData(data, length);
}

// Process inputs and return outputs from control block
void PH517Runner::processInputs(const DataInput& inputs, DataOutput& outputs) {

  //
  // Push Buttons
  //
  outputs.modeLEDs = 0;

  // White: Reverse
  if (inputs.isSwitchPressed(1) || _carMode == REVERSE_MODE) {
    // Enable light and reverse mode
    outputs.setLedOn(1, true);
    _carMode = REVERSE_MODE;
  }
  // Blue: Econ
  if (inputs.isSwitchPressed(2) || _carMode == ECON_MODE) {

    // If already in Econ mode, activate starter
    if (inputs.isSwitchPressed(2) && _carMode == ECON_MODE) {
      _crankActive = true;      
    } else {
      _crankActive = false;
    }
    outputs.setLedOn(2, true);
    _carMode = ECON_MODE;
    // Delay so starter isn't immediately activated
    delay(250);
  }

  // Green: Electric
  if (inputs.isSwitchPressed(3) || _carMode == ELECTRIC_MODE) {
    if (_carMode != ELECTRIC_MODE) {
      // Set this. The runner will delay for a few seconds and then set to false
      _enginePoweroffActive = true;
    }
    outputs.setLedOn(3, true);
    _carMode = ELECTRIC_MODE;
  }
  // Red: Sport
  if (inputs.isSwitchPressed(4) || _carMode == SPORT_MODE) {
    if (inputs.isSwitchPressed(4) && _carMode == SPORT_MODE) {
      _crankActive = true;
    } else {
      _crankActive = false;
    }
    outputs.setLedOn(4, true);
    _carMode = SPORT_MODE;
    // Delay so starter isn't immediately activated
    delay(250);
  }

  // Calculate engine, motor, and regen values
  uint8_t engineAccel = constrain(map(inputs.throttle, THROTTLE_MIN, THROTTLE_MAX, 0, 255), 0, 255);
  uint16_t motorAccel = constrain(map(inputs.throttle, THROTTLE_MIN, THROTTLE_MAX, 0, 4095), 0, 4095);
  uint16_t throttleRegen = constrain(map(inputs.throttle, THROTTLE_MIN, THROTTLE_MAX, 0, 4095), 0, 4095);
  uint16_t brakeRegen = constrain(map(inputs.brake, BRAKE_MIN, BRAKE_MAX, 0, 4095), 0, 4095);

  // Reverse mode. Electic only //
  if (_carMode == REVERSE_MODE) {
    outputs.motorAccel = motorAccel;
    outputs.engineServo = 0;
    outputs.motorRegen = 0;

  // Econ mode. Engine 100%, motorRegen 50%, brakeRegen %100
  } else if (_carMode == ECON_MODE) {
    outputs.motorAccel = 0;
    outputs.engineServo = engineAccel;
    outputs.motorRegen = max(0.5*throttleRegen, brakeRegen);

  // Only electric
  } else if (_carMode == ELECTRIC_MODE) {
    outputs.motorAccel = motorAccel;
    outputs.engineServo = 0;
    outputs.motorRegen = brakeRegen;

  // Both electric and engine
  } else if (_carMode == SPORT_MODE) {
    outputs.motorAccel = motorAccel;
    outputs.engineServo = engineAccel;
    outputs.motorRegen = brakeRegen;
  }
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
  if (_enginePoweroffActive) {

    digitalWrite(RELAY_POWER, LOW);

    // Hold if switching to electric electric mode
    if (_carMode == ELECTRIC_MODE) {
      delay(2000);
      _enginePoweroffActive = false;
    }
  } else {
    digitalWrite(RELAY_POWER, HIGH);
  }

  // Starter
  if (_crankActive) {
    digitalWrite(RELAY_CRANK, LOW);
  } else {
    digitalWrite(RELAY_CRANK, HIGH);
  }

  // Reverse: Reverse motor direction and activate brake lights
  if (_carMode == REVERSE_MODE) {
    digitalWrite(RELAY_MOTORDIR, LOW);
    digitalWrite(RELAY_REVERSE, LOW);
  } else {
    digitalWrite(RELAY_MOTORDIR, HIGH);
    digitalWrite(RELAY_REVERSE, HIGH);
  }

  // Enable/disable DC/DC converter
  if (_carMode == ECON_MODE) {
    digitalWrite(RELAY_DCDC, HIGH);
  } else if (_carMode == ELECTRIC_MODE) {
    digitalWrite(RELAY_DCDC, LOW);
  } else if (_carMode == SPORT_MODE) {
    digitalWrite(RELAY_DCDC, HIGH);
  }

  // Send data if the send delay timer is overdue
  char sendBuf[32];
  uint32_t now = millis();
  if (now - _lastFrameSend > FRAME_SEND_DELAY) {
    _lastFrameSend = now;
    
    uint16_t cursor = 0;

    // Send first 8 bytes of input struct
    memcpy(sendBuf, &_inputs, 8);
    cursor += 8;

    memcpy(&sendBuf[cursor], &_inputs.batt, sizeof(_inputs.batt));
    cursor += sizeof(_inputs.batt);

    memcpy(&sendBuf[cursor], &_inputs.motor, sizeof(_inputs.motor));
    cursor += sizeof(_inputs.motor);

    hdlc.frameDecode((const char*)  sendBuf, cursor);

  }
}

uint8_t PH517Runner::getModeFromSwitches(uint8_t switches) {

  uint8_t mode = NO_MODE;
  if (switches & 1) {
    mode = REVERSE_MODE;
  } else if ((switches>>1) & 1) {
    mode = ECON_MODE;
  } else if ((switches>>2) & 1) {
    mode = ELECTRIC_MODE;
  } else if ((switches>>3) & 1) {
    mode = SPORT_MODE;
  }

  return mode;
}

/* Blink lights until mode button pressed */
void PH517Runner::waitForMode() {

  uint8_t mode;
  uint8_t iSwitch = 0;
  uint8_t steps = 0;
  do {
    // 5 Cases. 1, 2, 3, 4, then off.
    switch (iSwitch) {
      case 0: digitalWrite(BUTTON_LED_1, HIGH); break;
      case 1: digitalWrite(BUTTON_LED_2, HIGH); break;
      case 2: digitalWrite(BUTTON_LED_3, HIGH); break;
      case 3: digitalWrite(BUTTON_LED_4, HIGH); break;
      default: 
        digitalWrite(BUTTON_LED_1, LOW);
        digitalWrite(BUTTON_LED_2, LOW);
        digitalWrite(BUTTON_LED_3, LOW);
        digitalWrite(BUTTON_LED_4, LOW);
        break;
    }
    // Change every 250ms
    if (steps == 25) {
      iSwitch = (iSwitch + 1) % 5;
      steps = 0;
    }
    mode = getModeFromSwitches(_io.readModeSwitches());

    steps++;
    delay(10);
  } while (mode == NO_MODE);

  _carMode = mode;
}

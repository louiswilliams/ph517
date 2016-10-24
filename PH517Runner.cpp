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

  return true;
}

// Collect data from hardware into the inputs structure 
void PH517Runner::collectInputs(DataInputs& inputs) {
  inputs.throttle = _io.readThrottle();
  inputs.brake = _io.readBrake();
  inputs.battTemp = _io.readBattTemp();
  inputs.modeSwitches = _io.readModeSwitches();

  _io.getBattData(inputs.batt);
  _io.getMotorData(inputs.motor); 
}

// Parse incoming 
void PH517Runner::setEngineData(const uint8_t* data, uint16_t length) {
  _io.getEngineDataFromBuffer(data, length, _inputs.engine);
}

// Process inputs and return outputs from control block
void PH517Runner::processInputs(const DataInputs& inputs, DataOutputs& outputs) {
  outputs.engineServo = map(inputs.throttle, 862, 555, 0, 255);
  outputs.motorAccel = map(inputs.throttle, 862, 555, 0, 4095);
}

// Actuate values to hardware components
void PH517Runner::sendOutputs(const DataOutputs& outputs) {
  // _io.sendEngineAccel(outputs.engineServo);
  _io.sendMotorAccel(outputs.motorAccel);
  // TODO: Send Motor accel/regen
  // TODO: Swtich LEDs
  // TODO: Switch relays
}

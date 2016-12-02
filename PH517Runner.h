/**
  @author Louis Williams
  @date 2016

  PH517 Controls Runner
**/

#ifndef _PH517_Runner_H
#define _PH517_Runner_H

#include "Data.h"
#include "InputOutput.h"

#define FRAME_SEND_DELAY 250
#define DEBUG_PRINT_DELAY 2000

#define CONST_ENGINE_PERCENT 1.0
#define CONST_MOTOR_PERCENT 1.0
#define CONST_REGEN_PERCENT 0.0

typedef enum {
  NO_MODE,
  ECON_MODE,
  ELECTRIC_MODE,
  SPORT_MODE,
  REVERSE_MODE
};

class PH517Runner {
public:
  PH517Runner() {};
  PH517Runner(InputOutput& io);
  // Step forward once. Returns true on successful completion
  bool step();
  // Collect data from hardware into the inputs structure 
  void collectInputs(DataInput& inputs);
  // Process inputs and return outputs from control block
  void processInputs(const DataInput& inputs, DataOutput& outputs);
  // Actuate values to hardware components
  void sendOutputs(const DataOutput& outputs);

  // Block until a mode selection has been made
  void waitForMode();

  // Callback from HDLC link when engine data is received
  void engineDataReceived(const uint8_t* data, uint16_t length);

private:

  // Return mode enum from input switch mask
  uint8_t getModeFromSwitches(uint8_t switches);

  InputOutput _io;  
  DataInput _inputs;
  DataOutput _outputs;

  // Keep track of last frame sent to Uno
  uint32_t _lastFrameSend = 0; 
  uint32_t _lastDebugPrint = 0; 
  uint16_t _numSteps = 0;

  // State
  uint8_t _carMode = NO_MODE;
  bool _reverseActive = false;
  bool _crankActive = false;
  bool _enginePoweroffActive = false;
};

#endif
/**
  @author Louis Williams
  @date 2016

  PH517 Controls Runner
**/

#ifndef _PH517_Runner_H
#define _PH517_Runner_H

#include "InputOutput.h"

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


  // Callback from HDLC link when engine data is received
  void engineDataReceived(const uint8_t* data, uint16_t length);

private:
  InputOutput _io;  
  DataInput _inputs;
  DataOutput _outputs;
};

#endif
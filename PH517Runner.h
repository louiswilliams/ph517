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
  void collectInputs(DataInputs& inputs);
  // Process inputs and return outputs from control block
  void processInputs(const DataInputs& inputs, DataOutputs& outputs);
  // Actuate values to hardware components
  void sendOutputs(const DataOutputs& outputs);
private:
  InputOutput _io;  
  DataInputs _inputs;
  DataOutputs _outputs;
};

#endif
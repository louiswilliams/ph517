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
  PH517Runner(bool debug = false);
  // Run setup
  bool setup();
  // Run, only returns on error
  bool run();
private:
  InputOutput io;
  DataSample dataSample;
};

#endif
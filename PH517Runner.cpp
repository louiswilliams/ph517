/**
  @author Louis Williams
  @date 2016

  PH517 Controls Runner
**/

#include "PH517Runner.h"

PH517Runner::PH517Runner(bool debug) {
  io = InputOutput(debug);
}

bool PH517Runner::setup() {
  return io.setup();
}

bool PH517Runner::run() {
  bool running = true;
  while (running) {
    uint16_t throttle = io.readThrottle();
    io.sendEngineAccel(map(throttle, 0, 1023, 0, 255));
    delay(10);
  }

  return running;
}
  #define __ASSERT_USE_STDERR
#define PH_DEBUG true

#include <assert.h>
#include "InputOutput.h"
#include "PH517Runner.h"

InputOutput io(PH_DEBUG);
PH517Runner runner(io);

// Define HDLC object and handler functions
// We can't put this in the IO service because the frame receiver needs access
// to the global io object. Instead we send the buffer to the runner, which 
// uses the IO service to parse the data
Arduhdlc hdlc = Arduhdlc(&hdlcSendChar, &hdldRecvFrame, HLDC_MAX_FRAME_LEN);

void setup() {
  Serial.begin(115200);
  if (!io.setup()) {
    Serial.println("Failed to setup");
  } else {
    Serial.println("Setup done");
  }
}

void loop() {
  if (!runner.step()) {
    if (PH_DEBUG){
      Serial.println("Step failed");
      // TODO: Add abillity to get error from runner
    }
  }
}

// HDLC data handling
// Send data
void hdlcSendChar(uint8_t data) {
  ENGINE_HWSERIAL.print((char)data);
}

// Capture data available on the engine serial line
void hdldRecvFrame(const uint8_t* data, uint16_t len) {
  runner.setEngineData(data, len);
}


// We need to handle assertions and log problems
void __assert(const char *__func, const char *__file, int __lineno, const char *__sexp) {
    // transmit diagnostic informations through serial link. 
    Serial.println(__func);
    Serial.println(__file);
    Serial.println(__lineno, DEC);
    Serial.println(__sexp);
    Serial.flush();
    // abort program execution.
    abort();  
}

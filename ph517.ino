#define __ASSERT_USE_STDERR
#define PH_DEBUG true

#include "PH517Runner.h"

InputOutput io(PH_DEBUG);
PH517Runner runner(io);

void setup() {
  Serial.begin(115200);
  if (!io.setup()) {
    Serial.println("Failed to setup");
  }
}

void loop() {
  runner.step();
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

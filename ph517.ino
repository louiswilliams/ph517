#define __ASSERT_USE_STDERR
#define PH_DEBUG true

#include "PH517Runner.h"

PH517Runner runner;
void setup() {
  Serial.begin(9600);
  if (!runner.setup()) {
    Serial.println("Failed to setup");
  }
}

void loop() {
  runner.run();
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

#include <Dali.h>

Dali dali;

void setup() {
  dali.begin();
  Serial.println("Cosino DALI application - by HCE Engineering & Luca Zulberti");
}

void loop() {
  serialDali();
}


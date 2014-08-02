#include <Dali.h>

Dali dali0, dali1, dali2;

void setup() {
	dali0.begin(11, 12);	// Overwrite OC1A and OC1B
	dali1.begin(14, 15);	// Overwrite Serial3 (RX3 and TX3)
	dali2.begin(A14, A15);	// Overwrite ADch 14 and 15
	Serial.println("Cosino DALI application - by HCE Engineering & Luca Zulberti");
}

void loop() {
	serialDali();
}


#include <Dali.h>

Dali dali0, dali1;

void setup() {
	dali0.begin(18, 10);		// Overwrite OC1A and OC1B
	//dali1.begin(A14, A15);	// Overwrite ADCch 14 and 15
	Serial.print(" \r");
	Serial.println("Cosino DALI application - by HCE Engineering & Luca Zulberti");
}


void loop() {
	serialDali();
	//delay(1000);
	
	//dali0.remap(ALL);			//Init devs from addr 0x00
	//dali0.sendDirect(254, SINGLE, 0x00);
	//delay(1000);
	//dali0.sendDirect(0, SINGLE, 0x00);
	
	// Prove Dimming Curve
	/*dali0.sendExtCommand(257, 1);		//DTR = 1
	dali0.sendCommand(227, SINGLE, 0x00);	//Dimming Curve = DTR*/
	
	/*dali0.sendCommand(2, SINGLE, 0x00);
	dali0.sendCommand(2, SINGLE, 0x00);
	dali0.sendCommand(2, SINGLE, 0x00);
	dali0.sendCommand(2, SINGLE, 0x00);
	dali0.sendCommand(2, SINGLE, 0x00);
	dali0.sendCommand(2, SINGLE, 0x00);
	dali0.sendCommand(2, SINGLE, 0x00);
	dali0.sendCommand(2, SINGLE, 0x00);
	dali0.sendCommand(2, SINGLE, 0x00);
	dali0.sendCommand(2, SINGLE, 0x00);
	dali0.sendDirect(254, SINGLE, 0x00);*/
	
	//dali0.readStat(SINGLE, 0x00);
	//while(1);
}

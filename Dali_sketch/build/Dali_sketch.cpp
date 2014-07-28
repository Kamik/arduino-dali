#include <Arduino.h>

int main(void)
{
	init();

#if defined(USBCON)
	USBDevice.attach();
#endif
	
	setup();
    
	for (;;) {
		loop();
		if (serialEventRun) serialEventRun();
	}
        
	return 0;
}

#line 1 "build/Dali_sketch.pde"
#include <Dali.h>

Dali dali;

void setup() {
  dali.begin();
}

void loop() {
  serialDali();
}


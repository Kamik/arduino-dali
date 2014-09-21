#include "Dali.h"
#include <avr/eeprom.h>

void Dali_rx(Dali *d, uint8_t *data, uint8_t len);
void serialDali_rx(uint8_t errn, uint8_t *data, uint8_t n);
uint8_t exeCmd(char *msg);
uint8_t devCmd(char *msg);
uint8_t grpCmd(char *msg);
uint8_t busCmd(char *msg);
uint8_t rmpCmd(char *msg);
uint8_t action(char *msg);
void storeSlaves(Dali *dali);

Dali * Master[2];
uint8_t bytes_rx;
char msg[10];
uint8_t rx_buf[10];	//first byte = lenght

void serialDali(void)
{
	uint8_t errn;
	
	if (Serial.available()) {
		msg[bytes_rx] = (char)Serial.read();
		if (msg[bytes_rx] == '\n') {
			if (msg[bytes_rx-1] == '\r')	// Adjustment protocol
				msg[bytes_rx-1] = '\n';
			bytes_rx = 0;
			errn = exeCmd(msg);
			if (errn) serialDali_rx(errn, NULL, 0);
			else serialDali_rx(0, rx_buf + 1, rx_buf[0]);
		} else if (bytes_rx == 9) bytes_rx = 0;
		else bytes_rx++;
	}
}

void serialDali_rx(uint8_t errn, uint8_t *data, uint8_t n)
{
	uint8_t buf[10];
	
	if (errn == 0){
		buf[0] = 'O';
		buf[1] = n;
		for (int a = 0; a < n; a++)
			buf[a+2] = data[a];
		buf[n+2] = '\n';
		Serial.write(buf, n+3);
	} else {	
		Serial.print("E");
		switch(errn) {
			case 0x01:
				Serial.write(0x01);
				Serial.write(0x01);
				Serial.println("E1");
				break;
			case 0x02:
				Serial.write(0x01);
				Serial.write(0x02);
				Serial.println("E2");
				break;
			case 0x03:
				Serial.write(0x01);
				Serial.write(0x03);
				Serial.println("E3");
				break;
			case 0x20:
				Serial.write(0x01);
				Serial.write(0x20);
				Serial.println("E20");
				break;
			case 0x90:
				Serial.write(0x01);
				Serial.write(0x90);
				Serial.println("E90");
				break;
		}
	}	
}

void Dali_rx(Dali *d, uint8_t *data, uint8_t len)
{
	serialDali_rx(0, data, 1);
}

uint8_t exeCmd(char *msg)
{
	switch(msg[0]) {
		case 'd':
			return devCmd(msg);
			break;
		case 'g':
			return grpCmd(msg);
			break;
		case 'b':
			return busCmd(msg);
			break;
		case 'R':
			return rmpCmd(msg);
			break;
	}
	return 0x01;			// Syntax error
}

uint8_t devCmd(char *msg)
{
	uint8_t bus, dev, arc;
	char str[] = "00";

	if (msg[2] == '0') bus = 0;
	else if (msg[2] == '1') bus = 1;
	else return 0x20;
	if ((Master[bus]->dali_status & 0x01) == 1)
		return 0x03;

	str[0] = msg[3];
	str[1] = msg[4];
	dev = (uint8_t)strtol(str, NULL, 16);
	
	switch (msg[1]) {
		case '1':
			Master[bus]->sendCommand(5, SINGLE, dev);	// ON
			rx_buf[0] = 0;
			return 0x00;
		case '0':
			Master[bus]->sendCommand(0, SINGLE, dev);	// OFF
			rx_buf[0] = 0;
			return 0x00;
		case 'a':
			arc = (uint8_t)strtol(msg+5, NULL, 16);
			Master[bus]->sendDirect(arc, SINGLE, dev);	// arc level
			rx_buf[0] = 0;
			return 0x00;
		case 'i':
			// Chiedere le info del divice
			return 0x02;
		case 'c':
			// Configurare il device a seconda del tipo
			Master[bus]->sendExtCommand(257, 1);		//DTR = 1
			Master[bus]->sendCommand(227, SINGLE, dev);	//Dimming Curve = DTR
			rx_buf[0] = 0;
			return 0x00;
	}

	return 0x01;
}

uint8_t grpCmd(char *msg)
{
	uint8_t bus, grp, arc;
	char str[] = "0";

	if (msg[2] == '0') bus = 0;
	else if (msg[2] == '1') bus = 1;
	else return 0x20;
	if ((Master[bus]->dali_status & 0x01) == 1)
		return 0x03;

	str[0] = msg[3];
	grp = (uint8_t)strtol(str, NULL, 16);
	
	switch (msg[1]) {
		case '1':
			Master[bus]->sendCommand(5, GROUP, grp);	// ON
			rx_buf[0] = 0;
			return 0x00;
		case '0':
			Master[bus]->sendCommand(0, GROUP, grp);	// OFF
			rx_buf[0] = 0;
			return 0x00;
		case 'a':
			arc = (uint8_t)strtol(msg+4, NULL, 16);
			Master[bus]->sendDirect(arc, GROUP, grp);	// arc level
			rx_buf[0] = 0;
			return 0x00;
	}

	return 0x01;
}

uint8_t busCmd(char *msg)
{
	uint8_t bus, grp, arc;
	char str[] = "00";

	if (msg[2] == '0') bus = 0;
	else if (msg[2] == '1') bus = 1;
	else return 0x20;
	if ((Master[bus]->dali_status & 0x01) == 1)
		return 0x03;
	
	switch (msg[1]) {
		case '1':
			Master[bus]->sendCommand(5, BROADCAST, grp);	// ON
			rx_buf[0] = 0;
			return 0x00;
		case '0':
			Master[bus]->sendCommand(0, BROADCAST, grp);	// OFF
			rx_buf[0] = 0;
			return 0x00;
		case 'a':
			arc = (uint8_t)strtol(msg+3, NULL, 16);
			Master[bus]->sendDirect(arc, BROADCAST, grp);	// arc level
			rx_buf[0] = 0;
			return 0x00;
		case 'd':
			// Detect
			return 0x02;
	}

	return 0x01;
}

uint8_t rmpCmd(char *msg)
{
	switch (msg[1]) {
		case '\n':	// Remap All
			if ((Master[0]->dali_status & 0x01) == 1 || (Master[1]->dali_status & 0x01) == 1)
				return 0x03;
			for (int i = 0; i < 2; i++)
				if (Master[i] != NULL)
					Master[i]->remap(ALL);
			rx_buf[0] = 1;
			rx_buf[1] = dev_found;
			return 0x00;
		case 'u':	// Remap unknown Dev
			if ((Master[0]->dali_status & 0x01) == 1 || (Master[1]->dali_status & 0x01) == 1)
				return 0x03;
			for (int i = 0; i < 2; i++)
				if (Master[i] != NULL)
					Master[i]->remap(MISS_SHORT);
			rx_buf[0] = 1;
			rx_buf[1] = dev_found;
			return 0x00;
		case 'f':	// Remap finished?
			if ((Master[0]->dali_status & 0x01) == 1 || (Master[1]->dali_status & 0x01) == 1)
				rx_buf[1] = dev_found;
			else
				rx_buf[1] = 0xFF;
			rx_buf[0] = 1;
			return 0x00;
		case 'A':	// Remap Abort
			for (int i = 0; i < 2; i++)
				if (Master[i] != NULL)
					Master[i]->abort_remap();
			rx_buf[0] = 0;
			return 0x00;
	}
	return 0x01;
}

void storeSlaves(Dali *dali, uint8_t *slaves)
{
	int base_addr;

	if (dali == Master[0])
		base_addr = 0x0000;
	else if (dali == Master[1])
		base_addr = 0x0100;
	else return;

	for (int i = 0; i < 8; i++)
		eeprom_write_byte((unsigned char *) (base_addr + 1), slaves[i]);

	// return eeprom_read_byte((unsigned char *) address);
}



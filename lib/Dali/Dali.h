#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include "Arduino.h"

#define DALI_HOOK_COUNT 2
enum addr_type {BROADCAST = 0, GROUP, SINGLE};
enum readdr_type {ALL = 0, MISS_SHORT = 255};

class Dali {
public:
	typedef void (*EventHandlerReceivedDataFuncPtr)(Dali *sender, uint8_t *data, uint8_t len);
	EventHandlerReceivedDataFuncPtr EventHandlerReceivedData;

	void begin(uint8_t tx_pin, uint8_t rx_pin);
	uint8_t send(uint8_t* tx_msg, uint8_t tx_len_bytes);
	uint8_t sendwait(uint8_t* tx_msg, uint8_t tx_len_bytes, uint32_t timeout_ms=500);
	uint8_t sendwait_int(uint16_t tx_msg, uint32_t timeout_ms=500);
	uint8_t sendwait_byte(uint8_t tx_msg, uint32_t timeout_ms=500);
	void ISR_timer();
	void ISR_pinchange();

	uint8_t sendDirect(uint8_t val, addr_type type_addr, uint8_t addr);
	uint8_t sendCommand(uint8_t val, addr_type type_addr, uint8_t addr);
	uint8_t sendExtCommand(uint16_t com, uint8_t val);

	void readStat(addr_type type_addr, uint8_t addr);

	//DALIDA functions
	void remap(readdr_type remap_type);
	void abort_remap(void);
	void list_dev(void);

	//DALIDA variables
	uint8_t dali_status;	//b0 = remapping, b1 = need remap,
	uint8_t dali_cmd;	//b0 = stop remap,
	uint8_t slaves[8];

private:
	uint8_t bus_number;

	enum tx_stateEnum { IDLE=0,START,START_X,BIT,BIT_X,STOP1,STOP1_X,STOP2,STOP2_X,STOP3};
	uint8_t tx_pin;				//Pin di trasmissione (Arduino)
	uint8_t tx_msg[3];			//Sequenza da trasmettere
	uint8_t tx_len;				//Numero di bytes da trasmettere
	volatile uint8_t tx_pos;		//Posizione dell'attuale bit da trasmettere
	volatile tx_stateEnum tx_state;		//Stato di trasmissione corrente
	volatile uint8_t tx_bus_low;		//Bus a livello logico 0?
	volatile uint8_t tx_collision;  	//Collisione avvenuta?
	
	enum rx_stateEnum { RX_IDLE,RX_START,RX_BIT};
	uint8_t rx_pin;				//Pin di ricezione (Arduino)
	volatile uint8_t rx_last_bus_low;	//Ultimo livello logico bus di ricezione = 0?
	volatile uint32_t rx_last_change_ts;	//Timestamp ultimo pinchange
	volatile rx_stateEnum rx_state;		//Stato di ricezione corrente
	volatile uint8_t rx_msg[3];		//Sequenza ricevuta
	volatile int8_t rx_len;			//Numero di bytes ricevuti
	volatile uint8_t rx_last_halfbit;	//Ultimo mezzo-bit ricevuto
	volatile uint8_t rx_int_rq;		//Richiesta di lancio routine interrupt personale
	volatile uint8_t bus_idle_te_cnt;	//Numero di tempi te(417 us) con il bus IDLE
	
	void push_halfbit(uint8_t bit);
	
	//DALIDA
	uint8_t setDevAddress(uint8_t start_addr, readdr_type all);
	uint32_t findDev(uint32_t base_addr, uint32_t delta_addr, uint8_t n);
	void setSearch(uint32_t addr);
};

void serialDali(void);
extern Dali * Master[2];
extern uint8_t bytes_rx;
extern void storeSlaves(Dali *dali, uint8_t *slaves);
extern uint8_t dev_found;

/*  
SIGNAL CHARACTERISTICS
High Level: 9.5 to 22.5 V (Typical 16 V)
Low Level: -6.5 to + 6.5 V (Typical 0 V)
Te = half cycle = 416.67 �s +/- 10 %
10 �s <= tfall <= 100 �s 
10 �s <= trise <= 100 �s

BIT TIMING
msb send first
 logical 1 = 1Te Low 1Te High
 logical 0 = 1Te High 1Te Low
 Start bit = logical 1
 Stop bit = 2Te High

FRAME TIMING
FF: TX Forward Frame 2 bytes (38Te) = 2*(1start+16bits+2stop)
BF: RX Backward Frame 1 byte (22Te) = 2*(1start+8bits+2stop)
no reply: FF >22Te pause FF
with reply: FF >7Te <22Te pause BF >22Te pause FF


DALI commands
=============
In accordance with the DIN EN 60929 standard, addresses and commands are transmitted as numbers with a length of two bytes.

These commands take the form YAAA AAAS xxXXxx. Each letter here stands for one bit.

Y: type of address
     0bin:    short address
     1bin:    group address or collective call

A: significant address bit

S: selection bit (specifies the significance of the following eight bits):
     0bin:    the 8 xxXXxx bits contain a value for direct control of the lamp power
     1bin:    the 8 xxXXxx bits contain a command number.

x: a bit in the lamp power or in the command number


Type of Addresses
=================
Type of Addresses Byte Description
Short address 0AAAAAAS (AAAAAA = 0 to 63, S = 0/1)
Group address 100AAAAS (AAAA = 0 to 15, S = 0/1)
Broadcast address 1111111S (S = 0/1)
Special command 101CCCC1 (CCCC = command number)


Direct DALI commands for lamp power
===================================
These commands take the form YAAA AAA0 xxXXxx.

xxXXxx: the value representing the lamp power is transmitted in these 8 bits. It is calculated according to this formula: 

Pvalue = 10 ^ ((value-1) / (253/3)) * Pmax / 1000

253 values from 1dec to 254dec are available for transmission in accordance with this formula.

There are also 2 direct DALI commands with special meanings:

Command; Command No; Description; Answer
00hex; 0dec; The DALI device dims using the current fade time down to the parameterised MIN value, and then switches off.; - 
FFhex; 254dec; Mask (no change): this value is ignored in what follows, and is therefore not loaded into memory.; - 


Indirect DALI commands for lamp power
=====================================
These commands take the form YAAA AAA1 xxXXxx.

xxXXxx: These 8 bits transfer the command number. The available command numbers are listed and explained in the following tables in hexadecimal and decimal formats.

0 YAAA AAA1 0000 0000 OFF
1 YAAA AAA1 0000 0001 UP
2 YAAA AAA1 0000 0010 DOWN
3 YAAA AAA1 0000 0011 STEP UP
4 YAAA AAA1 0000 0100 STEP DOWN
5 YAAA AAA1 0000 0101 RECALL MAX LEVEL
6 YAAA AAA1 0000 0110 RECALL MIN LEVEL
7 YAAA AAA1 0000 0111 STEP DOWN AND OFF
8 YAAA AAA1 0000 1000 ON AND STEP UP
9 YAAA AAA1 0000 1001 ENABLE DAPC SEQUENCE

16 – 31 YAAA AAA1 0001 XXXX GO TO SCENE
32 YAAA AAA1 0010 0000 RESET
33 YAAA AAA1 0010 0001 STORE ACTUAL LEVEL IN THE DTR

42 YAAA AAA1 0010 1010 STORE THE DTR AS MAX LEVEL
43 YAAA AAA1 0010 1011 STORE THE DTR AS MIN LEVEL
44 YAAA AAA1 0010 1100 STORE THE DTR AS SYSTEM FAILURE LEVEL
45 YAAA AAA1 0010 1101 STORE THE DTR AS POWER ON LEVEL
46 YAAA AAA1 0010 1110 STORE THE DTR AS FADE TIME
47 YAAA AAA1 0010 1111 STORE THE DTR AS FADE RATE

64 – 79 YAAA AAA1 0100 XXXX STORE THE DTR AS SCENE
80 – 95 YAAA AAA1 0101 XXXX REMOVE FROM SCENE
96 – 111 YAAA AAA1 0110 XXXX ADD TO GROUP
112 – 127 YAAA AAA1 0111 XXXX REMOVE FROM GROUP
128 YAAA AAA1 1000 0000 STORE DTR AS SHORT ADDRESS
129 YAAA AAA1 1000 0001 ENABLE WRITE MEMORY

144 YAAA AAA1 1001 0000 QUERY STATUS
145 YAAA AAA1 1001 0001 QUERY CONTROL GEAR
146 YAAA AAA1 1001 0010 QUERY LAMP FAILURE
147 YAAA AAA1 1001 0011 QUERY LAMP POWER ONLicensed by SOURCE to METROLIGHT

148 YAAA AAA1 1001 0100 QUERY LIMIT ERROR
149 YAAA AAA1 1001 0101 QUERY RESET STATE
150 YAAA AAA1 1001 0110 QUERY MISSING SHORT ADDRESS
151 YAAA AAA1 1001 0111 QUERY VERSION NUMBER
152 YAAA AAA1 1001 1000 QUERY CONTENT DTR
153 YAAA AAA1 1001 1001 QUERY DEVICE TYPE
154 YAAA AAA1 1001 1010 QUERY PHYSICAL MINIMUM LEVEL
155 YAAA AAA1 1001 1011 QUERY POWER FAILURE
156 YAAA AAA1 1001 1100 QUERY CONTENT DTR1
157 YAAA AAA1 1001 1101 QUERY CONTENT DTR2

160 YAAA AAA1 1010 0000 QUERY ACTUAL LEVEL
161 YAAA AAA1 1010 0001 QUERY MAX LEVEL
162 YAAA AAA1 1010 0010 QUERY MIN LEVEL
163 YAAA AAA1 1010 0011 QUERY POWER ON LEVEL
164 YAAA AAA1 1010 0100 QUERY SYSTEM FAILURE LEVEL
165 YAAA AAA1 1010 0101 QUERY FADE TIME/FADE RATE

176 – 191 YAAA AAA1 1011 XXXX QUERY SCENE LEVEL (SCENES 0-15)
192 YAAA AAA1 1100 0000 QUERY GROUPS 0-7
193 YAAA AAA1 1100 0001 QUERY GROUPS 8-15
194 YAAA AAA1 1100 0010 QUERY RANDOM ADDRESS (H)
195 YAAA AAA1 1100 0011 QUERY RANDOM ADDRESS (M)
196 YAAA AAA1 1100 0100 QUERY RANDOM ADDRESS (L)
197 YAAA AAA1 1100 0101 READ MEMORY LOCATION

224 – 254 YAAA AAA1 111X XXXX See parts 2XX of this standard

255 YAAA AAA1 1111 1111 QUERY EXTENDED VERSION NUMBER
256 1010 0001 0000 0000 TERMINATE
257 1010 0011 XXXX XXXX DATA TRANSFER REGISTER (DTR)
258 1010 0101 XXXX XXXX INITIALISE
259 1010 0111 0000 0000 RANDOMISE
260 1010 1001 0000 0000 COMPARE
261 1010 1011 0000 0000 WITHDRAW
262 – 263 1010 11X1 0000 0000 a
264 1011 0001 HHHH HHHH SEARCHADDRH
265 1011 0011 MMMM MMMM SEARCHADDRM
266 1011 0101 LLLL LLLL SEARCHADDRL

267 1011 0111 0AAA AAA1 PROGRAM SHORT ADDRESS
268 1011 1001 0AAA AAA1 VERIFY SHORT ADDRESS
269 1011 1011 0000 0000 QUERY SHORT ADDRESS
270 1011 1101 0000 0000 PHYSICAL SELECTION

272 1100 0001 XXXX XXXX ENABLE DEVICE TYPE X
273 1100 0011 XXXX XXXX DATA TRANSFER REGISTER 1 (DTR1)
274 1100 0101 XXXX XXXX DATA TRANSFER REGISTER 2 (DTR2)
275 1100 0111 XXXX XXXX WRITE MEMORY LOCATION

Però questi comandi estesi non hanno l'indirizzamento, quindi ogni slave va programmato prima di metterlo sul bus con gli altri.


La lista dei comandi specifici (224-254) per i device di tipo 6 (standard 207) è questa:
224 YAAA AAA1 1110 0000 REFERENCE SYSTEM POWER
225 YAAA AAA1 1110 0001 ENABLE CURRENT PROTECTOR
226 YAAA AAA1 1110 0010 DISABLE CURRENT PROTECTOR
227 YAAA AAA1 1110 0011 SELECT DIMMING CURVE
228 YAAA AAA1 1110 0100 STORE DTR AS FAST FADE TIME

237 YAAA AAA1 1110 1101 QUERY GEAR TYPE
238 YAAA AAA1 1110 1110 QUERY DIMMING CURVE
239 YAAA AAA1 1110 1111 QUERY POSSIBLE OPERATING MODES
240 YAAA AAA1 1111 0000 QUERY FEATURES
241 YAAA AAA1 1111 0001 QUERY FAILURE STATUS
242 YAAA AAA1 1111 0010 QUERY SHORT CIRCUIT
243 YAAA AAA1 1111 0011 QUERY OPEN CIRCUIT
244 YAAA AAA1 1111 0100 QUERY LOAD DECREASE
245 YAAA AAA1 1111 0101 QUERY LOAD INCREASE
246 YAAA AAA1 1111 0110 QUERY CURRENT PROTECTOR ACTIVE
247 YAAA AAA1 1111 0111 QUERY THERMAL SHUT DOWN
248 YAAA AAA1 1111 1000 QUERY THERMAL OVERLOAD
249 YAAA AAA1 1111 1001 QUERY REFERENCE RUNNING
250 YAAA AAA1 1111 1010 QUERY REFERENCE MEASUREMENT FAILED
251 YAAA AAA1 1111 1011 QUERY CURRENT PROTECTOR ENABLED
252 YAAA AAA1 1111 1100 QUERY OPERATING MODE
253 YAAA AAA1 1111 1101 QUERY FAST FADE TIME
254 YAAA AAA1 1111 1110 QUERY MIN FAST FADE TIME
255 YAAA AAA1 1111 1111 QUERY EXTENDED VERSION NUMBER
272 1100 0001 0000 0110 ENABLE DEVICE TYPE 6


Note Repeat of DALI commands 
============================
According to IEC 60929, a DALI Master has to repeat several commands within 100 ms, so that DALI-Slaves will execute them. 

The DALI Master Terminal KL6811 repeats the commands 32dez to 128dez, 258dez and 259dez (bold marked) automatically to make the the double call from the user program unnecessary.

The DALI Master Terminal KL6811 repeats also the commands 224dez to 255dez, if you have activated this with Bit 1 of the Control-Byte (CB.1) before.


DALI Control Device Type List
=============================
Type DEC Type HEX Name Comments
128 0x80 Unknown Device. If one of the devices below don�t apply
129 0x81 Switch Device A Wall-Switch based Controller including, but not limited to ON/OFF devices, Scene switches, dimming device.
130 0x82 Slide Dimmer An analog/positional dimming controller 
131 0x83 Motion/Occupancy Sensor. A device that indicates the presence of people within a control area.
132 0x84 Open-loop daylight Controller. A device that outputs current light level and/or sends control messages to actuators based on light passing a threshold.
133 0x85 Closed-loop daylight controller. A device that outputs current light level and/or sends control messages to actuators based on a change in light level.
134 0x86 Scheduler. A device that establishes the building mode based on time of day, or which provides control outputs.
135 0x87 Gateway. An interface to other control systems or communication busses
136 0x88 Sequencer. A device which sequences lights based on a triggering event
137 0x89 Power Supply *). A DALI Power Supply device which supplies power for the communication loop
138 0x8a Emergency Lighting Controller. A device, which is certified for use in control of emergency lighting, or, if not certified, for noncritical backup lighting.
139 0x8b Analog input unit. A general device with analog input.
140 0x8c Data Logger. A unit logging data (can be digital or analog data)


Flash Variables and Offset in Information
=========================================
Memory Name Offset
Power On Level [0]
System Failure Level [1]
Minimum Level [2]
Maximum Level [3]
Fade Rate [4]
Fade Time [5]
Short Address [6]
Group 0 through 7 [7]
Group 8 through 15 [8]
Scene 0 through 15 [9-24]
Random Address [25-27]
Fast Fade Time [28]
Failure Status [29]
Operating Mode [30]
Dimming Curve [31]
*/

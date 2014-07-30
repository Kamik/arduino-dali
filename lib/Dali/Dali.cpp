#include "Dali.h"
#include "Arduino.h"
//###########################################################################
// Define utili
//###########################################################################
#define DALI_BUS_LOW() digitalWrite(this->tx_pin,LOW); this->tx_bus_low=1
#define DALI_BUS_HIGH() digitalWrite(this->tx_pin,HIGH); this->tx_bus_low=0
#define DALI_IS_BUS_LOW() (digitalRead(this->rx_pin)==LOW)
#define DALI_BAUD 1200
#define DALI_TE ((1000000+(DALI_BAUD))/(2*(DALI_BAUD)))  //417us
#define DALI_TE_MIN (80*DALI_TE)/100  
#define DALI_TE_MAX (120*DALI_TE)/100  
#define DALI_IS_TE(x) ((DALI_TE_MIN)<=(x) && (x)<=(DALI_TE_MAX))
#define DALI_IS_2TE(x) ((2*(DALI_TE_MIN))<=(x) && (x)<=(2*(DALI_TE_MAX)))

//###########################################################################
// ISR Trasmissione
//###########################################################################
static Dali *IsrTimerHooks[DALI_HOOK_COUNT+1];
void serialDali_rx(Dali *d, uint8_t *data, uint8_t len);
void exeCmd(uint8_t *msg);
Masters Master;
uint8_t bytes_rx;

ISR(TIMER1_COMPA_vect) {         
	for(uint8_t i=0;i<DALI_HOOK_COUNT;i++) {
		if(IsrTimerHooks[i]==NULL) {return;}
		IsrTimerHooks[i]->ISR_timer();
	}
}

void Dali::ISR_timer() {  
	if(this->bus_idle_te_cnt<0xff) this->bus_idle_te_cnt++;
	 
	//Tabella di invio: start bit, messaggio, 2 stop bits
	switch(this->tx_state) {
		case IDLE: 
			break;
		case START: 
			//Aspetto 22*te come da protocollo
			if(this->bus_idle_te_cnt >= 22) {
				DALI_BUS_LOW();
				this->tx_state=START_X;
			}
		break;
		case START_X:
			DALI_BUS_HIGH();
			this->tx_pos=0;
			this->tx_state=BIT;
		break;
		case BIT: 
			if(this->tx_msg[this->tx_pos >> 3] & 1 << (7 - (this->tx_pos & 0x7))) {
				DALI_BUS_LOW();
			} else {
				DALI_BUS_HIGH();
			}
			this->tx_state = BIT_X;
		break;
		case BIT_X: 
			if(this->tx_msg[this->tx_pos >> 3] & 1 << (7 - (this->tx_pos & 0x7))) {
				DALI_BUS_HIGH();
				} else {
				DALI_BUS_LOW();
			}
			this->tx_state = BIT_X;
			this->tx_pos++;
			if(this->tx_pos < this->tx_len) {
				this->tx_state = BIT;
			} else {
				this->tx_state = STOP1;
			}
		break;  	
		case STOP1: 
			DALI_BUS_HIGH();
			this->tx_state = STOP1_X;
		break;  
		case STOP1_X: 
			this->tx_state = STOP2;
		break;  
		case STOP2: 
			this->tx_state = STOP2_X;
		break;  
		case STOP2_X: 
			this->tx_state = STOP3;
		break;  
		case STOP3: 
			this->bus_idle_te_cnt = 0; 
			this->tx_state = IDLE;
			this->rx_state = RX_IDLE;
		break;  	
	}
  
	//Riconoscimento stop bits in ricezione (4 volte 1 logico)
	if(this->rx_state == RX_BIT && this->bus_idle_te_cnt > 4) {
		this->rx_state = RX_IDLE;
		//2 stop bits ricevuti, fornisco il messaggio ricevuto all'utente
		uint8_t bitlen = (this->rx_len+1) >> 1;
		this->rx_data = this->rx_msg;
		if((bitlen & 0x7) == 0) {
			uint8_t len = bitlen >> 3;
			if(this->EventHandlerReceivedData != NULL) this->EventHandlerReceivedData(this,(uint8_t*)this->rx_msg, len);
		}
	}
}

//###########################################################################
// ISR RICEZIONE
//###########################################################################
//pin PCINT SBAGLIATO
//0-7 PCINT2_vect PCINT16-23
//8-13 PCINT0_vect PCINT0-5
//14-19 PCINT1_vect PCINT8-13
static Dali *IsrPCINT0Hook;
static Dali *IsrPCINT1Hook;
static Dali *IsrPCINT2Hook;

ISR(PCINT0_vect) {
	if(IsrPCINT0Hook != NULL) IsrPCINT0Hook->ISR_pinchange();
} 
ISR(PCINT1_vect) {
	if(IsrPCINT1Hook != NULL) IsrPCINT1Hook->ISR_pinchange();
} 
ISR(PCINT2_vect) {
	if(IsrPCINT2Hook != NULL) IsrPCINT2Hook->ISR_pinchange();
} 


void Dali::ISR_pinchange() {
	uint32_t ts = micros();				//Timestamp pinchange
	this->bus_idle_te_cnt = 0;			//Resetto il contatore dei cicli IDLE
	uint8_t bus_low = DALI_IS_BUS_LOW();//Bus = 0?


	//Se questa istanza sta trasmettendo
	if(this->tx_state != IDLE) {
		//C'è stata una collisisone?
		if(bus_low && !this->tx_bus_low) {
			this->tx_state = IDLE;		//Interrompo la trasmissione
			this->tx_collision = 1;		//Segno la collisisone
		}
		return;
	}

	//Livello logico del bus inalterato
	if(bus_low == this->rx_last_bus_low) return;

	//Livello logico cambiato -> salvo i seguenti valori
	uint32_t dt = ts - this->rx_last_change_ts;		//delta Timestamp per il periodo del livello logico mantenuto
	this->rx_last_change_ts = ts;					//salvo nuovo ultimo Timestamp
	this->rx_last_bus_low = bus_low;				//salvo il nuovo stato del bus

	switch(this->rx_state) {
		case RX_IDLE: 
			if(bus_low) {
				this->rx_state = RX_START;
			}
		break;	  
		case RX_START: 
			if(bus_low || !DALI_IS_TE(dt)) {
				this->rx_state = RX_IDLE;
			}else{
				this->rx_len = -1;
				for(uint8_t i = 0; i < 7; i++)
					this->rx_msg[0] = 0;		  
				this->rx_state = RX_BIT;
			}
		break;
		case RX_BIT:
			if(DALI_IS_TE(dt)) {
				//Inserisco mezzo-bit
				this->push_halfbit(bus_low);
			} else if(DALI_IS_2TE(dt)) {
				//inserisco 2 mezzi-bit
				this->push_halfbit(bus_low);
				this->push_halfbit(bus_low);
			} else {
				//qualcosa non va...
				this->rx_state = RX_IDLE;
				//TODO rx error
				return;
			}		
		break;
  }
}

void Dali::push_halfbit(uint8_t bit) {	
	bit = (~bit) & 1;
	if((this->rx_len & 1) == 0) {
		uint8_t i = this->rx_len >> 4;
		if(i < 3) {
		this->rx_msg[i] = (this->rx_msg[i] << 1) | bit;
		}
	}
	this->rx_len++;
}


//###########################################################################
// Dali Class
//###########################################################################
void Dali::begin(void) {
	this->tx_pin=12;
	this->rx_pin=11;
	this->tx_state = IDLE;
	this->rx_state = RX_IDLE;
    
	Serial.begin(115200);
	
	//setup tx
	if(this->tx_pin>=0) {
		//setup tx pin
		pinMode(this->tx_pin, OUTPUT);  
		DALI_BUS_HIGH();	
    
		//setup tx timer interrupt	
		TCCR1A = 0;
		TCCR1B = 0;
		TCNT1  = 0;

		OCR1A = (F_CPU + DALI_BAUD) / (2 * DALI_BAUD);            // compare match register 16MHz/256/2Hz
		TCCR1B |= (1 << WGM12);   // CTC mode
		TCCR1B |= (1 << CS10);    // 1:1 prescaler 
		TIMSK1 |= (1 << OCIE1A);  // Interrupt abilitato
    
		//setup timer interrupt hooks
		for(uint8_t i = 0; i < DALI_HOOK_COUNT; i++) {
			if(IsrTimerHooks[i] == NULL) {
				IsrTimerHooks[i] = this;
				break;
			}
		}
  	}
   
	//setup rx  
	if(this->rx_pin >= 0) {
	  //setup rx pin
		pinMode(this->rx_pin, INPUT);    

		//setup rx pinchange interrupt
		// 0- 7 PCINT2_vect PCINT16-23
		// 8-13 PCINT0_vect PCINT0-5
		//14-19 PCINT1_vect PCINT8-13
		/*  SBAGLIATO ASSOLUTAMENTE!!!!
		if(this->rx_pin<=7){
			PCICR |= (1<<PCIE2);
			PCMSK2 |= (1<< (this->rx_pin));
			IsrPCINT2Hook = this; //setup pinchange interrupt hook
		}else if(this->rx_pin<=13) {
			PCICR |= (1<<PCIE0);
			PCMSK0 |= (1<< (this->rx_pin-8));
			IsrPCINT0Hook = this; //setup pinchange interrupt hook
		}else if(this->rx_pin<=19) {
			PCICR |= (1<<PCIE1);
			PCMSK1 |= (1<< (this->rx_pin-14));
			IsrPCINT1Hook = this; //setup pinchange interrupt hook
		}*/
		PCICR |= (1 << PCIE0);
		PCMSK0 |= (1 << 4);
		IsrPCINT0Hook = this;
	}
	
	uint8_t i;
	for(i = 0; i < Master.n_bus; i++)
		if (Master.bus[i] != NULL) break;
	
	Master.bus[i] = (Dali *) malloc(sizeof(this));
	Master.n_bus++;
	this->bus_number = i;
	bytes_rx = 0;
}


uint8_t Dali::send(uint8_t* tx_msg, uint8_t tx_len_bytes) {
	if(tx_len_bytes > 3)
		return 2;
	if(this->tx_state != IDLE)
		return 1; 
	for(uint8_t i = 0; i < tx_len_bytes; i++)
		this->tx_msg[i] = tx_msg[i];
	this->tx_len = tx_len_bytes << 3;
	this->tx_collision = 0;
	this->tx_state = START;
	this->rx_int_rq = 1;
	return 0;
}

uint8_t Dali::sendwait(uint8_t* tx_msg, uint8_t tx_len_bytes, uint32_t timeout_ms) {
	if(tx_len_bytes > 3)
		return 5;
	uint32_t ts = millis();
	//wait for idle
	while(this->tx_state != IDLE)
		if(millis() - ts > timeout_ms)
			return 4;
	//start transmit
	if(this->send(tx_msg,tx_len_bytes))
		return 5;
	this->rx_int_rq = 0;
	//Aspetto che la trasmissione sia completa
	while(this->tx_state != IDLE)
		if(millis() - ts > timeout_ms)		//Timeout??
			return 6;
	while(this->rx_int_rq == 0);
	return this->rx_int_rq;
}

uint8_t Dali::sendwait_int(uint16_t tx_msg, uint32_t timeout_ms) {
	uint8_t m[3];
	m[0] = tx_msg >> 8;
	m[1] = tx_msg & 0xff;
	return sendwait(m,2,timeout_ms);
}  

uint8_t Dali::sendwait_byte(uint8_t tx_msg, uint32_t timeout_ms) {
	uint8_t m[3];
	m[0] = tx_msg;
	return sendwait(m,1,timeout_ms);
}  


void serialDali(void)
{
	char msg[9];
	
	if (Serial.available()){
		Serial.print("char: ");
		Serial.read(msg+bytes_rx);
		bytes_rx++;
		Serial.print(msg[bytes_rx]);
		Serial.println(";");
		if (bytes_rx == 9){
			if (msg[8] != '\n') return;
			exeCmd(reinterpret_cast<uint8_t *>(msg));
			bytes_rx = 0;
			Serial.println("Waiting for istructions...");
		}
	}
}

void serialDali_rx(Dali *d, uint8_t *data, uint8_t len)
{
	uint8_t buf[3];
	
	buf[0] = data[0] / 10;
	buf[1] = data[0] % 10;
	buf[2] = '\n';
	Serial.write(reinterpret_cast<const uint8_t *>(buf), 3);
}

void exeCmd(uint8_t *msg)
{
	uint8_t bus_n, addr_gr, sel, buf[2];
	
	bus_n = (msg[0]-48)*10 + (msg[1]-48);
	if (Master.bus[bus_n] == NULL)	return;
	addr_gr = (msg[3]-48)*10 + (msg[4]-48);
	if (addr_gr > 63) return;
	buf[1] = (msg[6]-48)*10 + (msg[7]-48);

	if (msg[5] == 'd') sel = 0;
	else if (msg[5] == 'c') sel = 1;
	else return;

	if (msg[2] == 's')
		buf[0] = (addr_gr << 1) | sel;
	else if (msg[2] == 'b')
		buf[0] = 0xFE | sel;
	else if (msg[2] == 'g'){
		if (addr_gr > 15) return;
		buf[0] = 0x80 | (addr_gr << 1) | sel;
	}else return;
	
	Master.bus[bus_n]->send(buf, 2);
	if (buf[1] >= 0x90 || buf[1] <= 0x9B || buf[1] >= 0xA0 || buf[1] <= 0xA5 || buf[1] >= 0xB0 || buf[1] <= 0xC4)
		Master.bus[bus_n]->EventHandlerReceivedData = &serialDali_rx;
	Serial.println("Comando inviato");
}




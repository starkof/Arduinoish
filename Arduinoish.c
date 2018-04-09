#include <MKL25Z4.h>
#include <string.h>

#define A ('A')
#define B ('B')
#define C ('C')
#define D ('D')
#define E ('E')

#define LOW (0)
#define HIGH (1)
#define ANALOG_IN (2)
#define ANALOG_OUT (3)
#define DIGITAL_IN (4)
#define DIGITAL_OUT (5)
#define SERIAL_IN (6)
#define SERIAL_OUT (7)

#define MASK(x) (1UL << (x))

typedef struct pin {
	char port;
	int bit;
}Pin;


void pinMode(Pin pin, int mode){
	char port = pin.port;
	int bit = pin.bit;
	
	if (mode == DIGITAL_OUT){
		if (port == A){
			// attach clock to port A
			SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
			PORTA->PCR[bit] &= ~PORT_PCR_MUX_MASK;
			PORTA->PCR[bit] |= PORT_PCR_MUX(1);
			// set bits to output
			PTA->PDDR |= MASK(bit);
		}
		else if (port == B){
			SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
			PORTB->PCR[bit] &= ~PORT_PCR_MUX_MASK;
			PORTB->PCR[bit] |= PORT_PCR_MUX(1);
			PTB->PDDR |= MASK(bit);
		}

		else if (port == C){
			SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
			PORTC->PCR[bit] &= ~PORT_PCR_MUX_MASK;
			PORTC->PCR[bit] |= PORT_PCR_MUX(1);
			PTC->PDDR |= MASK(bit);
		}
		else if (port == D){
			SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
			PORTD->PCR[bit] &= ~PORT_PCR_MUX_MASK;
			PORTD->PCR[bit] |= PORT_PCR_MUX(1);
			PTD->PDDR |= MASK(bit);
		}
		else if (port == E){
			SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
			PORTE->PCR[bit] &= ~PORT_PCR_MUX_MASK;
			PORTE->PCR[bit] |= PORT_PCR_MUX(1);
			PTE->PDDR |= MASK(bit);
		}
	}
	
	else if (mode == DIGITAL_IN){
		if (port == A){
			// attach clock to port A
			SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
			
			PORTA->PCR[bit] &= ~PORT_PCR_MUX_MASK;
			// enable pull-up resistors
			PORTA->PCR[bit] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK;;
			
			// clear bits to input
			PTA->PDDR &= ~MASK(bit);
		}
		else if (port == B){
			SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
			PORTB->PCR[bit] &= ~PORT_PCR_MUX_MASK;
			PORTB->PCR[bit] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK;;
			PTB->PDDR &= ~MASK(bit);
		}
		else if (port == C){
			SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
			PORTC->PCR[bit] &= ~PORT_PCR_MUX_MASK;
			PORTC->PCR[bit] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK;;
			PTC->PDDR &= ~MASK(bit);
		}
		else if (port == D){
			SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
			PORTD->PCR[bit] &= ~PORT_PCR_MUX_MASK;
			PORTD->PCR[bit] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK;;
			PTD->PDDR &= ~MASK(bit);
		}
		else if (port == E){
			SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
			PORTE->PCR[bit] &= ~PORT_PCR_MUX_MASK;
			PORTE->PCR[bit] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK;;
			PTE->PDDR &= ~MASK(bit);
		}
	}
}

int digitalRead(Pin pin){
	char port = pin.port;
	int bit = pin.bit;
	
	if (port == A){
		if (PTA->PDIR & MASK(bit)){
			return 1;
		}
		else{
			return 0;
		}
	}
	else if (port == B){
		if (PTB->PDIR & MASK(bit)){
			return 1;
		}
		else{
			return 0;
		}
	}
	else if (port == C){
		if (PTC->PDIR & MASK(bit)){
			return 1;
		}
		else{
			return 0;
		}
	}
	else if (port == D){
		if (PTD->PDIR & MASK(bit)){
			return 1;
		}
		else{
			return 0;
		}
	}
	else if (port == E){
		if (PTE->PDIR & MASK(bit)){
			return 1;
		}
		else{
			return 0;
		}
	}
	return -1;
}

void digitalWrite(Pin pin, int state){
	char port = pin.port;
	int bit = pin.bit;
	
	if (port == A){
		if (state == HIGH){
			PTA->PSOR = MASK(bit);
		}
		else if (state == LOW){
			PTA->PCOR = MASK(bit);
		}
	}
	else if (port == B){
		if (state == HIGH){
			PTB->PSOR = MASK(bit);
		}
		else if (state == LOW){
			PTB->PCOR = MASK(bit);
		}
	}
	else if (port == C){
		if (state == HIGH){
			PTC->PSOR = MASK(bit);
		}
		else if (state == LOW){
			PTC->PCOR = MASK(bit);
		}
	}
	else if (port == D){
		if (state == HIGH){
			PTD->PSOR = MASK(bit);
		}
		else if (state == LOW){
			PTD->PCOR = MASK(bit);
		}
	}
	else if (port == E){
		if (state == HIGH){
			PTE->PSOR = MASK(bit);
		}
		else if (state == LOW){
			PTE->PCOR = MASK(bit);
		}
	}
}

void toggle(Pin pin){
	char port = pin.port;
	int bit = pin.bit;
	
	if (port  == A){
		PTA->PTOR = MASK(bit);
	}
	else if (port  == B){
		PTB->PTOR = MASK(bit);
	}
	else if (port  == C){
		PTC->PTOR = MASK(bit);
	}
	else if (port  == D){
		PTD->PTOR = MASK(bit);
	}
	else if (port  == E){
		PTE->PTOR = MASK(bit);
	}
}

int analogRead(char port, int bit){
}

void analogWrite(char port, int bit){
}

void delay(volatile unsigned int time_del){
	volatile unsigned int del = time_del * 3000;
	while (del--){
		;
	}
}

void delay_micro(volatile unsigned int time_del){
	volatile unsigned int del = time_del * 3;
	while (del--){
		;
	}
}

void tone(){
}

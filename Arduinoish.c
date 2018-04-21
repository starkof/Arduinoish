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

// stores current masking state when interrpts are disableed
uint32_t interrupt_masking_state;

// new type func: pointer to function
typedef void (*func)(void);

// number of ISRs added to ports A and D
int len_portA_funcs = 0;
int len_portD_funcs = 0;

// arrays of interrupts attached to ports A and D
func portA_funcs[50];
func portD_funcs[50];


// type for storing a pin's port and bit
typedef struct pin {
	char port;
	int bit;
}Pin;


// setup pin for GPIO or analog interface
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
			PORTA->PCR[bit] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK;
			
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

// read current state of digital input pin
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

// write high or low to digital pin
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

// toggle the state of a digital pin
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

// not implemented
int analogRead(Pin pin){
	return 0;
}

// not implemented
void analogWrite(Pin pin, int analog_value){
}

void delay(volatile unsigned int time_del){
	time_del= time_del * 3000;
	while (time_del--){
		;
	}
}

void delay_micro(volatile unsigned int time_del){
	time_del = time_del * 3;
	while (time_del--){
		;
	}
}

void tone(Pin pin, unsigned int period, unsigned int duration){
	// period in mircoseconds and duration in milliseconds
	int n_loops =  (1000*duration)/period;
	
	for (int i = 0; i < n_loops; i++){
		toggle(pin);
		delay_micro(period/2);
		
		toggle(pin);
		delay_micro(period/2);
	}
}

void enable_interrupt(Pin pin, int priority){
	char port = pin.port;
	int bit = pin.bit;
	
	if (port == D){
		// enable interrupts on the pin
		PORTD->PCR[bit] |= PORT_PCR_IRQC(11);
		
		// configure NVIC
		NVIC_SetPriority(PORTD_IRQn, priority);
		NVIC_ClearPendingIRQ(PORTD_IRQn);
		NVIC_EnableIRQ(PORTD_IRQn);
	}
	else if (port == A){
		// enable interrupts on the pin
		PORTA->PCR[bit] |= PORT_PCR_IRQC(11);;
		
		// configure NVIC
		NVIC_SetPriority(PORTA_IRQn, priority);
		NVIC_ClearPendingIRQ(PORTA_IRQn);
		NVIC_EnableIRQ(PORTA_IRQn);
	}
	
	// enable interrupts in case they were disabled
		__enable_irq();
}

// need to save interrupt state before disabling
void noInterrupts(void){
	// disables interrupts
	interrupt_masking_state = __get_PRIMASK();
	__disable_irq();
}

// use saved interrupt state when re-enabling
void interrupts(void){
	// enables interrupts
	__set_PRIMASK(interrupt_masking_state);
	__enable_irq();
}

void PORTA_IRQHandler(void){
	for (int i=0; i < len_portA_funcs; i++){
		(*portA_funcs[i])();
	}
	PORTA->ISFR = 0xffffffff;
}

void PORTD_IRQHandler(void){
	for (int i=0; i < len_portD_funcs; i++){
		(*portD_funcs[i])();
	}
	PORTD->ISFR = 0xffffffff;
}

void attachInterrupts(Pin pin, void (*ISR)(void), int priority){
	// attaches a function that accepts nothing and returns nothing to the port ISR
	// functions will be run in the order they were added
	enable_interrupt(pin, priority);
	char port = pin.port;
	
	// add the new ISR to its port's ISR
	if (port == A){
		portA_funcs[len_portA_funcs] = ISR;
		len_portA_funcs += 1;
	}
	else if (port == D){
		portD_funcs[len_portD_funcs] = ISR;
		len_portD_funcs += 1;
	}
}

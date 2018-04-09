// Blinks an LED conneced to PTA1 if a switch connected to PTA2 is pressed

#include "duino_ish.h"


int main(void){
	Pin LED1 = {A, 1};
	
	Pin SW1 = {A, 2};
	
	pinMode(LED1, DIGITAL_OUT);
	
	pinMode(SW1, DIGITAL_IN);
	
	while (1){
		// PTA2 reads a zero when the switch is pressed		
		if (~digitalRead(SW1)){
			// blink the LED if the switch is pressed
			digitalWrite(LED1, HIGH);
			delay(100);

			digitalWrite(LD1, LOW);
			delay(100);
		}
	}
}

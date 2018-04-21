#ifndef DUINO_ISH_H
#define DUINO_ISH_H

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

typedef struct pin {
	char port;
	int bit;
}Pin;

void pinMode(Pin pin, int mode);

int digitalRead(Pin pin);

void digitalWrite(Pin pin, int state);

void toggle(Pin pin);

int analogRead(Pin pin);

void analogWrite(Pin pin, int analog_value);

void delay(volatile unsigned int time_del);

void delay_micro(volatile unsigned int time_del);

void attachInterrupts(Pin pin, void (*ISR)(void), int priority);

void noInterrupts(void);

void interrupts(void);

#endif

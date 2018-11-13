/*
 * GccApplication3.cpp
 *
 * Created: 17.10.2018 20.11.08
 * Author : Jørgen
 */


#include <avr/io.h>
#include <avr/interrupt.h>

#define F_CPU 1000000UL								//The system clock is 1MHz
#define LED_ON		PORTD |=(1<<PORTD2)				//Defines what LED_ON does (Sets PORT D Pin 2 High)
#define LED_OFF		PORTD &= ~(1<<PORTD2)			//Defines what LED_OFF does (Sets PORT D Pin 2 Low)

enum {UP, DOWN};



void PWM_SoftBlink(void);
void PWM_Init(void);
void millis_timer(uint8_t millis);


int main(void)
{
	sei();									//Enables interrupts by setting the global interrupt mask
	DDRB = 0x00;							//Configures Port B as an Input port
	DDRC = 0x00;							//Configures Port C as an Input port
	DDRD = 0b00000100;						//Configures half of Port D as an Input and half as output
	
	PORTB = 0xFF;							//Activates Pull up on all pins
	PORTC = 0xFF;							//Activates Pull up on all pins
	PORTD = 0b11111011;						//Activates Pull up on the input pins
	
	
	millis_timer(2);
	PWM_Init();
	
	while (1)
	{
		
		};
    

}

ISR(TIMER1_COMPA_vect)						//Interrupt service routine (Set LED_ON)
{
	LED_ON;
}

ISR(TIMER1_COMPB_vect)						//Interrupt service routine (Set LED_OFF)
{
	LED_OFF;
}

ISR(TIMER0_COMPA_vect)						//Interrupt service routine (calls PWM)
{
	PWM_SoftBlink();
}

void PWM_SoftBlink(void)					//Counts up and down to increase or decrease the duty cycle of the LED
{
	uint16_t Duty = OCR1B;
	uint16_t Period= OCR1A;
	static uint8_t direction;

	switch(direction)
	{
		case UP:
		if (++Duty ==(Period-1))
		direction=DOWN;
		break;
		
		case DOWN:
		if(--Duty == 2)
		direction = UP;
		break;
	}
	OCR1B = Duty;
}

void PWM_Init(void)							//Initializes the starting values of the PWM										
{
	TCCR1B |= (1<<CS10) | (1<<WGM12);		//no Prescalar, CTC mode
	TIMSK1 |= (1<<OCIE1A)|(1<<OCIE1B);		//Output compare IRQ (OCIE1A turns on, OCIE1B turns off
	OCR1A = 500;							//(F_CPU/1000*2*1 -1)
	OCR1B = 0;
}

void millis_timer(uint8_t millis)			//Makes a millisecond timer from the clock frequency 
{
	TCCR0A |=(1<<WGM01);					//Set to CTC mode
	TCCR0B |=(1<<CS02) | (1<<CS00) ;		//Prescalar 1024
	OCR0A = millis*0.48828-1;				//(millis*F_CPU/1000*2*1 -1)
	TIMSK0 |=(1<<OCIE0A);					//Output compare IRQ
}
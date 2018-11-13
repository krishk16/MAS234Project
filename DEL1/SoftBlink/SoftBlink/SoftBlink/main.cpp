/*
 * SoftBlink.cpp
 *
 * Created: 16/10/2018 15:38:26
 * Author : SnowFlake
 */ 

#define F_CPU 1000000

#include <avr/io.h>
#include<util/delay.h>

void sudoPwm(const int dutyCycle);
int expAppr(const int xx);

int main(void)
{
    //Dedicating port PD2 as output
	DDRD = 0x04;
	
	//Setting port PD2 high
	PORTD = 0x04;
	
    while (1) 
    {
		//Makes the led ramp up and down with a periode of two sec
		
		//Ramping up the light
		for(int ii = 0; ii < 100; ii++)
		{	
			sudoPwm(expAppr(ii));
		}
		//Ramping down the light
		for(int ii = 100; ii > 0; ii--)
		{
			sudoPwm(expAppr(ii));
		}
    }
}

void sudoPwm(const int dutyCycle)
{
	//set duty between 0-100 for 0-100%
	
	const int dutyTime = 100;	//us

	for(int ii = 0; ii < dutyTime ; ii++)
	{
		//Sets port PD2 high
		if(ii < dutyCycle)
		{
			PORTD = 0x04;
		}
		//Sets port PD2 low
		else
		{
			PORTD = 0x00;
		}
		
		_delay_us(100);
	}
	
}

int expAppr(const int xx)
{
	//Approximating a exp function with a third degree polynomial, mapping 0-100, to 0-100
	//Distributing the conversion factor to keep floats small
	
	const float val = (0.01f*(float)xx)*(0.1f*(float)xx)*(0.1f*(float)xx);
	return (int)val;
}
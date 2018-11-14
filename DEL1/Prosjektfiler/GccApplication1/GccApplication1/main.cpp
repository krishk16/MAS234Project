/*
 * GccApplication1.cpp
 *
 * Created: 15.10.2018 08:39:20
 * Author : Kriss
 */ 
#define F_CPU 1000000

#include <avr/io.h>
#include<util/delay.h>

int sB = 0;
int a = 1;
int period = 100;
int dutyCycle = 1;

int main(void)
{
	DDRD = 0x04; // Set PD2 as output
    while (1) 
    {
		for(int ii = 0; ii <= period; ++ii)
		{
			if( (ii - sB) <= dutyCycle)
			{
				PORTD = 0x04;	
			}
			
			else 
			{
				PORTD = 0x00;
			}
			
			_delay_us(1);
			
		}


	
    }
}


/* ultrason.c
 * Created: 07/07/2015 10:56:34
 *  Author: inès*/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <util/twi.h>
#include "I2C_MASTER.h"
#include "OS.h"
#include <util/delay.h>
void init();
int main(void)
{
	init();
	//TWI_Master_init(TWI_FREQ_SELECT(10000,1000000UL));
    OSInitialiser();
	sei();
	OSStart();
	return 0;
}
void init()
{	
	//Interruption on PD2 & PD3
	DDRD =(1<<DDD3)|(1<<DDD2);
	PORTD=(0<<PD2);
	EICRA |= 3<<ISC10 ;		//Interruption on falling edge because by default line is on HIGH
	EIMSK |= 1 <<INT1;		//Activate interruption
}

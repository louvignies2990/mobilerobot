#include <avr/io.h>
#include "ADC.h"
#include "ultrason.h"
void Init_ADC(){
	//  ADC: paramétrages
	// Reférence de tension: AVREF
	// Channel de départ: ADC1
	// ADC Left adjust : lire ADCH uniquement (précision sur 8bit)
	ADMUX |= ((0<<REFS1)|(0<<REFS0)|(1<<ADLAR)|(0<<MUX3)|(0<<MUX2)|(0<<MUX1)|(1<<MUX0));
	// Prescaler: 2 avec 1Mhz de clock de base
	// Freq ADC = 500kHz
	ADCSRA |= ((0<<ADPS2)|(0<<ADPS1)|(0<<ADPS0));	
}
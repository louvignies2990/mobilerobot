/* OS.c
 * Created: 08/03/2015 18:14:15
 *  Author: inès*/
#include "OS.h" 
//#include "usart.h"
#include "ultrason.h"
//#include "ADC.h"
//#include "dht.h"
//#include "spi.h"
#include <util/twi.h>
#include "I2C_Master.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
//#include <util/delay.h>
void ultrason(void);
//*********** VARIABLES GLOBALES *********************
unsigned char idx;
int vaultrason;
/*char capt;
char state;
int humidity;
char humid[10];
char IDCB_DHT22 = 0;			// mémorisation de l'identificateur créé lors de l'enregistrement de la Callback DHT22;
unsigned char IDCB_ADC = 0;	// mémorisation de l'identificateur créé lors de l'enregistrement de la Callback ADC*/
//void compas(void);
//void accelerometre();
//Variables pour CallBack Chrono
void (*MaCB[MAXCALLBACKCHRONO])(void);
unsigned int TempsCB[MAXCALLBACKCHRONO];
volatile unsigned int TickCB[MAXCALLBACKCHRONO];
struct hc_sr04
{
	uint8_t pin;
	uint8_t port;
	uint8_t ddr;
};
/*// ****************  OSInitialiser ******************************
// Initialise l'OS: initialise les variables et démarre le Timer
// *****************************************************************/
void OSInitialiser(void)
{
	unsigned char i;
	//Initialisation pour variables CallBack Chrono
	for (i=0; i<MAXCALLBACKCHRONO; i++)
	{
		MaCB[i] = 0;
		TempsCB[i] = 0;
	}
	//IDCB_ADC = OSEnregistrerCB_TIMER(ADCfct, 500);
}
/*// ****************  EnregistrerFonctionDeRappel ******************************
// Sauve l'adresse de la fonction à rappeler. Lorsque le nombre d'interruptions
// aura atteint temps millisecondes, le système rappellera la fonction
// **************************************************************************/
unsigned char OSEnregistrerCB_TIMER(void(*ptFonction)(void), unsigned int tps)
{
	unsigned int i=0;
	while (MaCB[i]!=0 && i<MAXCALLBACKCHRONO) i++;
	//S'il reste de la place on enregistre et on retourne 0 (opération réussie)
	if (i<MAXCALLBACKCHRONO)
	{
		MaCB[i] = ptFonction;
		TempsCB[i] = tps;
		TickCB[i] = 0; //Initialiser le compteur à 0;
		return i; // ID du call back
	}
	else return 255; //Il n'y a plus de place pour enregistrer un callback
}
/*// ****************  Retirer fonction de rappel ******************************
// Libère l'emplacement de la callback
// **************************************************************************/
void OSRetirerCB_TIMER(unsigned char IDCB)
{
	if ((IDCB >=0) && IDCB<MAXCALLBACKCHRONO)
	{
		MaCB[IDCB] = 0;
		TempsCB[IDCB] = 0;
	}
}
/*// ****************  Boucle principale de l'OS ******************************
// Boucle infinie qui attend des événement liés aux interruptions pour
// appeler les fonctions enregistrées
// **************************************************************************/
void OSStart()
{
	// Initialisation des interruptions, on autorise toutes les interruptions
	// Pour les interruptions particulières, voir chaque fonction
	sei();
	//Boucle principale de l'OS d'où on ne sort jamais
	for(;;)
	{
		// Check les conditions pour rappeler les fonctions liées au temps
		for (idx = 0; idx < MAXCALLBACKCHRONO; idx++)
		{
			if (MaCB[idx]) //Si on a l'adresse d'une fonction CB à cet index
			//Si on est arrivé au nombre de mS demandé, on appelle la fonction
			if (TickCB[idx] >= TempsCB[idx])
			{
				TickCB[idx] = 0;
				MaCB[idx]();  //Rappel de la fonction enregistrée!
			}
		}
	}
}
//void compas(void){
	////If we correctly send the message and we got a notification from Slave to get his datas
	//if(TWI_statusReg.currentState==TWI_STATE_REQUEST && TWI_statusReg.lastTransOK==1){
		//TWI_Master_getDatas(slave3Adress,10);
		//TWI_statusReg.currentState=TWI_STATE_SEND;
	//}
//}
void ultrason(void){
		PORTD=(1<<PD2);
		TCCR0A = (0<<COM0A1)|(0<<COM0A0)|(0<<COM0B0)|(0<<COM0B1)|(0<<WGM00)|(0<<WGM01)|(0<<WGM02);
		TCCR0B |= (0<<CS12)|(0<<CS11)|(1<<CS10);
		// Pour fOSC = 1 MHZ --> Valeur initiale du compteur = 65536 - 10 = 65526
		TCNT0=0;
		// Pour fOSC = 8 MHZ --> Valeur initiale du compteur = 65536 - 8000 = 57536
		//TCNT1H = 0xE0;
		//TCNT1L = 0xC0;
		// Autorisation de l'interruption en cas d'overflow
		TIMSK0 = (1<<TOIE0);
		_delay_us(10);
		PORTD=(0<<PD2);
		unsigned time=TCNT0;
	}
ISR(TIMER0_OVF_vect)
{
		TCCR0B |= (0<<CS12)|(0<<CS11)|(0<<CS10);
}
// Routines INTERRUPTIONS
// ISR ...
//Interruption TIMER1
ISR(TIMER1_OVF_vect)
{
		// Réinitialisation TIMER1
		// POUR 1 MHz
		TCNT1H = 0xEC;
		TCNT1L = 0x78;
		// POUR 8 MHz
		//TCNT1H = 0xE0;
		//TCNT1L = 0xC0;
		// Incrémenter tous les ticks
		unsigned char i;
		for (i = 0; i < MAXCALLBACKCHRONO; i++) TickCB[i]++;
		PORTB |= 1<<PINB4;
	}
//void accelerometre()
//{
	//if ( ! TWI_busy() )
	//{
		////If we correctly send the message and we got a notification from Slave to get his datas
		//if(TWI_statusReg.currentState==TWI_STATE_REQUEST && TWI_statusReg.lastTransOK==1){
			//TWI_Master_getDatas(slave1Adress,10);
			//TWI_statusReg.currentState=TWI_STATE_SEND;
		//}
	//}
//}
/* OS.h
 * Created: 08/03/2015 18:13:56
 *  Author: inès*/
#ifndef OS_H_
#define OS_H_
//Define the maximal number of callbacks
#define	MAXCALLBACKCHRONO		20
#define MAXBUFRS232				20
//Functions
/* @brief   OS initialisation
   @param   none
   @return  none*/
void OSInitialiser(void);
/* @brief   Register callback function linked with time
   @param   function pointer and time
   @return  ID associated to the recording in the OS*/
unsigned char OSEnregistrerCB_TIMER(void(*ptFonction)(void), unsigned int temps); 
/* @brief   Register callback function linked for the DHT22
   @param   function pointer
   @return  ID associated to the recording in the OS*/
//void OSEnregistrerCB_DHT22(void(*ptFonction)(volatile unsigned char*));
/* @brief   Register callback function for the ADC
   @param   function pointer 
   @return  ID associated to the recording in the OS*/
//void OSEnregistrerCB_ADC(void(*ptFonction)(volatile unsigned char*));
/* @brief   Remove callback function linked with time
   @param   ID associated to the recording in the OS
   @return  none*/
void OSRetirerCB_TIMER(unsigned char IDCB);
/* @brief   Remove callback function for DHT22
   @param   ID associated to the recording in the OS
   @return  none*/
//void OSRetirerCB_DHT22(void);
/* @brief   Remove callback function for the ADC
   @param   ID associated to the recording in the OS
   @return  none*/
//void OSRetirerCB_ADC(void);
/* @brief   Starting the OS
   @param   none
   @return  none*/
void OSStart(void);
int send_spi_signal(int i);
#endif /* OS_H_ */
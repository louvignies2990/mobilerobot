#ifndef TWI_H_
#define TWI_H_
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#if (__GNUC__ * 100 + __GNUC_MINOR__) < 304
#error "This library requires AVR-GCC 3.4 or later, update to newer AVR-GCC compiler !"
#endif
/************************************************************************/
/* Constants and macros                                                 */
/************************************************************************/
/* TWI Mode */
#define TWI_MASTER_ENABLED
//#define TWI_SLAVE_ENABLED
/** @brief  TWI Baudrate Expression
 *  @param  xtalCpu  system clock in Mhz, e.g. 4000000L for 4Mhz          
 *  @param  SCL frequency in kHz, e.g. 100, 400    
 */
#define TWI_FREQ_SELECT(xtalCpu, freq)  (xtalCpu/(2UL*freq*1000))-8UL //Refer to Datasheet to see calculus
/* Set size of receive and transmit buffers */
#ifndef TWI_RX_BUFFER_SIZE
#define TWI_RX_BUFFER_SIZE 64 /**< Size of the circular receive buffer, must be power of 2 */
#endif
#ifndef TWI_TX_BUFFER_SIZE
#define TWI_TX_BUFFER_SIZE 64 /**< Size of the circular transmit buffer, must be power of 2 */
#endif
/* TWI struct to know status of transmission */
struct TWI_statusReg
{
	uint8_t currentState;
	uint8_t lastTransOK;
};
volatile struct TWI_statusReg TWI_statusReg;
/************************************************************************/
/* Functions prototype                                                  */
/************************************************************************/
/**
   @brief   Initialize TWI in Master and set SCL frequency
   @param   Specify SCL frequency using macro TWI_FREQ_SELECT()
   @return  none
*/
extern void twi_master_init(uint8_t scl_frequency);
/**
 *  @brief   Put string to ringbuffer for transmitting via I2C & start transmission
 *			 Stop when nothing more to transmit
 *  @param   s string to be transmitted
 *  @return  none
 */
extern void twi_master_transmits(uint8_t slaveAddress, const char *s);
/**
 *  @brief   Put char to ringbuffer for transmitting via I2C & start transmission
 *			 Stop when nothing more to transmit
 *  @param   c char to be transmitted
 *  @return  none
 */
extern void twi_master_transmitc(uint8_t slaveAddress, uint8_t data);
/**
 *  @brief   Read x bytes from the slave
 *  @param   slaveAddress is the address of the slave
 *  @param   numberOfBytes to read from the slave
 *  @return  none
 */
extern void twi_master_read(uint8_t slaveAddress, uint8_t numberOfBytes);
/**
 *  @brief   Put regAdress+string to ringbuffer for transmitting via I2C & start transmission
 *			 Stop when nothing more to transmit
 *  @param   regAdress is the address of the register
 *  @param   s string to be transmitted
 *  @return  none
 */
extern void twi_master_transmitToRegister(uint8_t slaveAddress, uint8_t regAdress, uint8_t data);
/**
 *  @brief   Write the register address then read x bytes from the slave
 *  @param   slaveAddress is the address of the slave
 *  @param   regAdress is the address of the register
 *  @param   numberOfBytes to read from the slave
 *  @return  none
 */
extern void twi_master_readRegister(uint8_t slaveAddress, uint8_t regAdress, uint8_t numberOfBytes);
/**
 *  @brief   Initialize SPI in Slave Mode
 *  @param   address is the address of the slave
 *  @param   ?
 *  @return  none
*/
extern void twi_slave_init(uint8_t address, uint8_t address_mask);
/**
 *  @brief   Allow to set TWI_statusReg.lastTransOK to TRUE by matching the number
  *			 of bytes expected with the one received.
 *  @param   address is the address of the slave
 *  @param   numberOfBytes to expect
 *  @return  none
*/
extern void twi_slave_receive(uint8_t numberOfBytes);
/**
 *  @brief   transmit string to I2C and ACK the I2C communication
 *  @param   s string to be transmitted
 *  @return  none
 */
extern void twi_slave_transmit(const char *s);
/**
 *  @brief   Get received byte from ringbuffer
 *
 * Returns in the lower byte the received character and in the 
 * higher byte the last receive error.
 *
 *  @return  byte:  received byte from ringbuffer
 */
extern uint8_t twi_getc(void);
/**
 *  @brief   Return number of bytes waiting in the receive buffer
 *  @return  bytes waiting in the receive buffer
 */
extern uint8_t twi_available(void);
/**
 *  @brief   Flush bytes waiting in receive buffer
 */
extern void twi_flush(void);
#endif /* TWI_H_ */
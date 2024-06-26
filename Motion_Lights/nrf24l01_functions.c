/*
 * @file : nrf24l01_functions.c
 * @Author : Github.com/bheesma-10
 * @Brief : source file for nrf24l01 based radio module operations   
 */ 


#include "nrf24l01_functions.h"
#include "spi_functions.h"
#include "uart_functions.h"

#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Settings
uint8_t rx_address[5] = { 0xe7, 0xe7, 0xe7, 0xe7, 0xe7 };	// Read pipe address
uint8_t tx_address[5] = { 0xe7, 0xe7, 0xe7, 0xe7, 0xe7 };	// Write pipe address
#define READ_PIPE0		0									// Number of read pipe
#define READ_PIPE1      1

// AUTO_ACK can be disabled when running on 2MBPS @ <= 32 byte messages.
// 250KBPS and 1MBPS with AUTO_ACK disabled lost many packets
// if the packet size was bigger than 4 bytes.
// If AUTO_ACK is enabled, tx_address = rx_address.
#define AUTO_ACK		1								// Auto acknowledgment
#define DATARATE		RF_DR_1MBPS							// 250kbps, 1mbps, 2mbps
#define POWER			POWER_LOW							// Set power (MAX 0dBm..HIGH -6dBm..LOW -12dBm.. MIN -18dBm)
#define CHANNEL			0x4C								// 2.4GHz-2.5GHz channel selection (0x01 - 0x7C)
#define DYN_PAYLOAD		1								// Dynamic payload enabled
#define CONTINUOUS		0								// Continuous carrier transmit mode (not tested)

// ISR(INT0_vect) is triggered depending on config (only one can be true)
#define RX_INTERRUPT	0								// Interrupt when message is received (RX)
#define TX_INTERRUPT	0								// Interrupt when message is sent (TX)
#define RT_INTERRUPT	0								// Interrupt when maximum re-transmits are reached (MAX_RT)

// Used to store SPI data
uint8_t data_send = 0x00;
uint8_t data_read = 0x00;

/************************************************************************
 *@brief : function to send spi data sequence
 *@param : register address, pointer to data to be written at that register, 
           number of bytes of data to be written
 *@retval : None
************************************************************************/
void nrf24_send_spi(uint8_t register_address, void *data_comm, unsigned int bytes)
{
	cs_low();
	spi_transmit(&register_address,sizeof(uint8_t));
	spi_transmit((uint8_t*)data_comm,bytes);
	cs_high();
}

/************************************************************************
 *@brief : function to receive spi data 
 *@param : register address to read from, pointer to data storing variable, 
           number of bytes of data to be read
 *@retval : unsigned character byte
************************************************************************/
void nrf24_read_spi(uint8_t register_address, void *data_read, unsigned int bytes)
{
	cs_low();
	spi_transmit(&register_address,sizeof(uint8_t));
	spi_receive((uint8_t*)data_read,bytes);
	cs_high();
}

/************************************************************************
 *@brief : function to write data to NRF24L01 radio module
 *@param : register address, pointer to data to be written at that register,
           number of bytes of data to be written
 *@retval : None
 *@note : W_REGISTER is write command for spi
************************************************************************/
void nrf24_write(uint8_t register_address, uint8_t *data_comm, unsigned int bytes)
{
	nrf24_send_spi(W_REGISTER | register_address, data_comm, bytes);
}

/************************************************************************
 *@brief : function to read data to NRF24L01 radio module
 *@param : register address to read from, pointer to data storing variable,
           number of bytes of data to be read
 *@retval : None
 *@note : R_REGISTER is read command for spi
************************************************************************/
void nrf24_read(uint8_t register_address, uint8_t *data_comm, unsigned int bytes)
{
	nrf24_send_spi(R_REGISTER | register_address, data_comm, bytes);
}

/************************************************************************
 *@brief : function to initialize NRF24L01 radio module
 *@param : None
 *@retval : None
************************************************************************/
void nrf24_init(void)
{
	
	//set CE pin as output (CSN pin already set as output in spi_functions.c file)
	DDRB |= (1<<CE_PIN);    

	ce_low();
	cs_high();

    //allow the radio time to settle else configuration bits will not necessarily stick.
	_delay_ms(10);
	
	// Set retries
	data_send = 0xF3;				// Delay 4000us with 3 re-try (will be added in settings)
	nrf24_send_spi(W_REGISTER | SETUP_RETR,(uint8_t*)&data_send,1);

    //set data rate, power level and lna gain
	data_send = (DATARATE | POWER | 0x01);
	nrf24_send_spi(W_REGISTER | RF_SETUP,(uint8_t*)&data_send,1);
	
	// Disable Dynamic payload on all pipes
	data_send =
	~(DYN_PAYLOAD << DPL_P0) |
	~(DYN_PAYLOAD << DPL_P1) |
	~(DYN_PAYLOAD << DPL_P2) |
	~(DYN_PAYLOAD << DPL_P3) |
	~(DYN_PAYLOAD << DPL_P4) |
	~(DYN_PAYLOAD << DPL_P5);
	nrf24_send_spi(W_REGISTER | DYNPD,(uint8_t*) &data_send,1);

	// Enable Auto-acknowledge on all pipes
	data_send =
	(AUTO_ACK << ENAA_P5) |
	(AUTO_ACK << ENAA_P4) |
	(AUTO_ACK << ENAA_P3) |
	(AUTO_ACK << ENAA_P2) |
	(AUTO_ACK << ENAA_P1) |
	(AUTO_ACK << ENAA_P0);
	nrf24_send_spi(W_REGISTER | EN_AA,(uint8_t*)&data_send,1);

	// Open pipes
	nrf24_send_spi((W_REGISTER | RX_ADDR_P0),(uint8_t*)rx_address,5);
	nrf24_send_spi(W_REGISTER | TX_ADDR,(uint8_t*)tx_address,5);

	data_send = (1 << READ_PIPE0) | (1 << READ_PIPE1);
	nrf24_send_spi(W_REGISTER | EN_RXADDR,(uint8_t*)&data_send,1);

	// Set static payload size
	data_send = 0x20;
	nrf24_send_spi(W_REGISTER | RX_PW_P0,(uint8_t*)&data_send,1);

	// set address length
	data_send = 0x03;
	nrf24_send_spi(W_REGISTER | SETUP_AW,(uint8_t*)&data_send,1);

	// Set channel
	data_send = CHANNEL;
	nrf24_send_spi(W_REGISTER | RF_CH,(uint8_t*)&data_send,1);

	// Status - clear TX/RX FIFO's and MAX_RT by writing 1 into them
	data_send =
	(1 << RX_DR) |								// RX FIFO
	(1 << TX_DS) |								// TX FIFO
	(1 << MAX_RT);								// MAX RT
	nrf24_send_spi(W_REGISTER | STATUS,(uint8_t*)&data_send,1);

	// Flush TX/RX
	uint8_t flush_data = 0xFF;
	nrf24_send_spi(W_REGISTER | FLUSH_RX,&flush_data,0);
	nrf24_send_spi(W_REGISTER | FLUSH_TX,&flush_data,0);

    // clear config register 
	data_send =
	(1 << EN_CRC) |						// CRC enable
	(1 << CRC0) |						// CRC scheme
	(1 << PWR_UP);						// Power up	
	nrf24_send_spi(W_REGISTER | CONFIG,(uint8_t*)&data_send,1);
	
	_delay_ms(20);

    /*
	*print configuration details after setup
	*/
	char buffer[20];
	memset(buffer,0,sizeof(buffer));
	UART_SendString("nrf init finished\r\n");
	
	nrf24_read_spi(R_REGISTER | CONFIG,&data_read,1);
	sprintf(buffer,"Config:%d\r\n",data_read);
	UART_SendString(buffer);
	
	memset(buffer,0,sizeof(buffer));
	nrf24_read_spi(R_REGISTER | EN_AA,&data_read,1);
	sprintf(buffer,"EN_AA:%d\r\n",data_read);
	UART_SendString(buffer);
	
	memset(buffer,0,sizeof(buffer));
	nrf24_read_spi(R_REGISTER | EN_RXADDR,&data_read,1);
	sprintf(buffer,"EN_RXADDR:%d\r\n",data_read);
	UART_SendString(buffer);
	
	memset(buffer,0,sizeof(buffer));
	nrf24_read_spi(R_REGISTER | SETUP_RETR,&data_read,1);
	sprintf(buffer,"SETUP_RETR:%d\r\n",data_read);
	UART_SendString(buffer);
	
	memset(buffer,0,sizeof(buffer));
	nrf24_read_spi(R_REGISTER | RF_CH,&data_read,1);
	sprintf(buffer,"RF_CH:%d\r\n",data_read);
	UART_SendString(buffer);

	memset(buffer,0,sizeof(buffer));
	nrf24_read_spi(R_REGISTER | RF_SETUP,&data_read,1);
	sprintf(buffer,"RF_SETUP:%d\r\n",data_read);
	UART_SendString(buffer);
	
	memset(buffer,0,sizeof(buffer));
	nrf24_read_spi(R_REGISTER | STATUS,&data_read,1);
	sprintf(buffer,"STATUS:%d\r\n",data_read);
	UART_SendString(buffer);
	
	
}

/************************************************************************
 *@brief : function to send message from NRF24L01 radio module
 *@param : pointer to message to be sent, number of bytes of message to be sent
 *@retval : message sent status
 *@note : {"motion":0}/{"motion":1} is the current json message sent based on
	      motion detected 
************************************************************************/
uint8_t nrf24_send_message(void *tx_message, uint8_t len)
{
	uint8_t blank_len = 32 - len;

	// Start SPI, load message into TX_PAYLOAD
	uint8_t command = 0;
	cs_low();
	if (AUTO_ACK) {
		command = W_TX_PAYLOAD;
		spi_transmit(&command, sizeof(uint8_t));
	}else
	{ 
		command = W_TX_PAYLOAD_NOACK;
		spi_transmit(&command, sizeof(uint8_t));
	}
	spi_transmit((uint8_t*)tx_message, len);
	spi_transmit(0, blank_len);
	cs_high();
	
	// Send message by pulling CE high for more than 10us
	ce_high();
	_delay_us(100);
	ce_low();
	
	// Wait for message to be sent (TX_DS flag raised)
	nrf24_read_spi(R_REGISTER | STATUS,&data_read,1);
	
	while(!(data_read & ((1 << TX_DS) | (1<<MAX_RT)))) {
		nrf24_read_spi(R_REGISTER | STATUS,&data_read,1);
		_delay_us(100);
	}
	
	_delay_ms(20);
	
	// Status - clear TX/RX FIFO's and MAX_RT by writing 1 into them
	data_send =
	(1 << RX_DR) |								// RX FIFO
	(1 << TX_DS) |								// TX FIFO
	(1 << MAX_RT);								// MAX RT
	nrf24_send_spi(W_REGISTER | STATUS,&data_send,1);

	if(data_read & (1<<MAX_RT)){
		//flush tx
		uint8_t flush_data = 0xFF;
	    nrf24_send_spi(W_REGISTER | FLUSH_TX,&flush_data,0);
		return 0;  //max retries exceeded
	}
	
	
	return 1;
}

/************************************************************************
 *@brief : function to check message availability in NRF24L01 radio module
 *@param : None
 *@retval : message available status
************************************************************************/
unsigned int nrf24_available(void)
{
	uint8_t status_read = 0x00;
	nrf24_read_spi(R_REGISTER | FIFO_STATUS,&status_read,1);
	if (!(status_read & (1 << RX_EMPTY))) return 1;
	return 0;
}

/************************************************************************
 *@brief : function to read message from NRF24L01 radio module
 *@param : pointer to message reading array, number of bytes of message to be read
 *@retval : None 
************************************************************************/
void nrf24_read_message(void *buffer, uint8_t len)
{
	uint8_t blank_len = 32 - len;
	uint8_t blank_data[blank_len];

	// Read message
	uint8_t command = R_RX_PAYLOAD;
	cs_low();
	spi_transmit(&command,sizeof(uint8_t));
	spi_receive((uint8_t*)buffer,len);

	// read remaining blank data
	spi_receive(blank_data,blank_len);
	cs_high();

	// Clear RX interrupt
	data_send = (1 << RX_DR);
	nrf24_send_spi(W_REGISTER | STATUS,&data_send,1);

}

/************************************************************************
 *@brief : function to set radio module chip enable pin low
 *@param : None
 *@retval : None
************************************************************************/
void ce_low(void){
	PORTB &= ~(1<<CE_PIN);                                 //make CS pin LOW
}

/************************************************************************
 *@brief : function to set radio module chip enable pin high
 *@param : None
 *@retval : None
************************************************************************/
void ce_high(void){
	PORTB |= (1<<CE_PIN);                                  //make CS pin HIGH
}
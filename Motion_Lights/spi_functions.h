/*
 * @file : spi_functions.h
 * @Author : Github.com/bheesma-10
 * @Brief : header file for spi based functions   
 */ 

#ifndef _SPI_FUNCTIONS_H_
#define _SPI_FUNCTIONS_H_

#include <stdint.h>

#define F_CPU 8000000UL 

//macro
#define IRQ_PORT      PORTD 
#define IRQ_PIN       PIN2
//all below pins are in PORTB
#define CE_PIN        PIN3
#define SPI_CSN_PIN   PIN4
#define SPI_MOSI_PIN  PIN5
#define SPI_MISO_PIN  PIN6
#define SPI_SCK_PIN   PIN7

//structure
typedef struct  
{
	uint8_t mode;                //mode i.e., master or slave
	uint8_t data_order;          //msb or lsb first transmit
	uint8_t clock_phase;         //data clock phase for data transfer - rising edge or falling edge
	uint8_t clock_polarity;      //data clock polarity - low or high
	uint8_t spi2x_speed;         //double clock speed
	uint8_t baudrate_prescaler;  //pre-scaler to set clock frequency 	
}SPI_Handle_t;


//function declaration
void spi_init(SPI_Handle_t *spi_handle);
void spi_transmit(uint8_t *data, uint8_t datasize);
void spi_receive(uint8_t *data, uint8_t datasize);
void cs_low(void);
void cs_high(void);

#endif  /*_SPI_FUNCTIONS_H_*/
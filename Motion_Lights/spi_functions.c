/*
 * @file : spi_functions.c
 * @Author : Avinashee Tech
 * @Brief : source file for spi based functions   
 */ 

#include "spi_functions.h"
#include "uart_functions.h"

#include <stdio.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define UNUSED(x)   (void)(x)

/************************************************************************
 *@brief : function to initialize spi peripheral
 *@param : spi handler
 *@retval : None
************************************************************************/
void spi_init(SPI_Handle_t *spi_handle){
	
	//initialize spi parameters 
	spi_handle->mode = 1;
	spi_handle->data_order = 0;
	spi_handle->clock_phase = 0;
	spi_handle->clock_polarity = 0;
	spi_handle->spi2x_speed = 0;
	spi_handle->baudrate_prescaler = 4;
	 
	
	//setup mode
	if(spi_handle->mode == 1){
		DDRB |= (1<<SPI_MOSI_PIN) | (1<<SPI_CSN_PIN) | (1<<SPI_SCK_PIN);  //make MOSI,CSN and SCK pins as output
		DDRB &= ~(1<<SPI_MISO_PIN);                                       //make MISO pin as input
		
		SPCR |= (1<<SPE) | (1<<MSTR);                                     //peripheral enable and master mode selection	
	}else{
		DDRB &= ~((1<<SPI_MOSI_PIN)|(1<<SPI_SCK_PIN)|(1<<SPI_CSN_PIN));   //make MOSI, SCK, SS as input 
	    DDRB |= (1<<SPI_MISO_PIN);			                              //make MISO pin as output
	 
	    SPCR |= (1<<SPE);                                    //peripheral enable if not already done 
		SPCR &= ~(1<<MSTR);	                                              //slave mode selection
	}
	
	//setup data order
	if(spi_handle->data_order == 1){
		SPCR |= (1<<DORD);
	}else{
		SPCR &= ~(1<<DORD);
	}
	
	//setup clock phase for SPI
	if(spi_handle->clock_phase == 1){
		SPCR |= (1<<CPHA);
	}else{
		SPCR &= ~(1<<CPHA);
	}
	
	//setup clock polarity for SPI
	if(spi_handle->clock_polarity == 1){
		SPCR |= (1<<CPOL);
	}else{
		SPCR &= ~(1<<CPOL);
	}
	
	//setup clock doubler
	if(spi_handle->spi2x_speed == 1){
		SPSR |= (1<<SPI2X);
	}else{
		SPSR &= ~(1<<SPI2X);
	}
	
	//setup clock prescaler for SPI 
	if(spi_handle->spi2x_speed == 1){
		switch(spi_handle->baudrate_prescaler)
		{
			case 2: SPCR &= ~((1<<SPR0) | (1<<SPR1));
			       break;
			case 8: SPCR |= (1<<SPR0);
			        SPCR &= (1<<SPR1);
			       break;
			case 32: SPCR |= (1<<SPR1);
			         SPCR &= (1<<SPR0);
			       break;
			case 64: SPCR |= (1<<SPR0) | (1<<SPR1);
			       break;
			default:
			       break;
		}
	}else{
		switch(spi_handle->baudrate_prescaler)
		{
			case 4: SPCR &= ~((1<<SPR0) | (1<<SPR1));
			       break;
			case 16: SPCR |= (1<<SPR0);
			         SPCR &= (1<<SPR1);
			       break;
			case 64: SPCR |= (1<<SPR1);
			         SPCR &= (1<<SPR0);
			       break;
			case 128: SPCR |= (1<<SPR0) | (1<<SPR1);
			       break;
			default:
			       break;
		} 
	}
	
	cs_high();
	_delay_ms(20);
	
}

/************************************************************************
 *@brief : function to transmit spi data
 *@param : pointer to data to transmit, size of data to be transmitted
 *@retval : None
************************************************************************/
void spi_transmit(uint8_t *data, uint8_t datasize){
	
	uint8_t Tx_Counter = datasize;
	uint8_t flush_buffer = 0;

	while((Tx_Counter)>0U){
		
		
		UNUSED(flush_buffer);
		
		SPDR = (*data);
		data += sizeof(uint8_t);
		Tx_Counter--;
		
		while(!(SPSR & (1UL<<SPIF)));   //SPIF bit is cleared by first reading the SPSR with SPIF set, then accessing the SPDR
		flush_buffer = SPDR;
	}
	
}

/************************************************************************
 *@brief : function to read spi data
 *@param : pointer to data receiving variable, size of data to be received
 *@retval : None
************************************************************************/
void spi_receive(uint8_t *data, uint8_t datasize){
	
	uint8_t Tx_Counter = datasize;
	uint8_t Rx_Counter = datasize;
	
	while((Tx_Counter)>0U){
		uint8_t dummy_data = 0xFF;
        spi_transmit(&dummy_data,1);
		Tx_Counter--;
		
		(*data) = SPDR;
		data += sizeof(uint8_t);
		Rx_Counter--;
		
	}
	
}

/************************************************************************
 *@brief : function to set spi chip select pin low
 *@param : None
 *@retval : None
************************************************************************/
void cs_low(void){
	PORTB &= ~(1<<SPI_CSN_PIN);                                 //make CS pin LOW
}

/************************************************************************
 *@brief : function to set spi chip select pin high
 *@param : None
 *@retval : None
************************************************************************/
void cs_high(void){
	PORTB |= (1<<SPI_CSN_PIN);                                  //make CS pin HIGH 	
}

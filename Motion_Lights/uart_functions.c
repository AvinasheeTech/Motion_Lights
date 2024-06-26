/*
 * @file : uart_functions.c
 * @author : Github.com/bheesma-10
 * @brief : source file for uart based functions   
 */ 


#include "uart_functions.h"

#include <util/delay.h>
#include <stdio.h>
#include <avr/io.h>

#ifndef BAUD
#define BAUD 9600
#endif

uint16_t MYUBRR = F_CPU/16/BAUD - 1;


/************************************************************************
 *@brief : function for uart peripheral initialization
 *@param : None
 *@retval : None
 *@note : uart is used to display messages on a terminal software for
          debugging, you may or may not use it as per your needs.
		  Baud rate - 9600
************************************************************************/
void uart_init(void)
{
   	uint8_t UBL = 0x00;
	uint8_t UBH = 0x00;
	
	UBL = MYUBRR & 0x00FF;
	UBH = (MYUBRR & 0xFF00) >>8;                        //variable for baud rate
	
	UCSRB |= (1 << RXEN) | (1 << TXEN);                 /* Turn on transmission and reception */
	UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1);/* Use 8-bit character sizes */
	UBRRL = UBL;		                                /* Load lower 8-bits of the baud rate value */
	UBRRH = UBH ;	                                    /* Load upper 8-bits*/
}


/************************************************************************
 *@brief : function to receive character over uart
 *@param : None
 *@retval : unsigned character byte
************************************************************************/
unsigned char UART_RxChar()
{
	while ((UCSRA & (1 << RXC)) == 0);/* Wait till data is received */
	return(UDR);			/* Return the byte*/
}

/************************************************************************
 *@brief : function to transmit character over uart
 *@param : character byte to send
 *@retval : None
************************************************************************/
void UART_TxChar(char ch)
{
	while (! (UCSRA & (1<<UDRE)));	/* Wait for empty transmit buffer*/
	UDR = ch ;
	_delay_ms(1);
}

/************************************************************************
 *@brief : function to send string over uart
 *@param : string to be sent
 *@retval : None
************************************************************************/
void UART_SendString(char *str)
{
	unsigned char j=0;
	
	while (str[j]!=0)		/* Send string till null */
	{
		UART_TxChar(str[j]);
		j++;
	}
	
	_delay_ms(5);
}
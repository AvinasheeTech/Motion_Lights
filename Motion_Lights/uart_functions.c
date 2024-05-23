#include "uart_functions.h"

#include <util/delay.h>
#include <stdio.h>
#include <avr/io.h>

#ifndef BAUD
#define BAUD 9600
#endif



uint16_t MYUBRR = F_CPU/16/BAUD - 1;

void uart_init(void)
{
   	uint8_t UBL = 0x00;
	uint8_t UBH = 0x00;
	
	UBL = MYUBRR & 0x00FF;
	UBH = (MYUBRR & 0xFF00) >>8;
	
	UCSRB |= (1 << RXEN) | (1 << TXEN);/* Turn on transmission and reception */
	UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1);/* Use 8-bit character sizes */
	UBRRL = UBL;		/* Load lower 8-bits of the baud rate value */
	UBRRH = UBH ;	/* Load upper 8-bits*/
}



unsigned char UART_RxChar()
{
	while ((UCSRA & (1 << RXC)) == 0);/* Wait till data is received */
	return(UDR);			/* Return the byte*/
}

void UART_TxChar(char ch)
{
	while (! (UCSRA & (1<<UDRE)));	/* Wait for empty transmit buffer*/
	UDR = ch ;
	_delay_ms(1);
}

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
#ifndef STDIO_UART_H_
#define STDIO_UART_H_

#ifndef F_CPU
#define F_CPU 8000000UL
#endif

void uart_init(void);
void UART_SendString(char *str);
void UART_TxChar(char ch);
unsigned char UART_RxChar();

#endif /* STDIO_UART_H_ */

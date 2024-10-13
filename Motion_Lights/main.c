/*
 * @file : Motion_Lights.c
 * @author : Avinashee Tech
 * @brief : main project file  
 */ 


#include "spi_functions.h"
#include "json_decoder.h"
#include "nrf24l01_functions.h"
#include "uart_functions.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define MAX_FIELDS 2        //json max field param
#define Relay_Pin  0        //relay gpio pin

SPI_Handle_t spi_handle;    //spi handler

//	Used in IRQ ISR
volatile bool motion_detected = false;
volatile uint16_t counter = 0;
uint8_t seconds_passed = 0;

// Used to control whether this node is sending or receiving (nrf24L01)
bool role = false;        // true = TX role, false = RX role
char payload_buffer[30];  //buffer for message transmit and receive

/*AVR Json params*/
jsonNode_t *root;
jsonDecoderStatus_t json_status = JSON_DECODER_KEY_NOT_FOUND;


/***********************************************************************
 *@brief : ISR function to handle interrupt on INT1 pin 
 *@param : interrupt vector for respective interrupt
 *@retval : None
 *@note : this function is called when pin logic goes from low to high
          i.e. on rising edge
************************************************************************/
ISR(INT1_vect){
	motion_detected = true;
}

/***********************************************************************
 *@brief : function to initialize motion sensor settings 
 *@param : None
 *@retval : None
 *@note : interrupt based mechanism is chosen over polling
************************************************************************/
void motion_sensor_init(void){
	cli();                               //clear global interrupt mask
	MCUCR |= ((1<<ISC11) | (1<<ISC10));  //rising edge interrupt
	GICR |= (1<<INT1);                   //enable interrupt 1
	sei();                               //set global interrupt mask
}

/***********************************************************************
 *@brief : function to initialize adc peripheral of mcu 
 *@param : adc channel number
 *@retval : None
 *@note : adc is used to read LDR sensor values.
************************************************************************/
void adc_init(uint8_t channel){
	cli();                               //clear global interrupt mask
	ADMUX |= ((1<<REFS1) | (1<<REFS0));  //Internal 2.56v as voltage reference
	ADMUX |= (channel & 0x1F);           //select adc channel
	
	ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);   //clock prescaler - 128
	ADCSRA |= (1<<ADEN);                              //enable adc
	sei();                               //set global interrupt mask
}

/***********************************************************************
 *@brief : function to read adc data
 *@param : None
 *@retval : 16bit adc data
 *@note : actual adc value is only 10bit placed in two 8bit registers
************************************************************************/
uint16_t adc_read(void){
	
	ADCSRA |= (1<<ADSC);  //start conversion
	_delay_us(100);
	uint16_t adc_val = (uint16_t)((ADCL) | (ADCH<<8));
	
	return adc_val;
	
}

/***********************************************************************
 *@brief : function to initialize timer0 peripheral of mcu 
 *@param : None
 *@retval : None
 *@note : timer0 keeps count of seconds passed for timeout calculation. 
          Also, mode selected for the timer is CTC.
************************************************************************/
void timer0_init(void){
	 cli();                            //clear global interrupt mask
	 TCCR0 |= (1<<CS01) | (1<<CS00);   //clock selected is divided by 64.
	 TCCR0 |= (1<<WGM01);              //sets mode to CTC
	 OCR0 = 0x7C;                      //sets TOP to 124 so the timer will overflow every 1 ms.
	 TIMSK |= (1<<OCIE0);              //Output Compare Match A Interrupt Enable
	 sei();                            //set global interrupt mask 
}

/***********************************************************************
 *@brief : ISR function to handle interrupt on timer0 compare match
 *@param : interrupt vector for respective interrupt
 *@retval : None
 *@note : this function is called every 1ms 
************************************************************************/
ISR(TIMER0_COMP_vect)
{
	counter ++;                  
	if(counter==1000){
		seconds_passed++;
		counter = 0;
	}
}

/***********************************************************************
 *@brief : function to initialize radio module according to role selected
 *@param : role - false implies RX role and true implies TX role
 *@retval : sent or receive acknowledgement
 *@note : both modules stay in receiver mode by default
************************************************************************/
int radio_operation(bool role){
	/*When the nRF24L01 is in power down it must always settle in Standby for 1.5ms
	before it can enter one of the TX or RX modes.*/
	if(role){
		//shift to tx role
		uint8_t data_config = 0x08;
		nrf24_send_spi(W_REGISTER | CONFIG,(uint8_t*)&data_config,1); //power down
		_delay_ms(1);
		ce_low();
		_delay_ms(1);
		data_config = 	(1 << EN_CRC) |						// CRC enable
		(1 << CRC0) |						// CRC scheme
		(1 << PWR_UP) |						// Power up
		~(1<<PRIM_RX);
		nrf24_send_spi(W_REGISTER | CONFIG,(uint8_t*)&data_config,1);
		
		
		memset(payload_buffer,0,sizeof(payload_buffer));
		sprintf(payload_buffer,"{\"motion\":%d}\r\n",motion_detected?1:0);
		UART_SendString((char*)payload_buffer);
		_delay_ms(100);
		uint8_t sent = nrf24_send_message(payload_buffer,strlen(payload_buffer));
		_delay_ms(100);
		if(sent){
			UART_SendString("Transfer success\r\n");
			_delay_ms(100);
			return 1;
		}else{
			UART_SendString("Transfer failed\r\n");
			_delay_ms(100);
			return 0;
		}
		
	}else{
		//shift to rx role
		uint8_t data_config = 0x08;
		nrf24_send_spi(W_REGISTER | CONFIG,(uint8_t*)&data_config,1);  //power down
		_delay_ms(1);
		data_config = (1 << EN_CRC) |		// CRC enable
		(1 << CRC0) |						// CRC scheme
		(1 << PWR_UP) |						// Power up
		(1<<PRIM_RX);
		nrf24_send_spi(W_REGISTER | CONFIG,(uint8_t*)&data_config,1);
		ce_high();
		
		_delay_ms(1);
		
		memset(payload_buffer,0,sizeof(payload_buffer));
		if(nrf24_available()){
			nrf24_read_message(payload_buffer,sizeof(payload_buffer));
			UART_SendString((char*)payload_buffer);
		}
		_delay_ms(100);
	
	    /*Json decoding*/
		int received_motion = 0;
		json_status = JSON_DECODER_fromString(payload_buffer);
		if(json_status==JSON_DECODER_OK){
			UART_SendString("json ok\r\n");
		    _delay_ms(100);
		    JSON_DECODER_getRoot(&root);
		    JSON_DECODER_getNumber(root,"motion",&received_motion);
		    _delay_ms(100);
			return received_motion;
		}else{
			UART_SendString("json not ok\r\n");
			_delay_ms(100);
			return 0;
		}
		
	}	
	
}

/************************************************************************
 *@brief : function for peripheral initialization of system 
 *@param : None
 *@retval : None
 *@note : calls respective peripheral init functions
************************************************************************/
void init_system(void){
		//Initialize SPI
		spi_init(&spi_handle);
		_delay_ms(100);
		
		//Initialize UART
		uart_init();
		_delay_ms(100);

		//Initialize nRF24L01
		nrf24_init();
		_delay_ms(100);
		
		cs_high();  //chip select pin set to high logic before start
		
		//Initialize ADC
		adc_init(0x01);   
		
		//Initialize Timer0
		timer0_init();
		
		//Initialize motion sensor
		motion_sensor_init();
		
		//Initialize relay pin
		DDRA |= (1<<Relay_Pin);  //relay pin as output PA0
		PORTA |= (1<<Relay_Pin); 
}

/***********************************************************************
 *@brief : function to enable peripheral of mcu 
 *@param : None
 *@retval : None
************************************************************************/
void enable_peripherals(void){
	SPCR |= (1<<SPE);                    //enable SPI
	UCSRB |= ((1 << RXEN) | (1 << TXEN)); /* Turn on UART transmission and reception */
	ADCSRA |= (1<<ADEN);                 //enable ADC
}

/***********************************************************************
 *@brief : function to disable peripheral of mcu 
 *@param : None
 *@retval : None
************************************************************************/
void disable_peripherals(void){
	SPCR |= ~(1<<SPE);                    //disable SPI
	UCSRB |= (~(1 << RXEN) | ~(1 << TXEN)); /* Turn off UART transmission and reception */
	ADCSRA |= ~(1<<ADEN);                 //disable ADC
}
  
int main(void)
{
    //Initialize system
	init_system();	
	
	uint8_t lights_on = 0;       //variable indicating light status
	
    while (1) 
    {
        
		int received_motion = radio_operation(role);
		uint16_t ldr_value = adc_read();
		_delay_ms(10);
		
		/*action to be taken when self motion triggered*/
        if(motion_detected && ldr_value<550){       //Only consider motion in night/no light condition
			PORTA &= ~(1<<Relay_Pin);               //Close Relay N.O. Contact
			counter = 0;                            //reset counter
			seconds_passed = 0;                     //reset seconds counter
			lights_on = 1;
			role = true;
			int tx_status = radio_operation(role);  //send message to other node
			while((!tx_status) && (seconds_passed<30)){  //in case of transmit fail after 30seconds -> timeout
				tx_status = radio_operation(role);
			}	
			motion_detected = false;                //variable defaults to false
			role = false;                           //revert back to receiver role		
		}else{
			asm("nop");                             //Do nothing if motion detected in light condition
			motion_detected = false;
			if(lights_on && seconds_passed>=120){   //If light on due to previous motion detection, turn off after 2 minutes
				PORTA |= (1<<Relay_Pin);
				lights_on = 0;
			}
		}
        
		
		/*action to be taken when other node sends motion data*/
		if((received_motion==1) && (role==false)){
			PORTA &= ~(1<<Relay_Pin);               //Close Relay N.O. Contact
			counter = 0;                            //reset counter
			seconds_passed = 0;                     //reset seconds counter
			lights_on = 1;
		}else{
			asm("nop");
			if(lights_on && seconds_passed>=120){   //If light on due to previous motion detection, turn off after 2 minutes
				PORTA |= (1<<Relay_Pin);
				lights_on = 0;
			}
		}
		
    }
}



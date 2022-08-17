/********************************************************************************
 Written by: Rugved Katole, BITS Pilani  


 Date: 28th February 2022
 
 Application example: Motion Planning using Serial communication over USB RS232

 
 Serial Port used: UART2
********************************************************************************/



#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

#include <math.h>
#include  <stdlib.h>
#define pi 3.1415926535897932384626433832795
unsigned char FBR_Flag=0;   					//Keeps track of whether a wheel is moving forward of back. 1 for forward.
unsigned char FBL_Flag=0;


unsigned char data;				//to store received data from UDR2
unsigned char incomingByte;	
int packet_cnt=0,packet_len=4; 
char d[4]; 

float BATT_Voltage, BATT_V;


void motion_pin_config (void)
{
 DDRA = DDRA | 0x0F;			// refer table 3.3 hardware manual	
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;		//Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18;		//PL3 and PL4 pins are for velocity control using PWM.
}

//ADC pin configuration
void adc_pin_config (void)
{
 DDRF = 0x00; //set PORTF direction as input
 PORTF = 0x00; //set PORTF pins floating
 DDRK = 0x00; //set PORTK direction as input
 PORTK = 0x00; //set PORTK pins floating
}

//Function to initialize ports
void port_init()
{
	motion_pin_config();
}


void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
	For Overriding normal port functionality to OCRnA outputs.
	{WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

// Function for robot velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}


//Function To Initialize UART2
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
// parity: Disabled
void uart2_init(void)
{
 UCSR2B = 0x00;		//disable while setting baud rate
 UCSR2A = 0x00;
 UCSR2C = 0x06;
 UBRR2L = 0x5F;		//set baud rate lo
 UBRR2H = 0x00;		//set baud rate hi
 UCSR2B = 0x98;
}


ISR(USART2_RX_vect)		// ISR for receive complete interrupt
{	/*********************************************************************************
	We will take four bytes of data at a time ,t,he first is an escape character 0x7E.
	This is followed by a direction character 8,6,4,2,5 for F,R,L,B,Stop respectively.
	Final two bytes indicating the PWM input to each motor.
	*********************************************************************************/
	incomingByte = UDR2; 
	d[packet_cnt]=incomingByte;
	packet_cnt++;
	if (d[0]!='A')
	packet_cnt=0; 
	if( packet_cnt>=packet_len && d[0]=='A')
	{
		packet_cnt=0;	
		velocity((int)d[2],(int)d[3]);													  
		if(d[1] == 0x38) //ASCII for '8'	//ASCII value of 8
		{
			PORTA=0x06;															//forward
			//Both wheels move forward
			FBL_Flag=1;
			FBR_Flag=1;
		}

		if(d[1] == 0x32) //ASCII for '2'	//ASCII value of 2
		{
			PORTA=0x09;															//back
			//Both wheels move back.
			FBL_Flag=0;
			FBR_Flag=0;
		}

		if(d[1] == 0x34) //ASCII for '4'		//ASCII value of 4
		{
			PORTA=0x05;															//left
			//Right wheel should move with forward and LEft backward for perfect left turn.
			FBL_Flag=0;
			FBR_Flag=1;
		}

		if(d[1] == 0x36) //ASCII for '6' //ASCII value of 6
		{
			PORTA=0x0A;															//right
			//Left wheel should move with forward and right backward for perfect right turn..
			FBL_Flag=1;
			FBR_Flag=0;
		}

		if(d[1] == 0x35) //ASCII for '5'	//ASCII value of 5
		{
			PORTA=0x00;					//stop
		}

	}

}


//Function To Initialize all The Devices
void init_devices()
{
 cli();				//Clears the global interrupts
 port_init();		//Initializes all the ports
 uart2_init();		//Initailize UART2 for serial communiaction
 timer5_init();
 sei();				//Enables the global interrupts
}



//Main Function
int main(void)
{
	init_devices();
	while(1)
	{	

	}
}
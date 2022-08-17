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

int FBR_Flag = 0;   					//Keeps track of whether a wheel is moving forward of back. 1 for forward.
int FBL_Flag = 0;

long int ShaftCountLeft = 0;		//to keep track of left position encoder 
long int ShaftCountLeftPrev = 0;	//to keep track of left position encoder
long int ShaftCountRight = 0;		//to keep track of right position encoder
long int ShaftCountRightPrev = 0;	//to keep track of right position encoder

unsigned char data;				//to store received data from UDR2
unsigned char incomingByte;	
unsigned int packet_cnt = 0, packet_len = 5; 
unsigned char pwm_data[5]; 
unsigned char ADC_Conversion(unsigned char);
unsigned char sharp, distance, adc_reading;
unsigned int value = 0;
unsigned int sensor_cnt = 0, sensor_len = 5;
float volt = 9.6;

//buzzer is connected to PORTC 3 pin of Microcontroller
//PORTC 3 pin is configured as output with the initial state set at logic 0 to keep the buzzer off.
void buzzer_pin_config (void)
{
DDRC = DDRC | 0x08;     //setting PORTC 3 as output
PORTC = PORTC & 0xF7;	//setting PORTC 3 logic low to turnoff buzzer
}

// Bargraph LEDs are connected to PORTJ of MCU'
void LED_bargraph_config (void)
{
DDRJ = 0xFF; //PORT J is configured as output
PORTJ = 0x00; //Output is set to 0
}

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

//Function to configure INT4 (PORTE 4) pin as input for the left position encoder
void left_encoder_pin_config (void)
{
	 DDRE  = DDRE & 0xEF;			//Set the direction of the PORTE 4 pin as input
	 PORTE = PORTE | 0x10;			//Enable internal pull-up for PORTE 4 pin
}

// //Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
	 DDRE  = DDRE & 0xDF;			//Set the direction of the PORTE 4 pin as input
	 PORTE = PORTE | 0x20;			//Enable internal pull-up for PORTE 4 pin
}

//Function to configure Interrupt switch
void interrupt_switch_config (void)
{
 DDRE = DDRE & 0x7F;  //PORTE 7 pin set as input  
 PORTE = PORTE | 0x80; //PORTE7 internal pull-up enabled
}


//Function to initialize ports
void port_init()
{
	motion_pin_config();
	left_encoder_pin_config();			//left encoder pin config
    right_encoder_pin_config();			//right encoder pin config	
	buzzer_pin_config();
	LED_bargraph_config();
	adc_pin_config();
	interrupt_switch_config();
}

//Function to Initialize ADC
void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//This Function accepts the Channel Number and returns the corresponding Analog Value 
unsigned char ADC_Conversion(unsigned char Ch)
{
	unsigned char a;
	if(Ch > 7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;  			
	ADMUX = 0x20| Ch;	   		
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10) == 0);	//Wait for ADC conversion to complete
	a = ADCH;
	ADCSRA = ADCSRA | 0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

// This Function calculates the actual distance in millimeters(mm) from the input
// analog value of Sharp Sensor. range 40 mm to 300 mm

unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading)
{
	unsigned int distanceInt;
	distanceInt = (int)(10.00 * (1/(0.001240875 * adc_reading + 0.005)));
	if(distanceInt > 800)
	{
		distanceInt = 800;
	}
	return distanceInt;
}


// Function to enable Interrupt 4
void left_position_encoder_interrupt_init (void) 
{
	 cli();						// Clears the global interrupt
	 EICRB = EICRB | 0x02;		// INT4 is set to trigger with falling edge
	 EIMSK = EIMSK | 0x10;		// Enable Interrupt INT4 for left position encoder
	 sei();						// Enables the global interrupt 
}

// // Function to enable Interrupt 5
void right_position_encoder_interrupt_init (void) 
{
	 cli();						// Clears the global interrupt
	 EICRB = EICRB | 0x08;		// INT5 is set to trigger with falling edge
	 EIMSK = EIMSK | 0x20;		// Enable Interrupt INT5 for right position encoder
	 sei();						// Enables the global interrupt 
}

// Timer 5 initialized in PWM mode for velocity control
// Prescale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
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
	pwm_data[packet_cnt] = incomingByte;
	//UDR2=incomingByte;
	packet_cnt++;
	if (pwm_data[0] != 'P')
	packet_cnt = 0; 
		
	if(packet_cnt >= packet_len && pwm_data[0] == 'P')
	{	
		if ((int) pwm_data[4] != ((int) pwm_data[2] + (int) pwm_data[3]) % 256)
		{
		packet_cnt = 0;	
		velocity((int)pwm_data[2], (int)pwm_data[3]);	
		
		if(pwm_data[1] == 0x32) //ASCII for '1'	
		{
			PORTA = 0x08;	//Right wheel no rotation and left backwards.
			FBL_Flag = -1;
			FBR_Flag = 0;
		}

		if(pwm_data[1] == 0x32) //ASCII for '2'	
		{
			PORTA = 0x09;	//Both wheels move back.
			FBL_Flag = -1;
			FBR_Flag = -1;
		}

		if(pwm_data[1] == 0x33) //ASCII for '3'		
		{
			PORTA = 0x01;	//left wheel no rotation and right backwards.
			FBL_Flag = 0;
			FBR_Flag = -1;
		}

		if(pwm_data[1] == 0x34) //ASCII for '4'		
		{
			PORTA = 0x0A;	//Right wheel should move with forward and left backward for perfect left turn.
			FBL_Flag = -1;
			FBR_Flag = 1;
		}

		if(pwm_data[1] == 0x35) //ASCII for '5'
		{
			PORTA = 0x00;	//stop
			FBL_Flag = 0;
			FBL_Flag = 0;
		}

		if(pwm_data[1] == 0x36) //ASCII for '6' 
		{
			PORTA = 0x05;	//Left wheel should move with forward and right backward for perfect right turn..
			FBL_Flag = 1;
			FBR_Flag = -1;
		}

		if(pwm_data[1] == 0x37) //ASCII for '7'		
		{
			PORTA = 0x02;	//Right wheel should move with forward and left no rotation.
			FBL_Flag = 0;
			FBR_Flag = 1;
		}
		
		if(pwm_data[1] == 0x38) //ASCII for '8'	
		{
			PORTA = 0x06; //Both wheels move forward
			FBL_Flag = 1;
			FBR_Flag = 1;
		}

		if(pwm_data[1] == 0x39) //ASCII for '9' 
		{
			PORTA = 0x04;	//Left wheel should move with forward and right no rotation.
			FBL_Flag = 1;
			FBR_Flag = 0;
		}
		}
		else
		{
			packet_cnt = 0;
		}
	}
}

//Interrupt which runs every 100 ms. We use this to send data of the current encoder position.
void timer4_init(void) 
{
 TCCR4B = 0x00; //stop
 TCNT4H = 0xF7; //247 //Counter higher 8 bit value
 TCNT4L = 0x00; //0+247*256 = 63232 to reach 65535 we need 2303 counts.
 OCR4AH = 0x00; //Output Compair Register (OCR)- Not used Since TIMSK=0x01 we only use the overflow counter.
 OCR4AL = 0x00; //Output Compair Register (OCR)- Not used
 OCR4BH = 0x00; //Output Compair Register (OCR)- Not used
 OCR4BL = 0x00; //Output Compair Register (OCR)- Not used
 OCR4CH = 0x00; //Output Compair Register (OCR)- Not used
 OCR4CL = 0x00; //Output Compair Register (OCR)- Not used
 ICR4H  = 0x00; //Input Capture Register (ICR)- Not used
 ICR4L  = 0x00; //Input Capture Register (ICR)- Not used
 TCCR4A = 0x00; //This ensures normal mode as WGM0,1 are both 0. In addition WGM2 in in TCCR4B (in the 4th bit from right) is also 0
 TCCR4C = 0x00; //Just set to zero for now.
 TCCR4B = 0x04; //start Timer 0x04. Prescaler is 64 for CS0:2 set to 011. 14745600/64=230400  
}

//Function To Initialize all The Devices
void init_devices()
{
 cli();				//Clears the global interrupts
 port_init();		//Initializes all the ports
 uart2_init();		//Initailize UART2 for serial communiaction
 adc_init();
 timer5_init();
 timer4_init();
 TIMSK4 = 0x01;    //Enables the overflow interrupt.
 left_position_encoder_interrupt_init();
 right_position_encoder_interrupt_init();
 sei();				//Enables the global interrupts
}

//If this doesn't work then an interrupt can also be used. Refer page 106 of Software Manual
void USART_Transmit( unsigned char data )										
{
/* Wait for empty transmit buffer*/
while( !( UCSR2A & (1<<UDRE2)) )
;
/* Put data into buffer, sends the data*/
UDR2 = data;
}


ISR(TIMER4_OVF_vect)
{
char chksum;
TCCR4B = 0x00;			//Stops clock.
/*Apparently gives 0.2s gap or 5Hz. Should have been 20Hz */
//TCNT4H = 0xD2; 		//210 
//TCNT4L = 0xFF; 		//255+210*256 = 54015 to reach 65535 we need 11520 counts.

/*Apparently gives 0.04s gap 25Hz. Should have been 100Hz*/
TCNT4H = 0xF7; 			//247 
TCNT4L = 0x00; 			//0+247*256 = 63232 to reach 65535 we need 2303 counts.

/*Apparently gives 0.01s gap. But random stuff creep in so we will keep 25Hz*/
//TCNT4H = 0xFD; 			//253 
//TCNT4L = 0xBF; 			//191+253*256 = 64959 to reach 65535 we need 577 counts.

TCCR4B =  0x04;			// Restarts clock with 64 prescaler

//Here we just need to send the encoder positions. 

USART_Transmit('E');					//'A' denotes the start of the sequence of data to be sent.
USART_Transmit(ShaftCountRight/256);	//Sending the 4 bytes of encoder data.
USART_Transmit(ShaftCountRight%256);
USART_Transmit(ShaftCountLeft/256);
USART_Transmit(ShaftCountLeft%256);
chksum=ShaftCountRight/256+ShaftCountRight%256+ShaftCountLeft/256+ShaftCountLeft%256; 
USART_Transmit(chksum);	//Send the calculated checksum for comparison and accuracy check.

// sharp = ADC_Conversion(11);						//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
// value = Sharp_GP2D12_estimation(sharp);				//Stores Distance calsulated in a variable "value".
// USART_Transmit(value/256);
// USART_Transmit(value%256);

//Transmiting Proximity Sensor data located on channel 4,5,6,7,8 covering 180 deg(left to right) on front side of robot 
				
USART_Transmit(ADC_Conversion(4)/256);
USART_Transmit(ADC_Conversion(4)%256);

USART_Transmit(ADC_Conversion(5)/256);
USART_Transmit(ADC_Conversion(5)%256);

// USART_Transmit(ADC_Conversion(6)/256);   //useless sensor
// USART_Transmit(ADC_Conversion(6)%256);

USART_Transmit(ADC_Conversion(7)/256);
USART_Transmit(ADC_Conversion(7)%256);

USART_Transmit(ADC_Conversion(8)/256);
USART_Transmit(ADC_Conversion(8)%256);

// Transmiting sharp IR sensor reading in mm 

sharp = ADC_Conversion(11);						//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
value = Sharp_GP2D12_estimation(sharp);				//Stores Distance calsulated in a variable "value".
USART_Transmit(value/256);
USART_Transmit(value%256);

// Transmiting White Line Sensor Data Channel (1,2, 3)

USART_Transmit(ADC_Conversion(1)/256);   //right WL sensor
USART_Transmit(ADC_Conversion(1)%256);

USART_Transmit(ADC_Conversion(2)/256);    // middle Wl sensor
USART_Transmit(ADC_Conversion(2)%256);

USART_Transmit(ADC_Conversion(3)/256);    //left WL sensor
USART_Transmit(ADC_Conversion(3)%256);


}

/************************************************************************************************* 
The interrupt above will run start at 54015 and go upto 65535 which is 11520 counts.
Further the overflow interrupt on going from 65535 to 0. So 11521 counts before interrupts. 
counts of system clock 11521*64 = 737364. 0.05000569 is the time between steps.

To get a time step of 0.01 secs we need to get a count of 2304 or 1/5 of 11520. 65535-2304 = 63231.
We add one for the step to 0. 63232 should be the start coount. In hex this is 
**************************************************************************************************/


//ISR for right position encoder
ISR(INT5_vect)  
{
	if(FBR_Flag == 1)
	ShaftCountRight++;			//increment right shaft position count for forward motion
	else if(FBR_Flag == -1)
	ShaftCountRight--;
}

// //ISR for left position encoder
ISR(INT4_vect)
{
	if(FBL_Flag == 1)
	ShaftCountLeft++;			//increment left shaft position count for forward motion
	else if (FBL_Flag == -1)
	ShaftCountLeft--;			//decrement left shaft position count for backward motion			
}

//Main Function
int main(void)
{
	init_devices();
	while(1)
	{	
		// unsigned long measurement=256;
		// BATT_Voltage = 0.55;
		// BATT_Voltage = ADC_Conversion(0);
		// BATT_Voltage = (((ADC_Conversion(0))*0.046));
		BATT_Voltage = ((ADC_Conversion(0)*100)*0.07902) + 0.7;	//Prints Battery Voltage Status
		
		if (BATT_Voltage < 0x28A){
			PORTJ = 0x80; //Output is set to 1 bar
		}
		else if (BATT_Voltage < 0x2BC){
			PORTJ = 0xC0; //Output is set to 2 bars
		}
		else if (BATT_Voltage < 0x2EE){
			PORTJ = 0xE0; //Output is set to 3 bars
		}
		else if (BATT_Voltage < 0x320){
			PORTJ = 0xF0; //Output is set to 4 bars34                                                                                     
		}
		else if (BATT_Voltage < 0x352){
			PORTJ = 0xF8; //Output is set to 5 bars
		}
		else if (BATT_Voltage < 0x384){
			PORTJ = 0xFC; //Output is set to 6 bars
		}
		else if (BATT_Voltage < 0x3B6){
			PORTJ = 0xFE; //Output is set to 7 bars
		}
		else {
			PORTJ = 0xFF; //Output is set to 8 bars
		}

		// USART_Transmit(23);

		// USART_Transmit(measurement/256); //Encoder measurements are 2-bytes long and require 2 bytes to be sent. 
		// USART_Transmit(measurement%256);		
	}
}
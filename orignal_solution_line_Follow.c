/*
 * Team Id: eyrc#179
 * Author List: Tushar Garg, Harshal Wadhwa
 * Filename: Line_follow
 * Theme: Thirsty Crow
 * Functions:timer1_init_motor_count(), timer2_init_motor_speed() , adc_init() , interupt_init() , uart0_init() , initialise() , motor_Speed() , right_sixtydegree() , 
 *           left_sixtydegree() , node_to_node() , read_new() , set_forward(), set_reverse(), set_right(), set_left(), set_stop(), adc_pin_config(), break_stop(), callibrate(), 
 *           decide_rotation(), str_2_num(), correction(), arm_close(), arm_open()  
 * Global Variables: prev_error, flag, base_left, base_right, left_speed, right_speed, left_sensor, right_sensor,middle_sensor,path,robot_axis
 */

#define F_CPU 14745600
#define TE  (1<<5)
#define RE (1<<7)
#include<avr/io.h>
#include<util/delay.h>
#include<avr/interrupt.h>
#include<string.h>

#define magnet_on PORTH=PORTH|(1<<0)
#define magnet_off PORTH=PORTH&~(1<<0)
#define buzzer_on PORTA=PORTA|(1<<4)
#define buzzer_off PORTA=PORTA&~(1<<4)

int prev_error=0,flag=0,i=0,base_left=120,base_right=135,left_speed,right_speed,robot_axis;
unsigned int left_sensor=0,right_sensor=0,middle_sensor=0;
char path[100];


void timer1_init_motor_count(void);
void timer2_init_motor_Speed(void);
void adc_init(void);
void interupt_init(void);
void uart0_init(void);
unsigned int analogread(unsigned char);
void motor_init(void);
void uart_tx(char);
void initialise(void);
void motor_Speed(int,int);
void right_sixtydegree(int);
void left_sixtydegree(int);
void node_to_node(void);
void read_new(void);
void set_forward(void);
void set_stop(void);
void adc_pin_config(void);
void break_stop();
void callibrate(void);
int decide_rotation(char);
void start(void);
int str_2_num(char[]);
int correction(void);
void arm_close(void);
void arm_open(void);


/*Function Name - timer2_init_motor_Speed()
Input- None
Output - PWM wave output at OC2A and OC2B pins
Logic - It initializes the Timer 2 in Fast PWM mode by setting the WGM and COM bits in the corresponding register.PWM output is used to adjust the speed of the motors
Example call  - timer2_init_motor_Speed();
*/
void timer2_init_motor_Speed()              
{
	TCNT2=0 ;
	TCCR2A=TCCR2A|(1<<7);		//Non Inverting a
	TCCR2A=TCCR2A|(1<<5);		//Non Inverting b
	TCCR2A=TCCR2A|(3<<0);		//wgm0-1
	TCCR2B=TCCR2B|(1<<0);		//Prescaler=1
	DDRB=DDRB|(1<<4);           //PWM output at pin 4 of PORTB(left motor speed)
	DDRH=DDRH|(1<<6);           //PWM output at pin 6 of PORTB(right motor speed)
	
}

/*Function Name - timer1_init_motor_count()
Input-None
Output - None
Logic - It initializes the Timer 1 in external clock mode to read the counts from motor encoder
Example call  - timer1_init_motor_count();
*/
void timer1_init_motor_count()       
{
	TCCR1A=0x00;
	TCCR1C=0x00;
	TCNT1H=0;
	TCNT1L=0;
	TCCR1B=0x06;
	DDRD=DDRD&~(1<<6);             //Pin 6 of PORTD set as input to count motor pulses
}

/*Function Name - timer0_init_servo()
Input-None
Output - PWM Output at OC0A pin
Logic - It initializes the Timer 0 by setting respective bits in the corresponding register.It is used to control servo  angle
Example call  - timer1_init_clock();
*/
void timer0_init_servo()
{
	TCNT0=0 ;
	TCCR0A=TCCR0A|(1<<7);		//non inverting a
	TCCR0A=TCCR0A|(1<<5);		//non inverting b
	TCCR0A=TCCR0A|(3<<0);		//wgm0-1
	//TCCR0B=TCCR0B|(1<<3);		//wgm2
	TCCR0B=TCCR0B|(1<<2);		//Prescaler=1
	OCR0A=0;
	DDRB=DDRB|(1<<7);
}

/*Function Name - interrupt_init()
Input - None
Output - None
Logic - It initializes the interrupt for middle sensor by setting corresponding bits in registers
Example call  - interrupt_init();
*/
void interupt_init()
{
	cli();                        //Disabling global interrupt
	DDRD=DDRD&~(1<<0);            //Pin 0 set as input
	EICRA=0x02;
	EIMSK=0b00000001;             //setting external interrupt at PIN 0 (PORTD)
	//sei();
}

/*Function Name - adc_pin_config()
Input - None
Output - None
Logic - It sets PORTF pins for sensor input
Example call  - adc_pin_config();
*/
void adc_pin_config()
{
	DDRF = 0x00;       
	PORTF = 0x00;               
	DDRK = 0x00;
	PORTK = 0x00;
}

/*Function Name - adc_init()
Input - None
Output - None
Logic - It sets the bits in the corresponding register for ADC initialization
Example call  - adc_-init();
*/
void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;		//comparator off
	ADCSRA = 0x87;
}

/*Function Name - uart0_init()
Input - None
Output - None
Logic - It sets the bits in registers for USART initialization
*/
void uart0_init()
{
	UCSR0B = 0x00;							//disable while setting baud rate
	UCSR0A = 0x00;
	UCSR0C = 0x06;							//8 bit
	UBRR0L = 0x5F; 							//9600BPS at 14745600Hz
	UBRR0H = 0x00;
	UCSR0B = 0x98;
	UDR0=0x00;
	//UCSR0C  = 3<<1;							//setting 8-bit character and 1 stop bit
	//UCSR0B = RX | TX;
}

/*Function Name - analogread(unsigned char inputpin)
Input - Unsigned char inputpin
Output - Unsigned int sensor digital value
Logic - It requires the input pin from which sensor analog values will be obtained,then according to analog values it converts it into digital value for that specific
sensor connected at that inputpin.
Example call - analogread(1);
*/
unsigned int analogread(unsigned char inputpin)     //To read sensor values from given inputpin
{
	unsigned char a;
	int digital = 0;
	inputpin =inputpin & 0x07;
	ADMUX= 0x20| inputpin;
	ADCSRA=ADCSRA|(1<<ADSC);                         //start conversion
	while((ADCSRA&(1<<ADIF))==0);                    //waiting till conversion is not complete
	a=ADCH;
	if(a>=70)digital=1;                             //digitalising the sensor values
	ADCSRA=ADCSRA|(1<<ADIF);                        //clearing the ADC flag
	ADCSRB = 0x00;
	return digital;
}

/*Function Name - motor_init()
Input - None
Output - None
Logic - It initializes the DDRA register and PORTA pins for motor configuration
Example call - motor_init();
*/
void motor_init()
{
	DDRA=DDRA|(15<<0);                              
	PORTA=PORTA&0xf0;                               //setting the motor pins HIGH
}

/*Function Name - read_new()
Input - None
Output - None
Logic - It read the three sensors value  by  calling analogread function and stores the digital value of sensors in specific variables.
Example call - read_new();
*/
void read_new()
{
	left_sensor=analogread(0);
	right_sensor=analogread(1);
	middle_sensor=analogread(2);
}

/*Function Name - set_forward()
Input -  None
Output - None
Logic - It sets the motor port pins high and low in order to move the motors in forward direction
Example call - set_forward();
*/
void set_forward()
{
	PORTA=PORTA|(1<<0);
	PORTA=PORTA&~(1<<1);
	PORTA=PORTA|(1<<2);
	PORTA=PORTA&~(1<<3);
}

/*Function Name - set_stop()
Input - None
Output - None
Logic - It sets the motor port pins low in order to stop the motors.
Example call - set_stop();
*/
void set_stop()
{
	PORTA=PORTA&~(1<<0);
	PORTA=PORTA&~(1<<1);
	PORTA=PORTA&~(1<<2);
	PORTA=PORTA&~(1<<3);
}

/*Function Name - set_reverse()
Input - None
Output - None
Logic - It sets the motor port pins low and high in order to move the motors in reverse direction
Example call - set_reverse();
*/
void set_reverse()
{
	PORTA=PORTA&~(1<<0);
	PORTA=PORTA|(1<<1);
	PORTA=PORTA&~(1<<2);
	PORTA=PORTA|(1<<3);
}

/*Function Name - set_right()
Input - None
Output - None
Logic - It sets the left motor pins in forward direction and right motor pins in reverse direction in order to move in right direaction
Example call - set_right();
*/
void set_right()
{
	PORTA=PORTA&~(1<<0);
	PORTA=PORTA|(1<<1);
	PORTA=PORTA|(1<<2);
	PORTA=PORTA&~(1<<3);
}

/*Function Name - set_left()
Input -  None
Output - None
Logic - It sets the right motor pins in forward direction and left motor pins in reverse direction in order to move in left direction
Example call - set_left();
*/
void set_left()
{
	PORTA=PORTA|(1<<0);
	PORTA=PORTA&~(1<<1);
	PORTA=PORTA&~(1<<2);
	PORTA=PORTA|(1<<3);
}

/*Function Name - uart_tx()
Input -  char data
Output - None
Logic - It inputs the data to be transmit and initially it waits for the UDR register to be ready to transmit the new data and then set the UDR register
with new data
Example call - uart_tx('a');
*/
void uart_tx(char data)
{
	while(!(UCSR0A & TE));                       //waiting to transmit
	UDR0 = data;
}

/*Function Name - uart_rx()
Input -  None
Output - character (data which is received)
Logic - It initially waits for the data to be fully receive in UDR register and then return it
Example call - uart_rx();
*/
char uart_rx()
{
	while(!(UCSR0A & RE));                     //waiting to receive
	return UDR0;
}

/*Function Name - string_rx()
Input -  None
Output - None
Logic - It uses the uart_tx(transmit function) to acknowledge that bot is turned on and uart_rx(receive function) to receive the specific configuration path 
and stores it in the path variable
Example call - string_rx();
*/
void string_rx()
{
	while(1)
	{
		uart_tx('s');
		if(uart_rx() == 'a')break;
	}
 do
	{
		path[i] = uart_rx();
		i++;
	} while (path[i-1]!='\0');
}

/*Function Name - decide_rotation()
Input -  dest (It denotes 6 possible axis which the bot needs to align with)
Output - angle (This is the angle that the bot will move in order to get the required position)
Logic - With final alignment as input,this function return the value of angle which the bot has to rotate in either direction to obtain the required position 
Example call - decide_rotation('A');
*/
int decide_rotation(char dest)
{
	int dest_angle=0;
	int angle;

	switch(dest)
	{
		case 'A': dest_angle = 300;
		break;
		case 'B': dest_angle = 240;
		break;
		case 'C': dest_angle = 180;
		break;
		case 'E': dest_angle = 0;
		break;
		case 'F': dest_angle = 60;
		break;
		case 'G': dest_angle = 120;
		break;
	}

	if (dest_angle > robot_axis) angle =  dest_angle - robot_axis;
	else angle = 360 - (robot_axis - dest_angle);

	robot_axis = dest_angle;
	return angle;
}

/*Function Name - traverse()
Input -  char str[](path which is to be traversed)
Output - None
Logic - It inputs the path to be traversed that is obtained from the python script with the help of string_rx function and then instruct the bot to move according to
the value of the path with the cases of defined for the specific value of path received
Example call - traverse(path);
*/
void traverse(char str[])
{
	int angle;
	if (str[0] == '1')			//tells that from where the bot is started from start 1 or start 2 here 1 represent start 1
	robot_axis = 0;
	else if (str[0] == '2')     // 2 represent start 2
	robot_axis =180;

	int i=1;
	while(str[i]!='\0')          //string is called in loop till a null is detected
	{
		if((str[i]=='A')||(str[i] == 'B')||(str[i] == 'C')||(str[i] == 'E')||(str[i] == 'F')||(str[i] == 'G'))  //tells the bot about where it needs to be allinged these letters represents the six possible directions the bot has to turn
		{
			angle = decide_rotation(str[i]);
			if (angle > 180)
			left_sixtydegree(360-angle);
			else
			right_sixtydegree(angle);
			_delay_ms(100);
			node_to_node();
			_delay_ms(100);
		}
		else if(str[i]=='S')		//this is used to traverse the starting line from start to the first node
		{
			start();
			_delay_ms(100);
		}
		else if (str[i] >= '0' && str[i] <='9')		//before every pickup and drop a three digit number is received which tells about the angle the bot has to turn to pickup and drop 
		{
			
			char str_number[4];
			str_number[0] = str[i];
			str_number[1] = str[i+1];
			str_number[2] = str[i+2];
			str_number[3] = '\0';
			int angle = str_2_num(str_number);
			switch (angle)
			{
				case 60:right_sixtydegree(60);
				break;
				case 120:right_sixtydegree(120);
				break;
				case 180:right_sixtydegree(180);
				break;
				case 240:left_sixtydegree(120);
				break;
				case 300:left_sixtydegree(60);
				break;
				default:break;
			}
			i+=3;
			if(str[i]=='P')
			{
				set_forward();
				motor_Speed(135,145);
				_delay_ms(550);
				break_stop();
				arm_open();
				_delay_ms(100);
				magnet_on;
				uart_tx('p');
				_delay_ms(500);
				arm_close();
				_delay_ms(100);
				set_reverse();
				motor_Speed(135,145);
				_delay_ms(550);
				set_stop();
			}
			else if(str[i]=='D')
			{
				set_forward();
				motor_Speed(135,145);
				_delay_ms(550);
				break_stop();
				arm_open();
				_delay_ms(100);
				magnet_off;
				uart_tx('d');
				_delay_ms(500);
				arm_close();
				_delay_ms(100);
				set_reverse();
				motor_Speed(135,145);
				_delay_ms(550);
				set_stop();
			}
			switch (angle)
			{
				case 60:left_sixtydegree(60);
				break;
				case 120:left_sixtydegree(120);
				break;
				case 180:left_sixtydegree(180);
				break;
				case 240:right_sixtydegree(120);
				break;
				case 300:right_sixtydegree(60);
				break;
				default:break;
			}
			
		}
		i++;
	}
}

/*Function Name - str_2_num()
Input -  char string[](
Output - None
Logic - It sets the motor port pins high and low in order to move the motors in forward direction
Example call - set_forward();
*/
int str_2_num(char string[])
{
	int ans = 0;
	int len = strlen(string);
	int j = 0;
	int weight = 1;
	for (int k = 0; k < len-1; ++ k) weight *= 10;
	do{
		ans += (string[j] - '0') * weight;
		j++;
		weight /= 10;
	}while (string[j] != '\0');
	return ans;
}

/*Function Name - initilaise()
Input -  None
Output - None
Logic - It initializes the bot by calling all the init functions
Example call - initilaize();
*/
void initialise() 
{
	timer1_init_motor_count();
	timer2_init_motor_Speed();
	timer0_init_servo();
	uart0_init();
	motor_init();
	adc_init();
	interupt_init();
	adc_pin_config();
	DDRA=DDRA|(1<<4);		//Pin 4 of PORTA set as output for buzzer
	magnet_off;             //initially buzzer is set LOW
	DDRH=DDRH|(1<<0);       //Pin 0 of PORTH is set as output for electro-magnet
}

/*Function Name - motor_Speed()
Input -  left_motor(speed),right_motor(speed)
Output - None
Logic - It sets the motors speed as given in parameters by setting the respective values in OCR2A and OCR2B registers
Example call - motor_Speed(135,145);
*/
void motor_Speed(int left_motor,int right_motor)
{
	OCR2A=right_motor;
	OCR2B=left_motor;
}

/*Function Name - right_sixtydegree()
Input -  int ang(angle to be rotated)
Output - None
Logic - It inputs the angle to be rotated in right direction by specifying the counts of the motor according to input angle and after turning it is stopped.
Example call - right_sixtydegree(60);
*/
void right_sixtydegree(int ang)
{
	TCNT1=0;
	set_right();
	motor_Speed(190,245);
	int counter=0;
	switch(ang)
	{
		case 60:counter=230;
		break;
		case 120:counter=600;
		break;
		case 180:counter=910;
		break;
		default:counter=0;
	}
	while(TCNT1<=counter);                   //Motors are moving according to counter values which are set according to given angle
	set_stop();                              //motors are stop
}

/*Function Name - left_sixtydegree()
Input -  int ang (angle to be rotated)
Output - None
Logic - This function is used to turn the bot x (60,120) degree in left direction  by setting left motor in reverse direction and right motor in forward direction and then counting the pulses according to the angle
Example call - left_sixtydegree();
*/
void left_sixtydegree(int ang)
{
	TCNT1=0;
	set_left();
	motor_Speed(190,245);
	int counter=0;
	switch(ang)
	{
		case 60:counter=280;
		break;
		case 120:counter=500;
		break;
		default:counter=0;
	}
	while(TCNT1<=counter);                   //Motors are moving according to counter values which are set according to given angle
	set_stop();                              //motors are stop
}

/*Function Name - node_node()
Input -  None
Output - None
Logic - This function is used to cover the straight distance between two nodes and is the bot deviates from its path then interupt is called and the flag is set to 1 and after reaching the other node since the flag is 1 the function is terminated after stoping the bot
Example call - node_node();
*/
void node_to_node()
{
	sei();
	flag=0;					//135146
	TCNT1=0;
	motor_Speed(135,146);
	while(TCNT1<=4200)
	{
		if(flag==1)                      //flag is set when interrupt occurs
		{
			break;
		}
		set_forward();                   //Motors will move forward till the 4400 counts from motor is obtained
	}
	break_stop();
	cli();	//Disabling the external interrupt
}

/*Function Name - start()
Input -  None
Output - None
Logic - This function is used to cover the distance of start point and the first node
Example call - start();
*/
void start()
{
	TCNT1=0;
	while(TCNT1<=360)                     //360 counts are obtained
	{
		motor_Speed(190,245);
		set_forward();                     //Motors will move forward till the 360 counts from motor is obtained
	}
	set_stop();
}

/*Function Name - break_stop()
Input -  None
Output - None
Logic - This function instantly stops the bot by slightly moving it in reverse direction and then stoping it
Example call - break_stop();
*/
void break_stop()
{
	set_reverse();
	motor_Speed(base_left,base_right);
	_delay_ms(50);
	motor_Speed(0,0);
	set_stop();
}

/*Function Name - callibrate()
Input -  None
Output - None
Logic - This function is used to callibrate the sensor manually, it continously sends the value of the sensor via Xbee to the computer where the vslues are displayed and accordingly callibraton is done with the help of potentiometers on the bot
Example call - callibrate();
*/
void callibrate()
{
	while(1)
	{
		read_new();
		uart_tx(left_sensor);
		uart_tx(middle_sensor);
		uart_tx(right_sensor);
		_delay_ms(1000);
	}
}

/*Function Name - ISR()
Input -  INT0_vect
Output - None
Logic - An external interupt is connected to the middle sensor of the bot, always the middle sensor should be on the line and the sensor gives a high signal whenever the bot is diverged from the path the middle sensor moves from black line to white line and a high to low pulse is generated which makes a call to the interupt and interupt makes changes to the speed of the bot with the help of correction() function to rectify the deviation
Example call - NONE
*/
ISR(INT0_vect)
{
	flag=1;
	int local=0;
	set_forward();
	while(local!=1)
	{
		local=correction();
	}
}

/*Function Name - correction()
Input -  None
Output - None
Logic - This function calculates the error of the bot that is how much the bot is deviated from its path and the with the help of constant variables calculates a correction speed that is then passed to the motor_speed() function
Example call - correction();
*/
int correction()
{
	float kp = 0.22 , kd= 1.5 ;   //.09 .1                                                                            //setting the values for constants Kp and Kd
	int sum,error;
	read_new();                                                                                         //reading new sensor values
	sum= right_sensor + middle_sensor + left_sensor;                                                    //calculating the sum of sensor values
	if(sum!=0)                                                                                          //checking if sum is not equal to zero (all sensors on white)
	{
		set_forward();
		int correct,line_position = ( (right_sensor*100) + (middle_sensor*200) + (left_sensor*300) ) / sum ;  //calculating the position of sensor
		error = 200 - line_position ;                                                                         //finding the error relative to ideal position(=200)
		if ( right_sensor== 1 && left_sensor == 1 && middle_sensor == 1)                                     //if all sensors are on black line(node is reached)
		{
			break_stop() ;
			prev_error=0;                                                                                    //clearing previous error
			return 1 ;
		}
		correct = kp * error + kd * ( error - prev_error ) ;                                                  //finding the correction in speed
		left_speed = base_left + correct;                                                                     //adjusting the speed of left motor
		right_speed = base_right - (correct) ;                                                                  //adjusting the speed of right motor
		prev_error=error;
	}
	else
	if(sum == 0)                         //if sum is zero (all sensors on white)
	{
		set_reverse();                   //reversing the motors for this over-run condition
		motor_Speed(125,135);
		_delay_ms(225);
	}
	
	if(right_speed>180)right_speed=180;    //checking for maximum right speed
	if(left_speed>165)left_speed=165;      //checking for maximum left speed
	
	if(left_speed<110) left_speed=110;     //checking for minimum left speed
	if(right_speed<120)right_speed=120;    //checking for minimum right speed
	
	motor_Speed(left_speed,right_speed);   //setting the adjusted speed values
	return 0;
}

/*Function Name - arm_close()
Input -  None
Output - None
Logic - It sets the arm in closed position by rotating servo for a fixed time
Example call - arm_close();
*/
void arm_close()
{
	for(int i=90;i<=100;i++)
	{
		OCR0A=i;
		_delay_ms(15);
	}
	_delay_ms(650);
	OCR0A=0;
}

/*Function Name - arm_open()
Input -  None
Output - None
Logic - It sets the arm in extended position by rotating servo for a fixed time
Example call - arm_open();
*/
void arm_open()
{
	OCR0A=60;
	_delay_ms(800);
	OCR0A=0;
}

int main(void)
{
	initialise();
	string_rx();
	traverse(path);
	buzzer_on;
	_delay_ms(5000);
	buzzer_off;
	while(1);
}
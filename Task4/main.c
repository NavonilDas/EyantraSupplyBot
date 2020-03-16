// Define CPU Speed which is 16MHz
#define F_CPU 16000000UL

#include <avr/io.h>

// for _delay_ms function
#include <util/delay.h>

// For interrupts function and sub routine
#include <avr/interrupt.h>

// XBEE Baud Rate is 9600
// BAUD RATE CALCULATION = (16MHZ / 16 / 9600) -1
#define BAUDRATE 103

/// Queue -> Circular Queue

/************************************************************************/
/* Receiver Queue structure */
#define RX_BUFFER_SIZE 128  // Receiver Queue Size
static unsigned char RX_BUFFER[RX_BUFFER_SIZE];
static volatile unsigned char RX_HEAD;
static volatile unsigned char RX_TAIL;
/************************************************************************/

/************************************************************************/
/* Transmitting Queue structure */
// Transmitting Queue Size
#define TX_BUFFER_SIZE 128
static unsigned char TX_BUFFER[TX_BUFFER_SIZE];
static volatile unsigned char TX_HEAD;
static volatile unsigned char TX_TAIL;

// For PWM to control the speed of motor
volatile double dutyCycle = 70;

// This Function enables all the Data register to output mode
void set_output_enable(void){
	// Buzzer and L298n
	DDRD |= (1 << DDD2) | (1 << DDD7) | (1 << DDD6) | (1 << DDD5) | (1 << DDD4);
	// PWM	
	DDRB |= (1 << DDB1) | (1 << DDB2) | (1 << DDB3);
	// Striking mechanism
	DDRB |= (1 << DDB0);
	
	// LED
	DDRB |= (1 << DDB5);
	
}

/* This function is used to beep the buzzer twice                       */
void beep_twice(void){
	// Pin 2 Low for 301 mil sec then high 301 mil sec
	PORTD &= ~(1 << PORTD2);
	_delay_ms(301);
	PORTD |= (1 << PORTD2);
	_delay_ms(301);
	
	// Pin 2 Low for 301 mil sec then high for rest of the period
	PORTD &= ~(1 << PORTD2);
	_delay_ms(301);
	PORTD |= (1 << PORTD2);
}

// function to beep buzzer for five second
void beep_five(void){
	// Pin 2 Low for 5 sec then high for rest of the period
	PORTD &= ~(1 << PORTD2);
	_delay_ms(5000);
	PORTD |= (1 << PORTD2);
}

// function to stop motor
void stop_moving(){
	// LOW all pins on the motor driver
	PORTD &= ~((1 << PORTD7) | (1 << PORTD6) | (1 << PORTD5) | (1 << PORTD4) );
}

// move forward the bot
void move_forward(void){
	stop_moving();
	// High pin IN1 and IN3 in L298 Motor driver
	PORTD |= (1 << PORTD7) | (1 << PORTD5);
}

// move backward the bot ; for future purpose :)
void move_backward(void){
	stop_moving();
	// High pin IN2 and IN4 in L298 Motor driver
	PORTD |= (1 << PORTD6) | (1 << PORTD4);
}

// move left the bot
void move_left(void){
	stop_moving();
	// High pin IN1 in L298 Motor driver
	PORTD |= (1 << PORTD7);
}

// move right the bot
void move_right(void){
	stop_moving();	
	// High pin IN3 in L298 Motor driver
	PORTD |= (1 << PORTD5);
}

// enable timer 1
void set_timer(void){
	// Using a 16 Bit Timer
	TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10) | (1 << WGM11) | (1<<WGM12);
	
	// Timer 1 Overflow Interrupt Enable
	TIMSK1 |= (1 << TOIE1);
		
	// Set Duty Cycle on Output Compare Register 1 A
	OCR1A = (dutyCycle / 100) * 1023;
	// Set Duty Cycle on Output Compare Register 1 B
	OCR1B = (dutyCycle / 100) * 1023;
	
}

// Interrupt Service routine if Timer 1 overflows
ISR(TIMER1_OVF_vect){
	// Calculate the value of Output Compare Register 1 A
	OCR1A = (dutyCycle / 100) * 1023;
	// Calculate the value of Output Compare Register 2 A
	OCR1B = (dutyCycle / 100) * 1023;	
}

// Just a Testing code
void blink_led(void){
	PORTB ^= (1 << PORTB5);
	_delay_ms(1000);	
}

// Initialize analog reading
void analog_init(void){
	// AVCC with external capacitor at AREF pin
	ADMUX |= (1 << REFS0);
	/*
		Enable Analog to Digital Conversion
		Set the Prescaler to 128
	*/
	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
}

// function to read value from analog pin
uint16_t analog_read(unsigned char pin){
	// Select the pin in ADC multiplexer
	ADMUX = (ADMUX & 0xF8) | pin;
	ADCSRA |= (1<<ADSC);
	// wait till value is converted
	while(ADCSRA & (1<<ADSC));
	// return the value
	return (ADC);
}

// initialize serial with given baud rate
void serial_init(){
	// Set baud rate on USART baud rate register
	UBRR0H = (unsigned char)(BAUDRATE >> 8);
	UBRR0L = (unsigned char)BAUDRATE;
	//Enable receiver and transmitter
	UCSR0B = ((1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0));
	// Set frame format: 8data and no stop bits
	UCSR0C =  (1 << UCSZ01) | (1 << UCSZ00);
	
	// set the head and tail of the queue to zero
	RX_HEAD = RX_TAIL = 0;
	TX_HEAD = TX_TAIL = 0;
}

// function transmits a character
void serial_transmit(unsigned char ch){
	unsigned char Thead; // Temp head
	Thead = (TX_HEAD + 1) & (TX_BUFFER_SIZE - 1); // increment the head
	// Wait for the free space
	while(Thead == TX_TAIL);
	TX_BUFFER[Thead] = ch; // Store Data in the CQueue
	TX_HEAD = Thead; // Update the Head
	UCSR0B |= (1 << UDRIE0); // Set Bit for the interrupt
}

// function returns the serial value
unsigned char serial_recieve(void){
	unsigned char Ttail; // Temporary Tail
	// Wait till the queue is not empty
	while(RX_HEAD == RX_TAIL); // Wait for some data
	// Increment the tail and update
	Ttail = (RX_TAIL + 1) & (RX_BUFFER_SIZE - 1);
	RX_TAIL = Ttail;
	// return the value received
	return RX_BUFFER[Ttail];
}

// function to serial print an integer
// this function is for testing purpose for the value of white line sensor
void serial_print(int a){
	// Create a buffer of size 10
	unsigned char buff[10];
	unsigned char ind = 0;
	// store each digit inside the buffer as a character
	while(a > 0){
		buff[ind] = (a%10) + 48;// 48 -> ASCII value of 0
		a = a/10;
		++ind;
	}
	// transmit characters in reverse order
	while (ind)
	{
		--ind;
		serial_transmit(buff[ind]);
	}
}

// function to strike and beep after striking
void strike(void){
	// Enable striker
	PORTB &= ~(1 << PORTB0);
	_delay_ms(500);
	PORTB |= (1 << PORTB0);
	// After striking beep buzzer once
	PORTD &= ~(1 << PORTD2);
	_delay_ms(401);
	PORTD |= (1 << PORTD2);
}

// Interrupt service routine if data is received
ISR(USART_RX_vect){
	// temporary variables to store received data and head
	unsigned char data,tmphead;
	// get the data from USART Data register
	data = UDR0;
	// increment the temporary head
	tmphead = (RX_HEAD + 1) & (RX_BUFFER_SIZE - 1);
	// save the data in queue and change the head
	RX_BUFFER[tmphead] = data;
	RX_HEAD = tmphead;
	
}

// Interrupt service routine for data to send
ISR(USART_UDRE_vect){
	// temporary variables to store tail and data
	unsigned char Ttail,data;
	// check if queue is not empty
	if(TX_HEAD != TX_TAIL){
		// increment the tail
		Ttail = (TX_TAIL + 1) & (TX_BUFFER_SIZE - 1);
		TX_TAIL = Ttail;
		// get the data from queue
		data = TX_BUFFER[Ttail];
		// set data inside the USART data register
		UDR0 = data;
		}else{
			// if queue is empty then set the interrupt flag to low
			UCSR0B &= ~(1 << UDRIE0);
		}
}

int main(void)
{
	// Enable data register
	set_output_enable();
	
	// Set the default value for buzzer to high and striking mechanism to high
	PORTD |= (1 << PORTD2);
	PORTB |= (1 << PORTB0);
	
	// Enable timers
	set_timer();
	// Enable serial communication
	serial_init();
	// Enable Analog reading
	analog_init();
	
	sei(); // Enable Interrupt
	TCCR1B |= (1 << CS10); // Set Timer 1 prescaler to 1
	
	// variable to store the value of white line sensor
	int first=0,last=0,middle=0;
	// ch stores the received value
	// start -> flag for bot to move in white line or not
	unsigned char ch,start = 0;
	while (1) 
    {
		// Read the value from sensor
		first = analog_read(1);
		middle = analog_read(2);
		last = analog_read(3);
		
		// Check if Receiver queue is not empty or serial data is available
		if(RX_TAIL != RX_HEAD){
			// Read the value
			ch = serial_recieve();
			/*
				value of ch
				s -> Start the bot
				h -> start the striking mechanism
				e -> end of run
			*/
			if(ch == 's'){
				start = 1;
				serial_transmit('d'); // Return a response
			}
			else if(ch == 'h'){
				stop_moving();
				beep_twice(); // Before servicing beep twice
				strike();
				serial_transmit('d'); // Return a response
			}else if(ch == 'e'){
				stop_moving();
				beep_five(); // beep for continuous 5 second
				serial_transmit('d'); // Return a response
				start = 0;
			}
		}
		
		if(start){
			// if middle and last sensor are in black move left
			if(middle > 100 && last > 100){
				move_left();
			}// if middle and first are in black move right
			else if(middle > 100 && first > 100){
				move_right();
			}else{
				// otherwise move forward
				move_forward();
			}
		}
    }
}

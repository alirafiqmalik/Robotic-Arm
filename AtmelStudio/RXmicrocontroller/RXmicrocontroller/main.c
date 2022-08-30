#define F_CPU 16000000L //CPU Clock Freq
#define BAUD 57600 //Baudrate in bps
#define BRC ((F_CPU/16/BAUD)-1) //Baud Rate Register Value

#define SERVO_PORT  PORTB
#define SERVO_DDR   DDRB

#define N_SERVOS    8

#define SERVO_MIN    0
#define SERVO_MAX   180
#define SERVO_MID   (SERVO_MIN + SERVO_MAX) / 2

#define SERVO_FRAME 20000 // microseconds (50Hz)
#define SERVO_TIME_DIV (SERVO_FRAME / N_SERVOS)
#define US2TIMER1(us) ((us) * (uint16_t)(F_CPU / 1E6))
#define NumberofMotors 8


#include<avr\io.h>
#include<avr\interrupt.h>
#include <stdint.h>
#include <stdbool.h>
#include <util/delay.h>


volatile uint8_t servoTicks[N_SERVOS];

uint8_t motoranglesvalues[N_SERVOS];
int idC=0;

const static uint8_t servoOutMask[N_SERVOS] = {
	0b00000001, // PX0
	0b00000010, // PX1
	0b00000100, // PX2
	0b00001000, // PX3
	0b00010000, // PX4
	0b00100000, // PX5
	0b01000000, // PX6
	0b10000000, // PX7
};




volatile uint8_t map(int x, float xmin,float xmax,float ymin,float ymax);
void setupRX();
void initServo();
void writeangle(uint8_t servo, uint8_t angle );
void writeus(uint8_t servo, uint8_t angle );
void writepotval(uint8_t servo, uint8_t angle );
void setmotors();
void setmotorspot();






//Declaration of USART Reciever
void setupRX()
{
	//Baud Rate Register Setup
	UBRR0H=(BRC>>8);
	UBRR0L=BRC;
	// Enable Recieve and Recieve Complete Interrupt
	UCSR0B=(1<<RXEN0)|(1<<RXCIE0);
	// Prescaler to 128 and Even Parity Enable
	UCSR0C=(1<<UCSZ01)|(1<<UCSZ00)|(1<<UPM01);

}




volatile uint8_t map(int x, float xmin,float xmax,float ymin,float ymax) {

	return (((x - xmin) * (ymax - ymin))/ (xmax - xmin)) + ymin;
	
}



void initServo()
{
	// Outputs
	SERVO_DDR |= 0xff;
	// Setup a first compare match
	OCR1A = TCNT1 + US2TIMER1(100);
	// start timer 1 with no prescaler
	TCCR1B = (1 << CS10);
	// Enable interrupt
	TIMSK1 |= (1 << OCIE1A);
}

void writeangle(uint8_t servo, uint8_t angle )
{
	cli();
	servoTicks[servo] = map(angle,0,180,16000,32000);
	sei();
}

void writeus(uint8_t servo, uint8_t timems )
{
	cli();
	servoTicks[servo] = map(timems,1000,2000,16000,32000);
	sei();
}


void writepotval(uint8_t servo, uint8_t angle )
{
	cli();
	servoTicks[servo] = map(angle,0,255,16000,32000);
	sei();
}


void setmotors(){
	for(int i=0;i<sizeof(motoranglesvalues);i++){
		writeangle(i, motoranglesvalues[i]);
		_delay_ms(10);
	}
}

void setmotorspot(){
	for(int i=0;i<NumberofMotors;i++){
		writepotval(i, motoranglesvalues[i]);
		_delay_ms(10);
	}
}




int main(void)
{
	
//for(int i=0;i<NumberofMotors;i++){
	//motoranglesvalues[i]=0;
	//}	
	//setup Reciever
	setupRX();		
	initServo();
	sei();
	
	while(1) {
	setmotorspot();

		}
		
	}




ISR(TIMER1_COMPA_vect)
	{
		static uint16_t nextStart;
		static uint8_t servo;
		static int outputHigh = 1;
		uint16_t currentTime = OCR1A;
		uint8_t mask = servoOutMask[servo];
		
		if (outputHigh) {
			SERVO_PORT |= mask;
			// Set the end time for the servo pulse
			OCR1A = currentTime + servoTicks[servo];
			nextStart = currentTime +US2TIMER1(SERVO_TIME_DIV);
			} else {
			SERVO_PORT &= ~mask;
			if (++servo == N_SERVOS) {
				servo = 0;
			}
			OCR1A = nextStart;
		}
		outputHigh = !outputHigh;
	}


ISR(USART_RX_vect)
{
	motoranglesvalues[idC]=UDR0;
	if(idC>=(NumberofMotors-1))
	{
		idC=0;
	}
	else
	{
		idC++;
	}

}

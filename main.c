// ATMEGA328P Datasheet: https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
 
/* Pins:
*      Port D      - Output
*      Port D, 1-2 - Motor 1 Control
*      Port D, 3-4 - Motor 2 Control
*      Port C, 0   - IR Sensor Front
*      Port C, 1   - Potentiometer
*      Port C, 0   - IR Sensor Back
*      Port B, 1   - PWM For Motors
*/
 
#include <xc.h>
#include <avr/io.h>
#include <avr/interrupt.h>
 
#define SPEED_PIN 1
#define M1A 1
#define M1B 2
#define M2A 3
#define M2B 4
#define POT 1
#define IR1 0
#define IR2 2
 
void setup_ADC(void);
void setup_PWM(void);
void setup_timer(void);
unsigned int get_ADC(unsigned char);
void change_bit(volatile uint8_t *, unsigned char, unsigned char);
void change_direction(unsigned int duration, unsigned char direction, unsigned char synchronized);

unsigned int threshold = 0, ir1 = 0, ir2 = 0, delay = 0, target_delay = 0, in_reverse = 0;
 
int main(void) {
	DDRD |= (1 << M1A) | (1 << M1B) | (1 << M2A) | (1 << M2B);
	DDRB |= (1 << SPEED_PIN);
 
	setup_ADC();
	setup_PWM();
	setup_timer();
 
	OCR1A = 128; // Sets PWM to 50% duty cycle
	change_direction(0, 0b10, 2);   // continue going straight
 
	// Occurs from now till infinity
	while(1) {
		threshold = get_ADC(POT);                                            // gets ir1 of pot threshold
		ir1 = get_ADC(IR1);                                               // Get the result of the IR ADC
		ir2 = get_ADC(IR2);                                               // Get the result of the IR ADC
        
		if(delay > target_delay && ir1 < threshold && ir2 < threshold) 
        {
			if(in_reverse) {
                change_direction(0, 0b11, 1);
                change_direction(250, 0b10, 0);
                in_reverse = 0;
            } else 
                change_direction(0, 0b10, 2);   // continue going straight
			
            target_delay = 0;
			delay = 0;
		} 
        else if(ir1 >= threshold | ir2 >= threshold) // Runs only if we aren't delaying
        { 			
            change_direction(1000, 0b01, 2);
            in_reverse = 1;
        }
	}
}
 
// Sets up ADC
void setup_ADC(void)
{
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); // Enable ADC | Set Prescaler Bits (0-2) to 111 (128 ticks of system clock is 1 tick for the ADC (i think))
	ADMUX = (1<<REFS0);                                  // Set voltage reference to AVcc with external cap at AREF pin
}
 
// Sets up timer and interrupt
void setup_timer(void) {
	TCCR0A |= (1 << COM0A1); // Set timer mode to clear on compare
	TCCR0B |= (1 << CS02) | (1 << CS00); // Set timer prescalar to 1024
	OCR0A = 10;    // Set compare value to 97 ticks (976 ticks in 1 second with the prescalar set to 1024) as clock is at ~ 1MHz
	TIMSK0 = (1 << OCIE0A);  // Enable interupt
	sei();                   // Enable global interrupt
}
 
ISR(TIMER0_COMPA_vect) {
	if(target_delay) delay++;
}
 
// Sets up PWM for Port B1
void setup_PWM(void) {
	TCCR1B  |= (1 << CS12); // Sets Prescaler for timer 1 to 256
	TCCR1A  = (1 << COM1A1) & ~(1 << COM1A0); // Defines that when a match occurrs to update the PWM when the timer goes to 0 again
	TCCR1A |= (1 << WGM12) | (1 << WGM10); // Turns on fast PWM mode
}
 
// Gets the ADC value from analog pin channel
unsigned int get_ADC(unsigned char channel)
{
	ADMUX = (ADMUX & 0xF8)|channel;  // Do ADC on channel pin
	change_bit(&ADCSRA, ADSC, 1);    // Start ADC conversion
	while(ADCSRA & (1<<ADSC));       // While ADC is still in progress, do nothing
	return ADC;                      // Return the ADC value
}
 
// Is used to start the ADC conversion -> make parameters a reference to (&) ADCSRA, ADSC, and 1, respectively
void change_bit(volatile uint8_t *SFR, unsigned char my_bit, unsigned char bit_val)
{
	if(bit_val == 0) {
		*SFR &= ~(1 << my_bit); // If my_bit is ADSC then it will stop ADC conversion
		} else {
		*SFR |= (1 << my_bit); // If my_bit is ADSC then it will start ADC conversion
	}
}
 
/* Changes the direction of the robot
*  Parameters:
*     duration     - how long to turn for in ms
*     direction    - determines motor operation given by motorcontroller truth table
*     synchronized - if 0 targets left motor, if 1 targets right motor, if 2 targets both
*/
void change_direction(unsigned int duration, unsigned char direction, unsigned char synchronized) {
	if(synchronized == 2) {
		PORTD &= ~((~direction << M1A) | ~(direction << M2A)); // AND it to get the 0s in place
		PORTD |= (direction << M1A) | (~direction << M2A);     // OR it to get the 1s in place
		} else {
		PORTD &= synchronized ? ~(~direction << M1A) : ~(~direction << M2A); // Writes the 1s in the direction
		PORTD |= synchronized ? (direction << M1A) : (direction << M2A); // Writes the 0s in the direction
	}

	target_delay = duration/333; // I divide by 333 so that the 1 duration is 1 ms
	TCNT0 = 0;
    delay = 0;
}

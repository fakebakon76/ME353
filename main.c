// ATMEGA328P Datasheet: https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
 
/* Pins:
*      Port D      - Output
*      Port D, 0   - Motor 1 Enable
*      Port D, 1-2 - Motor 1 Control
*      Port D, 3   - Motor 2 Enable
*      Port D, 4-5 - Motor 2 Control
*      Port C, 0   - IR Sensor
*      Port C, 1   - Potentiometer
*      Port B, 1   - PWM For Motor
*/
 
#include <xc.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
 
#define SPEED_PIN 1
#define M1E 0
#define M1A 1
#define M1B 2
#define M2E 3
#define M2A 4
#define M2B 5
 
void setup_ADC(void);
unsigned int get_ADC(unsigned char);
void change_bit(volatile uint8_t *, unsigned char, unsigned char);
void change_direction(unsigned int duration, unsigned char direction, unsigned char synchronized);
 
unsigned int threshold = 0;
unsigned int result = 0;
 
int main(void) {   
    DDRD |= (1 << M1A) | (1 << M1B) | (1 << M2A) | (1 << M2B);
    DDRB |= (1 << SPEED_PIN);

    setup_ADC();
    setup_PWM();

    OCR1A = 128; // Sets PWM to 50% duty cycle

    // Occurs from now till infinity
    while(1) {
        threshold = get_ADC(1);                                            // gets result of pot threshold
		result = get_ADC(0);                                               // Get the result of the IR ADC
		PORTD = (result >= threshold) ? (PORTD | 0x01) : (PORTD & ~0x01);  // Outputs to port d bit 1
    }
}
 
// Sets up ADC
void setup_ADC(void)
{
    ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); // Enable ADC | Set Prescaler Bits (0-2) to 111 (128 ticks of system clock is 1 tick for the ADC (i think))
    ADMUX = (1<<REFS0);                                  // Set voltage reference to AVcc with external cap at AREF pin
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
        PORTD = ~((~direction << M1A) | (~direction << M2A)); // AND it to get the 0s in place
        PORTD |= (direction << M1A) | (direction << M2A);     // OR it to get the 1s in place
        PORTD |= (1 << M1E) | (1 << M2E); // Enable
    } else {
        PORTD &= synchronized ? ~(direction << M1A) : ~(direction << M2A); // Writes the 1s in the direction
        PORTD |= synchronized ? (direction << M1A) : (direction << M2A); // Writes the 0s in the direction
		PORTD |= synchronized ? (1 << M1E) : (1 << M2E); // Enables the turning
    }

	for(int i = 0; i < duration; i++) _delay_ms(1); 
    PORTD &= (~(1 << M1E) & ~(1 << M2E)); // Turns off the enable
}


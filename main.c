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
 
 
int main(void)
{
    DDRD |= (1 << M1A) | (1 << M1B) | (1 << M2A) | (1 << M2B);
    DDRB |= (1 << SPEED_PIN);  // Sets motor speed pin to be output
 
    while(1) {
    }
}
 
/* Changes the direction of the robot
* Parameters:
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

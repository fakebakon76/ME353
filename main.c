// ATMEGA328P Datasheet: https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
 
/* Pins:
*      Port D    - Output
*      Port D, 1 - Passed threshold indicator
*      Port C, 0 - IR Sensor
*      Port C, 1 - Potentiometer
*/
 
#include <xc.h>
#include <avr/io.h>
#include <avr/interrupt.h>
 
void setup_ADC(void);
unsigned int get_ADC(unsigned char);
void change_bit(volatile uint8_t *, unsigned char, unsigned char);
 
unsigned int threshold = 0;
unsigned int result = 0;
 
int main(void)
{   
    DDRD = 0b11111111;     // Sets all bits in Port D to be outputs
    setup_ADC();    
    // What happens from now till infinity
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

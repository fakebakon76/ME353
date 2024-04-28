// ATMEGA328P Datasheet: https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf

#include <xc.h>
#include <avr/io.h>
#include <avr/interrupt.h>

void setup_ADC(void);
unsigned int get_ADC(unsigned char);
void change_bit(volatile uint8_t *, unsigned char, unsigned char);

int main(void)
{
    DDRD = 0b11111111;
    TCCR0 |= (1 << CS02);  // Select 1024 clock cycles for the interupt
    TIMSK |= (1 << TOIE0); // Enables the timer interupt 
    sei();                 // Enables general interupt
    TCNT0 = 0;             // Sets the timer counter to 0 (i think)

    setup_ADC();

    // Do nothing and let interupts determine when and what to do
    while(1) { 
    }
}

// Interupt service routine for timer 0 
ISR(TIMER0_OVF_vect) 
{
    unsigned int result;
    result = get_ADC(0); // Get the result of the ADC
    PORTD = result >> 2; // Output the result (bit shifted by 2 to get rid of the 2 least significant bits) to Port D
    TCNT0 = 0;           // Reset the timer count to 0
}

void setup_ADC(void)
{
    ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); // Enable ADC | Set Prescaler Bits (0-2) to 111 (128 ticks of system clock is 1 tick for the ADC (i think))
    ADMUX = (1<<REFS0);                                  // Set voltage reference to AVcc with external cap at AREF pin
    ADMUX = (ADMUX & 0xF8);                              // Choose pin ADC0 to do conversion on
}

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

/*
 *  LDR.c
 * By Austen Fogarty 1231547
 * Shows the output of a comparitor on the
 * LCD which is comparing the LDR voltage
 * against a reference voltage
 *
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include "lcd.h"
#include <util/delay.h>
#include <stdio.h>
#include <string.h>

#define debounce_time  50

static volatile int count;
static timer_count;	// Debounce time delay in hundredths on milliseconds
static debounce_count;

static volatile int square1;	// Count of PHA
static volatile int square2;	// Count of PHB

static uint8_t previous_pin;
static uint8_t pin;

 // Initialise the timer and pin change interrupts
 void Initialise_Interrupts()
 {
			/* PHA on Port B pin 0 --- route via PCINT0 */
			/* PHB on Port B pin 1 --- route via PCINT1 */
	 	 	/* LDR Comparator on Port B pin 4 --- route via PCINT4 */
	        PCMSK0 |= (1 << PCINT0) | (1 << PCINT1) | (1 << PCINT4) ;
	        PCICR  |= (1 << PCIE0);

	        /* Initialise timer0 interrupts */
	        TCCR0A |= (1 << WGM01);					// Set the timer to CTC mode
	        OCR0A = 155;							// Value to count to 155
	        TCCR0B |= (1 << CS02) | (1 << CS00) ;	// 1024 pre-scaler
	        TIMSK0 |= (1 << OCIE0A);				// Enable interrupt flag when timer is matched with OCR0A

	        /* Globally enable interrupts */
	        sei();
 }

int main(void)
{
	Initialise_Interrupts();	//Initialise interrupts
	/* Set up Port B; input 1 on Bit 0 */
	DDRB = 0;
	/* No pull-up resistor on inputs */
	PORTB = 0x00;
	lcd_init('c');		// Initialise the LCD
	char str[20];

	while (1)
	{
		//Concatenate to an array
		snprintf(str,20,"%i  %i\n%i",square1,square2,count);
		lcd_display(str,0);		// Display the time
		_delay_ms(50);			// Short delay
    	lcd_clear();			//Clear the LCD
	}
	return 0;
}

// Timer interrupt
ISR(TIMER0_COMPA_vect)
{
	if (timer_count == debounce_time) // Set number of timer interrupts until count can be updated again
	{
		debounce_count = 0;
		timer_count = 0;
	}
	else if (debounce_count > 0)
	{
		timer_count++;
	}
}

// Pin change interrupt
ISR(PCINT0_vect)
{
	pin = PINB;		// Save the pin values on PORTB
	// If the interrupt is on the rising edge of PHA
	if((pin & (1<<PB0)) && !(previous_pin & (1<<PB0)))
	{
		square1++;
	}
	// If the interrupt is on the rising edge of PHB
	if((pin & (1<<PB1)) && !(previous_pin & (1<<PB1)))
	{
		square2++;
	}

	previous_pin = pin;	// Save current pin state for the next interrupt

 // LDR sense
	if (!(pin & (1 << PB4)))
	{
		if (debounce_count == 0)
			{
				count++;
				debounce_count++;
			}
	}
}





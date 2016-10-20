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
#include <stdlib.h>
#include <string.h>

#define debounce_time  50

static volatile int count;
static int timer_count;	// Debounce time delay in hundredths on milliseconds
static int debounce_count;

static uint8_t previous_pin;
static uint8_t pin;

static float time_count = 0;
static float rotation_count = 0;

static volatile float rpm;			// RPM of motor
static volatile int direction;		// motor direction

 // Initialise the timer and pin change interrupts
 void Initialise_Interrupts()
 {
			/* PHB on Port B pin 1 --- route via PCINT1 */
	 	 	/* PHA on Port B pin 0 --- route via PCINT0 */
	 	 	/* LDR Comparator on Port B pin 4 --- route via PCINT4 */
	        PCMSK0 |= (1 << PCINT0) | (1 << PCINT1) | (1 << PCINT4) ;
	        PCICR  |= (1 << PCIE0);

	        /* Initialise timer0 interrupts */
	        TCCR0A |= (1 << WGM01);					// Set the timer to CTC mode
	        OCR0A = 155;							// Value to count to 155
	        TCCR0B |= (1 << CS02) | (1 << CS00) ;	// 1024 pre-scaler
	        TIMSK0 |= (1 << OCIE0A);				// Enable interrupt flag when timer is matched with OCR0A

	        /* Initialise timer1 interrupts */
	        TCCR1B |= (1 << WGM12);	                // Set the timer to CTC mode
	        OCR1A = 2000;							// Value to count to
	        TCCR1B |= (1 << CS11) ;					// 8 pre-scaler
	        TIMSK1 |= (1 << OCIE1A);				// Enable interrupt flag when timer is matched with OCR0A

	        /* Initialise timer2 interrupts */
	        OCR2A = 0;								// Initial value to count to
	        TCCR2A |= (1 << COM2A1);				// Set non-inverting mode
	        TCCR2A |= (1 << WGM20) | (1 << WGM21);	// Set fast PWM mode
	        TCCR2B |= (1 << CS20) ;					// No pre-scaling

	        /* Globally enable interrupts */
	        sei();
 }

int main(void)
{
	Initialise_Interrupts();	//Initialise interrupts
	/* Set up Port B; input 1 on Bit 0, PWM out of bit 3*/
	DDRB |= (1 << PB3) ;
	/* No pull-up resistor on inputs */
	PORTB = 0x00;
	lcd_init('c');		// Initialise the LCD
	char str[33];
	char rpm_read [10];
	char duty[10];

	while (1)
	{
		cli();							// Disable interrupts
		dtostrf(rpm,2,0,rpm_read);
		dtostrf(((float)OCR2A/255)*100,2,0,duty);	// Calculate duty cycle
		sei();							// Enable interrupts

		//Concatenate to an array
		snprintf(str,33,"RPM = %s    %i\nDuty = %s%c   %i ",rpm_read,count,duty,'%',OCR2A);
		lcd_display(str,0);		// Display the time
		_delay_ms(500);			// Short delay
    	lcd_clear();			//Clear the LCD
	}
	return 0;
}


// Timer interrupt
ISR(TIMER1_COMPA_vect)
{
time_count++;	// increment every 1 ms
}

// Timer0 interrupt for debounce
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
		if(rotation_count < 276)
		{
			rotation_count++;
		}
		else
		{
			rpm = (1/(float)time_count)*60*1000;	// Calculate the RPM
			time_count = 0;
			rotation_count = 0;
		}
	}
	// If the interrupt is on the rising edge of PHB
	if((pin & (1<<PB1)) && !(previous_pin & (1<<PB1)))
	{
		// Determine direction of motor
		if(pin & (1<<PB0))
		{
			direction = 1;	// Forward direction
		}
		else
		{
			direction = -1;	// Reverse direction
		}
	}

	previous_pin = pin;	// Save current pin state for the next interrupt

 // LDR sense
	if (!(pin & (1 << PB4)))
	{
		if (debounce_count == 0)
			{
				count++;
				debounce_count++;
				if(OCR2A >= 255)
				{
					OCR2A = 0;		// Reset duty cycle
				}
				else
				{
					OCR2A += 10;	// Increment PWM duty
				}
			}
	}
}





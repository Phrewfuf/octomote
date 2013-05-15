#include<avr/io.h>
#include<util/delay.h> 
#include<avr/interrupt.h> //manages interrupts
#include<stdint.h> //defines uint8_t
#include<util/twi.h> //contains definitions of TWSR codes

#define SLAVE_ADDRESS 0x10
#define BUFFER_SIZE 5

#define JOY_LT_V PINA0
#define JOY_LT_H PINA1
#define JOY_LB_V PINA2
#define JOY_LB_H PINA3
#define BUT_LT PIND3
#define BUT_LB PIND2

#define TWCR_ACK TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC);
#define TWCR_RESET TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|(0<<TWWC);

uint8_t i2cdata[BUFFER_SIZE];
uint8_t i2cdata_adr;
ISR(TWI_vect);

void setup();
void twi_init_slave(uint8_t address);
void adc_init();
uint8_t readADC(uint8_t channel);

int main (void)
{
	uint8_t i = 0;
	setup();

	for (i = 0; i < 4; i++)
	{
		
		
		//make this thing an andless loop
		if (i == 3)
		{
			i = 0;
		}
	}
	return 0;
}

void setup()
{
	twi_init_slave(SLAVE_ADDRESS);
	adc_init();

	//set BUT_LB and BUT_LT as inputs and activate pull-ups
	DDRD &= ~((1<<BUT_LB)|(1<<BUT_LT));
	PORTB |= ((1<<BUT_LB)|(1<<BUT_LT));
}

void twi_init_slave(uint8_t address);
{
	TWAR = address;
	TWCR &= ~(1<<TWSTA)|(1<<TWSTO);
	TWCR |= (1<<TWEA)|(1<<TWEN)|(1<<TWIE);
	i2cdata_adr = 0xFF;
	sei();
}

ISR(TWI_vect)
{
	switch(TW_STATUS)
	{
		case TW_ST_SLA_ACK: //0xA8, read request by master, ACK sent by slave
		case TW_ST_DATA_ACK: //0xB8, slave transmitter, data requested
			if (i2cdata_adr = 0xFF)
			{
				i2cdata_adr = 0;
			}
			if (i2cdata_adr<BUFFER_SIZE)
			{
				TWDR = i2cdata[i2cdata_adr];
				i2cdata_adr++;
			}
			else
			{
				TWDR = 0;
			}
			TWCR_ACK;
			break;
		case TW_ST_DATA_NACK:
		case TW_ST_LAST_DATA:
		default:
			TWCR_RESET;
			break;
	}
}

void adc_init()
{
	//ADC initialization
	//set Reference Voltage to AVcc
	ADMUX |= (1<<REFS0);
	ADMUX &= ~(1<<REFS1);
	//left-align result, bits 9:2 in ADCH, 1:0 in ADCL
	ADMUX |= (1<<ADLAR);
	//set Channel to ADC0
	ADMUX &= ~((1<<MUX0) | (1<<MUX1) | (1<<MUX2) | (1<<MUX3) | (1<<MUX4));
	//set Prescaler to 125KHz
	ADCSRA |= ((1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0));
	//Disable trigger mode
	ADCSRA &= ~(1<<ADATE);
	//enable high speed mode
	ADCSRB |= (1<<ADHSM);
	//Enabling ADC
	ADCSRA |= (1<<ADEN);
	//initialize ADC
	ADCSRA |= (1<<ADSC);
	// Wait for the ADC conversion to complete
	while(ADCSRA & (1 << ADSC));
}

uint8_t readADC(uint8_t channel)
{
	// Clear the previous result
	ADCH = 0x00;
	ADCL = 0x00;

	//set ADMUX to correct channel without altering the rest
	ADMUX = (ADMUX & ~7) | channel;

	// Set ADSC to start an ADC conversion
	ADCSRA |= (1<<ADSC);

	// Wait for the ADC conversion to complete
	while(ADCSRA & (1 << ADSC));
	return ADCH;
}

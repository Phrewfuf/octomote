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
ISR (TWI_vect);

void twi_init_slave(uint8_t address);
void adc_init();

int main (void)
{
	twi_init_slave(SLAVE_ADDRESS);
	
	return 0;
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
	ADMUX &= ~(1<<ADLAR);
}

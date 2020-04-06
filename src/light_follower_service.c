/*
 Light Follower service
 - user indication
 - check external inputs
 - service utility and routines
 */

#define F_CPU 8000000L

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdio.h>

#define XTAL F_CPU
#define baudrate 19200L
#define bauddivider (XTAL / (16 * baudrate) - 1)
#define HI(x) ((x) >> 8)
#define LO(x) ((x) & 0xff)

#define CHANNEL_NUM 6

/*
 Directions of light
 */
typedef enum {
	front = 3,
	front_left = 4,
	front_right = 2,
	back = 0,
	left = 5,
	right = 1,
} direction_t;

const char Prologue[] PROGMEM = "Lighth Follower Service Demo\n";
const char OutItem[] PROGMEM = "Values: %d, %d, %d, %d, %d, %d; Direction: %S\n";

const char Front[] PROGMEM = "Front";
const char FrontLeft[] PROGMEM = "Front Left";
const char FrontRight[] PROGMEM = "Front Right";
const char Back[] PROGMEM = "Back";
const char Left[] PROGMEM = "Left";
const char Right[] PROGMEM = "Right";

const char * const Directions[] PROGMEM = {
	Back, Right, FrontRight, Front, FrontLeft, Left
};

volatile uint8_t adc_channels[CHANNEL_NUM];
volatile uint8_t processed[CHANNEL_NUM];
volatile direction_t direction;

void sensors_process(void);

static int uart_putchar(char c, FILE *stream);

static FILE uart_stdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

ISR(ADC_vect) {
	uint8_t channel_num = ADMUX & 0x07;
	uint8_t value = ADCL;
	value = ADCH;

	adc_channels[channel_num++] = value;

	if (channel_num >= CHANNEL_NUM) {
		channel_num = 0;
		sensors_process();
	}

	ADMUX = (ADMUX & 0xf8) | channel_num;

	ADCSRA = 1 << ADEN | 1 << ADIE | 1 << ADSC | 3 << ADPS0;
}

void sensors_process(void) {
	uint8_t max_channel_num = 0;
	uint8_t max_value = 0;
	for (uint8_t i = 0; i < CHANNEL_NUM; i++) {
		uint8_t value = 0xff - adc_channels[i];

		processed[i] = value;

		if (value > max_value) {
			max_value = value;
			max_channel_num = i;
		}
	}
	direction = max_channel_num;
}

ISR(USART_TX_vect) {
	printf_P(
		OutItem,
		processed[0],
		processed[1],
		processed[2],
		processed[3],
		processed[4],
		processed[5],
		(char *)pgm_read_word(&Directions[direction]));
}

void hwinit(void) {
	/* UART0 initialization */
	UBRR0L = LO(bauddivider);
	UBRR0H = HI(bauddivider);
	UCSR0A = 0;
	UCSR0B = 1 << RXEN0 | 1 << TXEN0 | 1 << RXCIE0 | 1 << TXCIE0;
	UCSR0C = 1 << UCSZ00 | 1 << UCSZ01;
	/* ADC initialization */
	ADMUX = 1 << REFS0 | 1 << ADLAR | 0 << MUX0;
	/* Enable interrupts */
	sei();
}

void swinit(void) {
	stdout = &uart_stdout;
}

int main(void) {
	hwinit();
	swinit();
	/* Display prologue */
	puts_P(Prologue);
	/* Enable ADC */
	ADCSRA = 1 << ADEN | 1 << ADIE | 1 << ADSC | 3 << ADPS0;

	while (1) {}

	return 0;
}

// Send one char
static int uart_putchar(char c, FILE *stream) {
	if (c == '\n') {
		uart_putchar('\r', stream);
	}

	loop_until_bit_is_set(UCSR0A, UDRE0);
	UDR0 = c;

	return 0;
}
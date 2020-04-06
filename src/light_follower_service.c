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

/*
 Baudrate settings
 */
#define XTAL F_CPU
#define baudrate 19200L
#define bauddivider (XTAL / (16 * baudrate) - 1)
#define HI(x) ((x) >> 8)
#define LO(x) ((x) & 0xff)

/*
 Sensors numbers
 */
#define CHANNEL_NUM 6

/*
 Indicator led pins and pwm registers
 */
#define RED_LED_PIN 3
#define RED_LED_PWM OCR2B
#define GREEN_LED_PIN 5
#define GREEN_LED_PWM OCR0B
#define BLUE_LED_PIN 6
#define BLUE_LED_PWM OCR0A

/*
 Color defines
 */
typedef enum {
	black = 0x0000,
	white = 0xffff,
	yellow = 0xffe0,
	blue = 0x001f,
	red = 0xf800,
	green = 0x07e0,
	cyan = 0x07ff,
	magenta = 0xf81f,
} color_t;

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

static void sensors_process(void);

static int uart_putchar(char c, FILE *stream);

static void set_color(uint16_t color);

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

	ADCSRA = _BV(ADEN) | _BV(ADIE) | _BV(ADSC) | 3 << ADPS0;
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

static void hwinit(void) {
	/* GPIO initialization */
	DDRD = _BV(RED_LED_PIN) | _BV(GREEN_LED_PIN) | _BV(BLUE_LED_PIN);
	/* TIMER initialization */
	TCCR0A = _BV(COM0A0) | _BV(COM0A1) | _BV(COM0B0) | _BV(COM0B1) | _BV(WGM00) | _BV(WGM01);
	TCCR0B = _BV(CS02);

	TCCR2A = _BV(COM2B0) | _BV(COM2B1) | _BV(WGM20) | _BV(WGM21);
	TCCR2B = _BV(CS22);

	set_color(black);
	/* UART0 initialization */
	UBRR0L = LO(bauddivider);
	UBRR0H = HI(bauddivider);
	UCSR0A = 0;
	UCSR0B = _BV(RXEN0) | _BV(TXEN0) | _BV(RXCIE0) | _BV(TXCIE0);
	UCSR0C = _BV(UCSZ00) | _BV(UCSZ01);
	/* ADC initialization */
	ADMUX = _BV(REFS0) | _BV(ADLAR);
	/* Enable interrupts */
	sei();
}

static void swinit(void) {
	stdout = &uart_stdout;
}

int main(void) {
	hwinit();
	swinit();
	/* Display prologue */
	puts_P(Prologue);
	/* Enable ADC */
	ADCSRA = _BV(ADEN) | _BV(ADIE) | _BV(ADSC) | 3 << ADPS0;

	while (1) {}

	return 0;
}

// Set color by 16 bit value
static void set_color(uint16_t color) {
	RED_LED_PWM = (color & 0b1111100000000000) >> 8;
	GREEN_LED_PWM = (color & 0b11111100000) >> 2;
	BLUE_LED_PWM = (color & 0b11111) << 3;
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
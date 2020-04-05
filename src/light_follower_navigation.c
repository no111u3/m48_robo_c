#define F_CPU 8000000L

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#define XTAL F_CPU
#define baudrate 19200L
#define bauddivider (XTAL / (16 * baudrate) - 1)
#define HI(x) ((x) >> 8)
#define LO(x) ((x) & 0xff)

#define CHANNEL_NUM 6

typedef enum {
	front = 3,
	front_left = 4,
	front_right = 2,
	back = 0,
	left = 5,
	right = 1,
} direction_t;

const char Prologue[] PROGMEM = "Lighth Follower Navigation Demo\r\n";
const char OutItem[] PROGMEM = "Direction: ";
const char NewLine[] PROGMEM = "\r\n";

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
volatile direction_t direction;

void sensors_process(void);

void send_str(char *string);
void send_str_p(const char *string);
void send_byte(char byte);

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
		if (value > max_value) {
			max_value = value;
			max_channel_num = i;
		}
	}
	direction = max_channel_num;
}

ISR(USART_TX_vect) {
	send_str_p(OutItem);
	send_str_p((char *)pgm_read_word(&Directions[direction]));
	send_str_p(NewLine);
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

int main(void) {
	hwinit();
	/* Display prologue */
	send_str_p(Prologue);
	/* Enable ADC */
	ADCSRA = 1 << ADEN | 1 << ADIE | 1 << ADSC | 3 << ADPS0;

	while (1) {}

	return 0;
}

// Send string by pointer
void send_str(char *string) {
	while (*string != '\0') {
		send_byte(*string);
		string++;
	}
}

// Send string by pointer on FLASH
void send_str_p(const char *string) {
	while (pgm_read_byte(string) != '\0') {
		send_byte(pgm_read_byte(string));
		string++;
	}
}

// Send one char
void send_byte(char byte) {
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = byte;
}
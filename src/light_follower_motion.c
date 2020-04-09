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
 Button defines
 */
#define WUP_BUTTON_PIN 2
#define BUTTON_JITTER 12

/*
 Button states
 */
typedef enum {
	released = 0,
	pressed,
} button_t;

/*
 Motor control defines
 */
#define MOTOR_L1 0
#define MOTOR_L1_PORT PORTB
#define MOTOR_R1 7
#define MOTOR_R1_PORT PORTB

#define MOTOR_L2 6
#define MOTOR_L2_PORT PORTB
#define MOTOR_R2 4
#define MOTOR_R2_PORT PORTD

#define MOTOR_E1 1
#define MOTOR_E1_PORT PORTB
#define MOTOR_E2 2
#define MOTOR_E2_PORT PORTB

#define MOTOR_E1_PWM OCR1AL
#define MOTOR_E2_PWM OCR1BL

/*
 Motors mode
 */
typedef enum {
	fast_forward,
	forward,
	stop,
	brake,
	backward,
	fast_backward,
} motor_mode_t;

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

const char Prologue[] PROGMEM = "Lighth Follower Motion Demo\n";
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

volatile button_t button_state;

static void sensors_process(void);

static void button_precess(void);

static int uart_putchar(char c, FILE *stream);

static void set_color(uint16_t color);

static void set_motors_mode(motor_mode_t left, motor_mode_t right);

static uint8_t pwm_from_mode(motor_mode_t mode);

static void set_motors_pwm(uint8_t left, uint8_t right);

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

ISR(TIMER0_OVF_vect) {
	button_precess();
}

static void button_precess(void) {
	static volatile uint8_t counter = 0;
	if ((PIND & _BV(WUP_BUTTON_PIN)) == 0) {
		counter++;
	} else {
		counter = 0;
	}

	if (counter > BUTTON_JITTER) {
		button_state = pressed;
	} else {
		button_state = released;
	}
}

static void hwinit(void) {
	/* GPIO initialization */
	/** Enable output pins for indication led */
	DDRD = _BV(RED_LED_PIN) | _BV(GREEN_LED_PIN) | _BV(BLUE_LED_PIN);
	/** Enable output pins for motors */
	DDRD |= _BV(MOTOR_R2);
	DDRB = _BV(MOTOR_L1) | _BV(MOTOR_L2) | _BV(MOTOR_R1) | _BV(MOTOR_E1) | _BV(MOTOR_E2);
	/** Enable pull-up for wake-up button */
	PORTD = _BV(WUP_BUTTON_PIN);
	/* TIMER initialization */
	/** Button debounce handling */
    TIMSK0 = _BV (TOIE0);
    /* Green and Blue channels of LED */
	TCCR0A = _BV(COM0A0) | _BV(COM0A1) | _BV(COM0B0) | _BV(COM0B1) | _BV(WGM00) | _BV(WGM01);
	TCCR0B = _BV(CS02);
	/* Red channel of LED */
	TCCR2A = _BV(COM2B0) | _BV(COM2B1) | _BV(WGM20) | _BV(WGM21);
	TCCR2B = _BV(CS22);
	/** Left and Right motors */
	TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10);
	TCCR1B = _BV(CS12) | _BV(WGM12);

	set_color(black);
	/* UART0 initialization */
	UBRR0L = LO(bauddivider);
	UBRR0H = HI(bauddivider);
	UCSR0A = 0;
	UCSR0B = _BV(RXEN0) | _BV(TXEN0);
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

	/* wait for run */
	while (button_state == released);
	for (uint8_t i = 0; i < 3; i++) {
		set_color(green);
		_delay_ms(200);
		set_color(black);
		_delay_ms(800);
	}

	while (button_state != pressed) {
		printf_P(
			OutItem,
			processed[0],
			processed[1],
			processed[2],
			processed[3],
			processed[4],
			processed[5],
			(char *)pgm_read_word(&Directions[direction]));

		/** Naive motion loop */
		switch (direction) {
			case front:
				set_motors_mode(fast_forward, fast_forward);
				break;
			case front_left:
				set_motors_mode(forward, fast_forward);
				break;
			case left:
				set_motors_mode(stop, fast_forward);
				break;
			case front_right:
				set_motors_mode(fast_forward, forward);
				break;
			case right:
				set_motors_mode(fast_forward, stop);
				break;
			case back:
				set_motors_mode(fast_forward, fast_backward);
			default:
				break;
		}

		/** Display status */
		switch (direction) {
			case front:
				set_color(green);
				break;
			case front_left:
			case left:
			case front_right:
			case right:
				set_color(yellow);
				break;
			case back:
			default:
				set_color(blue);
				break;
		}
		/** Control loop */
		_delay_ms(10);
	}

	puts_P(PSTR("Shutdown..."));
	set_motors_mode(stop, stop);
	set_color(red);

	return 0;
}

// Set color by 16 bit value
static void set_color(uint16_t color) {
	RED_LED_PWM = (color & 0b1111100000000000) >> 8;
	GREEN_LED_PWM = (color & 0b11111100000) >> 2;
	BLUE_LED_PWM = (color & 0b11111) << 3;
}

static void set_motors_mode(motor_mode_t left, motor_mode_t right) {
	uint8_t value_left = pwm_from_mode(left);
	uint8_t value_right = pwm_from_mode(right);

	/** Left motor direction */
	switch (left) {
		case brake:
			MOTOR_L1_PORT |= _BV(MOTOR_L1);
			MOTOR_R1_PORT |= _BV(MOTOR_R1);
			break;
		case forward: case fast_forward:
			MOTOR_L1_PORT |= _BV(MOTOR_L1);
			MOTOR_R1_PORT &= ~_BV(MOTOR_R1);
			break;
		case backward: case fast_backward:
			MOTOR_L1_PORT &= ~_BV(MOTOR_L1);
			MOTOR_R1_PORT |= _BV(MOTOR_R1);
			break;
		case stop: default:
			MOTOR_L1_PORT &= ~_BV(MOTOR_L1);
			MOTOR_R1_PORT &= ~_BV(MOTOR_R1);
			break;
	}

	/** Right motor direction */
	switch (right) {
		case brake:
			MOTOR_L2_PORT |= _BV(MOTOR_L2);
			MOTOR_R2_PORT |= _BV(MOTOR_R2);
			break;
		case forward: case fast_forward:
			MOTOR_L2_PORT |= _BV(MOTOR_L2);
			MOTOR_R2_PORT &= ~_BV(MOTOR_R2);
			break;
		case backward: case fast_backward:
			MOTOR_L2_PORT &= ~_BV(MOTOR_L2);
			MOTOR_R2_PORT |= _BV(MOTOR_R2);
			break;
		case stop: default:
			MOTOR_L2_PORT &= ~_BV(MOTOR_L2);
			MOTOR_R2_PORT &= ~_BV(MOTOR_R2);
			break;
	}

	set_motors_pwm(value_left, value_right);
}

// Pwm value from motor mode
static uint8_t pwm_from_mode(motor_mode_t mode) {
	uint8_t value = 0x00;
	switch (mode) {
		case brake:
			value = 0xff;
			break;
		case fast_backward: case fast_forward:
			value = 0x90;
			break;
		case backward: case forward:
			value = 0x60;
			break;
		case stop: default:
			break;
	}

	return value;
}

// Set pwm value for motors
static void set_motors_pwm(uint8_t left, uint8_t right) {
	MOTOR_E1_PWM = left;
	MOTOR_E2_PWM = right;
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
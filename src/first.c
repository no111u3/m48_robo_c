#define F_CPU 8000000L

#include <avr/io.h>
#include <util/delay.h>

#define XTAL F_CPU
#define baudrate 19200L
#define bauddivider (XTAL / (16 * baudrate) - 1)
#define HI(x) ((x) >> 8)
#define LO(x) ((x) & 0xff)

#define LED1 4
#define LED2 5
#define LED_PORT PORTD
#define LED_DDR DDRD

void hwinit(void) {
	/* UART0 initialization */
	UBRR0L = LO(bauddivider);
	UBRR0H = HI(bauddivider);
	UCSR0A = 0;
	UCSR0B = 1 << RXEN0 | 1 << TXEN0 | 1 << RXCIE0 | 0 << TXCIE0;
	UCSR0C = 1 << UCSZ00 | 1 << UCSZ01;

	/* GPIO initialization */
	LED_DDR = 1 << LED1;
}

int main(void) {
	volatile uint8_t i;

	hwinit();

	while (1) {
		i++;
		LED_PORT = 0 << LED1;
		_delay_ms(1000);
		LED_PORT = 1 << LED1;
		_delay_ms(1000);
	}

	return 0;
}
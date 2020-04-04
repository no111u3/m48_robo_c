#define F_CPU 8000000L

#define XTAL F_CPU
#define baudrate 19200L
#define bauddivider (XTAL / (16 * baudrate) - 1)
#define HI(x) ((x) >> 8)
#define LO(x) ((x) & 0xff)

#include <avr/io.h>
#include <avr/pgmspace.h>

void SendStr(char *string);
void SendStr_P(const char *string);
void SendByte(char byte);

void hwinit(void) {
	/* UART0 initialization */
	UBRR0L = LO(bauddivider);
	UBRR0H = HI(bauddivider);
	UCSR0A = 0;
	UCSR0B = 1 << RXEN0 | 1 << TXEN0 | 1 << RXCIE0 | 0 << TXCIE0;
	UCSR0C = 1 << UCSZ00 | 1 << UCSZ01;
}

const char StringP[] PROGMEM = "Hello in FLASH\r\n";

const char MenuItem0[] PROGMEM = "Menu Item 0\r\n";
const char MenuItem1[] PROGMEM = "Menu Item 1\r\n";
const char MenuItem2[] PROGMEM = "Menu Item 2\r\n";

const char * const MenuItemPtrs[] PROGMEM = { MenuItem0, MenuItem1, MenuItem2 };

int main(void) {
	char String[] = "Hello in RAM\r\n";
	char *u;
	const char *z;

	hwinit();

	u = String;

	z = StringP;

	/// Print string from RAM
	SendStr(u); // Print string from pointer
	SendStr("Hello inline in RAM\r\n"); // Print string constant

	// Print string from FLAS
	SendStr_P(z); // Print string from pointer
	SendStr_P(PSTR("Hello inline in FLASH\r\n")); // Print string constant
	SendStr_P(StringP); // Print by direct pointer

	// Display menu items
	SendStr_P(MenuItem0); // Print main menu item

	// Print other menu items
	SendStr_P((char *)pgm_read_word(&MenuItemPtrs[1]));
	SendStr_P((char *)pgm_read_word(&MenuItemPtrs[2]));

	return 0;
}

// Send string by pointer
void SendStr(char *string) {
	while (*string != '\0') {
		SendByte(*string);
		string++;
	}
}

// Send string by pointer on FLASH
void SendStr_P(const char *string) {
	while (pgm_read_byte(string) != '\0') {
		SendByte(pgm_read_byte(string));
		string++;
	}
}

// Send one char
void SendByte(char byte) {
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = byte;
}
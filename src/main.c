#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdint.h>

/* Pins, Arduino Uno */
#define TRIG_PORT PORTD
#define TRIG_DDR  DDRD
#define TRIG_BIT  PD5      /* D5 */

#define ECHO_PINR PIND
#define ECHO_DDR  DDRD
#define ECHO_BIT  PD6      /* D6 */

/* UART 9600 bps, alleen zenden */
static void uart_init(void) {
    /* UBRR = F_CPU / (16 * baud) - 1  bij 16 MHz en 9600 is dat 103 */
    UBRR0H = (uint8_t)(103 >> 8);
    UBRR0L = (uint8_t)(103 & 0xFF);
    UCSR0B = (1 << TXEN0);                      /* TX aan */
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);     /* 8N1 */
}
static void uart_putc(char c) {
    while (!(UCSR0A & (1 << UDRE0))) {}
    UDR0 = c;
}
static void uart_print(const char *s) {
    while (*s) uart_putc(*s++);
}
static void uart_print_u32(uint32_t v) {
    char buf[11];
    int i = 10;
    buf[i--] = '\0';
    if (v == 0) { uart_putc('0'); return; }
    while (v && i >= 0) { buf[i--] = '0' + (v % 10); v /= 10; }
    uart_print(&buf[i + 1]);
}



static void hcsr04_init(void) {
    
    TRIG_PORT &= ~(1 << TRIG_BIT);
    TRIG_DDR  |=  (1 << TRIG_BIT);

    ECHO_DDR &= ~(1 << ECHO_BIT);
    PORTD    &= ~(1 << ECHO_BIT);

    /* Timer1 prescaler 8, 0,5 us per tick */
    TCCR1A = 0;
    TCCR1B = (1 << CS11);
    TCNT1  = 0;
}

static void hcsr04_trigger(void) {
    TRIG_PORT &= ~(1 << TRIG_BIT);
    _delay_us(2);
    TRIG_PORT |=  (1 << TRIG_BIT);
    _delay_us(10);
    TRIG_PORT &= ~(1 << TRIG_BIT);
}

static bool hcsr04_read_mm(uint32_t *mm_out) {
    hcsr04_trigger();

    uint32_t guard = 40000UL;
    while (!(ECHO_PINR & (1 << ECHO_BIT))) {
        if (--guard == 0) return false;
        _delay_us(1);
    }
    uint16_t t_start = TCNT1;

    guard = 40000UL;
    while (ECHO_PINR & (1 << ECHO_BIT)) {
        if (--guard == 0) return false;
        _delay_us(1);
    }
    uint16_t t_end = TCNT1;

    uint16_t ticks = t_end - t_start;

    /* afstand in mm, afgerond */
    uint32_t mm = ((uint32_t)ticks * 5U + 29U) / 58U;
    *mm_out = mm;
    return true;
}

int main(void) {
    uart_init();
    hcsr04_init();

    while (1) {
        uint32_t mm;
        if (hcsr04_read_mm(&mm)) {
            uart_print("afstand_mm=");
            uart_print_u32(mm);
            uart_print("\r\n");
        } else {
            uart_print("timeout\r\n");
        }
        _delay_ms(100);
    }
}

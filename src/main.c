#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdint.h>

/* ---------- HC-SR04, PD5 trig, PD6 echo ---------- */
#define TRIG_PORT PORTD
#define TRIG_DDR  DDRD
#define TRIG_BIT  PD5

#define ECHO_PINR PIND
#define ECHO_DDR  DDRD
#define ECHO_BIT  PD6

/* ---------- LCD ---------- */
#define PCF8574_ADDR 0x27

#define LCD_RS  (1 << 0)
#define LCD_RW  (1 << 1)
#define LCD_EN  (1 << 2)
#define LCD_BL  (1 << 3)

static uint8_t lcd_bl = LCD_BL;

static void hcsr04_init(void) {
    TRIG_PORT &= ~(1 << TRIG_BIT);
    TRIG_DDR  |=  (1 << TRIG_BIT);

    ECHO_DDR  &= ~(1 << ECHO_BIT);
    PORTD     &= ~(1 << ECHO_BIT);

    /* Timer1, prescaler 8, 0,5 us per tick */
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
    uint32_t mm = ((uint32_t)ticks * 5U + 29U) / 58U;
    *mm_out = mm;
    return true;
}


static void TWI_init_100k(void) {
    TWSR = 0x00;      /* prescaler 1 */
    TWBR = 72;        /* 100 kHz bij 16 MHz */
    TWCR = (1 << TWEN);
    PORTC |= (1 << PC4) | (1 << PC5);  /* pull-ups */
}

static void TWI_start(void) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))) {

    }
}
static void TWI_stop(void) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}
static void TWI_write(uint8_t d) {
    TWDR = d;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))) {

    }
}
static void PCF8574_write(uint8_t d) {
    TWI_start();
    TWI_write((PCF8574_ADDR << 1) | 0);
    TWI_write(d | lcd_bl);
    TWI_stop();
}

/* HD44780 4-bit via PCF8574 */
static void lcd_pulse(uint8_t d) {
    PCF8574_write(d | LCD_EN);
    _delay_us(1);
    PCF8574_write(d & ~LCD_EN);
    _delay_us(50);
}
static void lcd_write4(uint8_t hi, uint8_t rs) {
    uint8_t d = (hi & 0xF0) | (rs ? LCD_RS : 0);
    PCF8574_write(d);
    lcd_pulse(d);
}
static void lcd_write(uint8_t v, uint8_t rs) {
    lcd_write4(v & 0xF0, rs);
    lcd_write4((uint8_t)((v << 4) & 0xF0), rs);
}
static void lcd_cmd(uint8_t c) {
    lcd_write(c, 0);
    if (c == 0x01 || c == 0x02) _delay_ms(2);
}
static void lcd_data(uint8_t c) { lcd_write(c, 1); }

static void lcd_init(void) {
    _delay_ms(50);
    lcd_write4(0x30, 0); _delay_ms(5);
    lcd_write4(0x30, 0); _delay_us(150);
    lcd_write4(0x30, 0); _delay_us(150);
    lcd_write4(0x20, 0);         /* 4 bit */
    lcd_cmd(0x28);               /* 2 lines, 5x8 dots */
    lcd_cmd(0x08);               /* display off */
    lcd_cmd(0x01);               /* clear */
    _delay_ms(2);
    lcd_cmd(0x06);               /* cursor moves right */
    lcd_cmd(0x0C);               /* display on, cursor uit */
}
static void lcd_backlight(uint8_t on) {
    lcd_bl = on ? LCD_BL : 0;
    PCF8574_write(lcd_bl);
}
static void lcd_set_cursor(uint8_t col, uint8_t row) {
    static const uint8_t base[] = {0x00, 0x40, 0x14, 0x54};
    lcd_cmd(0x80 | (base[row] + col));
}
static void lcd_print(const char *s) {
    while (*s){
        lcd_data((uint8_t)*s++);
    }
}
static void lcd_print_u32(uint32_t v) {
    char b[11]; int i = 10; b[i--] = '\0';

    if (v == 0){ 
        lcd_data('0'); 
        return; 
    }

    while(v && i >= 0){ 
        b[i--] = '0' + (v % 10); v /= 10; 
    }

    const char *p = &b[i + 1]; while (*p) lcd_data(*p++);
}

/* ---------- main ---------- */
int main(void) {
    TWI_init_100k();
    lcd_init();
    lcd_backlight(1);
    hcsr04_init();

    while (1) {
        uint32_t mm;
        if (hcsr04_read_mm(&mm)) {
            /* regel 0, label en waarde in het formaat afstand:500 */
            lcd_set_cursor(0, 0);
            lcd_print("afstand:");
            lcd_set_cursor(8, 0);
            lcd_print("      ");
            lcd_set_cursor(8, 0);
            lcd_print_u32(mm);
        } else {
            lcd_set_cursor(0, 0);
            lcd_print("afstand:timeout ");
        }

        _delay_ms(100);
    }

}

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

/* ---------- LCD, PCF8574 ---------- */
#define PCF8574_ADDR 0x27
#define LCD_RS  (1 << 0)
#define LCD_RW  (1 << 1)
#define LCD_EN  (1 << 2)
#define LCD_BL  (1 << 3)
static uint8_t lcd_bl = LCD_BL;

/* ---------- Buzzer op PD3, OC2B, Timer2, continue ---------- */
#define BUZZ_DDR   DDRD
#define BUZZ_PORT  PORTD
#define BUZZ_PIN   PD3

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

    uint16_t ticks = t_end - t_start;                 /* 0,5 us per tick */
    uint32_t mm = ((uint32_t)ticks * 5U + 29U) / 58U; /* afgerond */
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
    while (!(TWCR & (1 << TWINT))) {}
}
static void TWI_stop(void) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}
static void TWI_write(uint8_t d) {
    TWDR = d;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))) {}
}
static void PCF8574_write(uint8_t d) {
    TWI_start();
    TWI_write((PCF8574_ADDR << 1) | 0);
    TWI_write(d | lcd_bl);
    TWI_stop();
}

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
    lcd_write4(0x20, 0);
    lcd_cmd(0x28);
    lcd_cmd(0x08);
    lcd_cmd(0x01);
    _delay_ms(2);
    lcd_cmd(0x06);
    lcd_cmd(0x0C);
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
    while (*s) lcd_data((uint8_t)*s++);
}
static void lcd_print_u32(uint32_t v) {
    char b[11]; int i = 10; b[i--] = '\0';
    if (v == 0) { lcd_data('0'); return; }
    while (v && i >= 0) { b[i--] = '0' + (v % 10); v /= 10; }
    const char *p = &b[i + 1]; while (*p) lcd_data(*p++);
}

static void buzzer_init_pd3(void) {
    BUZZ_DDR |= (1<<BUZZ_PIN);
    /* Fast PWM, TOP = OCR2A, OC2B non inverting, klok nog uit */
    TCCR2A = (1<<WGM21) | (1<<WGM20) | (1<<COM2B1);
    TCCR2B = (1<<WGM22);
    /* vaste prescaler 256, goede resolutie voor 300..3000 Hz */
    OCR2A = 207;                 /* startwaarde, circa 300 Hz */
    OCR2B = (OCR2A + 1) / 2;     /* 50 procent duty */
    TCNT2 = 0;
    TCCR2B |= (1<<CS22) | (1<<CS21); /* prescaler 256, start klok */
}
static void buzzer_set_freq_pd3(uint32_t f_hz) {
    if (f_hz < 300)  f_hz = 300;
    if (f_hz > 3000) f_hz = 3000;
    uint32_t ocr = (F_CPU / (256UL * f_hz)) - 1UL;   /* 19..207 */
    if (ocr < 1)   ocr = 1;
    if (ocr > 255) ocr = 255;
    uint8_t top = (uint8_t)ocr;
    OCR2A = top;
    OCR2B = (uint8_t)((top + 1) / 2);  /* 50 procent duty */
}
static void buzzer_mute_pd3(void) {
    TCCR2A &= ~((1<<COM2B1)|(1<<COM2B0));
    BUZZ_PORT &= ~(1<<BUZZ_PIN);
}
static void buzzer_unmute_pd3(void) {
    TCCR2A |= (1<<COM2B1);
}

static uint32_t map_mm_to_hz(uint32_t mm) {
    const uint32_t mm_min = 50, mm_max = 600;
    const uint32_t f_near = 3000;  /* dichtbij, hoge toon */
    const uint32_t f_far  = 300;   /* ver weg, lage toon */
    if (mm < mm_min) mm = mm_min;
    if (mm > mm_max) mm = mm_max;
    uint32_t num = (mm - mm_min) * (f_near - f_far);
    uint32_t den = (mm_max - mm_min);
    return f_near - num / den;     /* hogere toon bij kleinere afstand */
}


/* Exponential Moving Average, eenvoudige integer versie */
static uint32_t mm_filt = 0;        /* 0 betekent nog niet geïnitialiseerd */
#define EMA_ALPHA  32               /* 32/256 ongeveer 0,125 */
static uint32_t ema_update(uint32_t x) {
    if (mm_filt == 0) { mm_filt = x << 8; }                      /* init */
    mm_filt = mm_filt + ((int32_t)((x << 8) - mm_filt) * EMA_ALPHA) / 256;
    return mm_filt >> 8;
}

int main(void) {
    TWI_init_100k();
    lcd_init();
    lcd_backlight(1);
    hcsr04_init();
    buzzer_init_pd3();

    lcd_set_cursor(0, 0);
    lcd_print("freq:");

    while (1) {
        uint32_t mm_raw;
        if (hcsr04_read_mm(&mm_raw)) {
            uint32_t mm = ema_update(mm_raw);

            if (mm >= 50 && mm <= 600) {
                uint32_t f = map_mm_to_hz(mm);

                
                lcd_set_cursor(5, 0);
                lcd_print("      ");
                lcd_set_cursor(5, 0);
                lcd_print_u32(f);
                lcd_print(" Hz");

                buzzer_unmute_pd3();
                buzzer_set_freq_pd3(f);
            } else {
                buzzer_mute_pd3();
                lcd_set_cursor(5, 0);
                lcd_print("      ");
                lcd_set_cursor(5, 0);
                lcd_print("0 Hz");
            }
        } else {
            buzzer_mute_pd3();
            lcd_set_cursor(5, 0);
            lcd_print("      ");
            lcd_set_cursor(5, 0);
            lcd_print("timeout");
        }

        //_delay_ms(80);
    }

}

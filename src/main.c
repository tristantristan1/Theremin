#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

// I2C LCD
#define PCF8574_ADDR_LCD 0x27
#define LCD_RS (1 << 0)
#define LCD_RW (1 << 1)
#define LCD_EN (1 << 2)
#define LCD_BL (1 << 3)

static uint8_t lcd_bl = LCD_BL;
static uint8_t last_filter_size = 0;

//pins
#define BUZZ_PORT PORTD
#define BUZZ_DDR DDRD
#define BUZZ_PIN PD3
#define TRIG_PORT PORTB
#define TRIG_DDR DDRB
#define TRIG_PIN PB1
#define ECHO_PINR PINB
#define ECHO_DDR DDRB
#define ECHO_PIN PB0 // ICP1

//TWI
static void TWI_init_100k(void) {
    TWSR = 0x00;
    TWBR = 72; //100 kHz bij 16 MHz
    TWCR = (1 << TWEN);
    PORTC |= (1 << PC4) | (1 << PC5); //pull ups
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

//LCD helpers
static void PCF8574_write(uint8_t addr, uint8_t d) {
    TWI_start();
    TWI_write((addr << 1) | 0);
    TWI_write(d);
    TWI_stop();
}

static void lcd_pulse(uint8_t d) {
    PCF8574_write(PCF8574_ADDR_LCD, d | lcd_bl | LCD_EN);
    _delay_us(1);
    PCF8574_write(PCF8574_ADDR_LCD, d | lcd_bl);
    _delay_us(50);
}

static void lcd_write4(uint8_t hi, uint8_t rs) {
    uint8_t out = (hi & 0xF0) | (rs ? LCD_RS : 0);
    PCF8574_write(PCF8574_ADDR_LCD, out | lcd_bl);
    lcd_pulse(out);
}

static void lcd_write(uint8_t v, uint8_t rs) {
    lcd_write4(v & 0xF0, rs);
    lcd_write4((uint8_t)((v << 4) & 0xF0), rs);
}

static void lcd_cmd(uint8_t c) {
    lcd_write(c, 0);
    if (c == 0x01 || c == 0x02) _delay_ms(2);
}

static void lcd_data(uint8_t c) {
    lcd_write(c, 1);
}

static void lcd_init(void) {
    _delay_ms(50);
    lcd_write4(0x30, 0);
    _delay_ms(5);
    lcd_write4(0x30, 0);
    _delay_us(150);
    lcd_write4(0x30, 0);
    _delay_us(150);
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
    PCF8574_write(PCF8574_ADDR_LCD, lcd_bl);
}

static void lcd_set_cursor(uint8_t col, uint8_t row) {
    static const uint8_t base[] = {0x00, 0x40, 0x14, 0x54};
    lcd_cmd(0x80 | (base[row] + col));
}

static void lcd_print(const char *s) {
    while (*s) lcd_data((uint8_t)*s++);
}

static void lcd_print_u32(uint32_t v) {
    char b[11];
    int i = 10;
    b[i--] = '\0';
    if (v == 0) {
        lcd_data('0');
        return;
    }
    while (v && i >= 0) {
        b[i--] = (char)('0' + (v % 10));
        v /= 10;
    }
    const char *p = &b[i + 1];
    while (*p) lcd_data((uint8_t)*p++);
}

//ADC volume, freerun 8 bit via ADCH
static volatile uint8_t g_volume_pct = 100; // 0-100
static volatile uint8_t g_vol_raw255 = 255; //0-255

static void adc_init_freerun_pc0_8bit(void) {
    DDRC &= ~(1 << PC0);
    ADMUX = (1 << REFS0) | (1 << ADLAR) | 0;
    ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    ADCSRB = 0;
    DIDR0 |= (1 << ADC0D);
}

ISR(ADC_vect) {
    uint8_t adch = ADCH;
    g_vol_raw255 = adch;
    uint16_t pct = ((uint16_t)adch * 100U + 127U) / 255U;
    if (pct > 100) pct = 100;
    g_volume_pct = (uint8_t)pct;
}

// Buzzer volume, Timer2 fast PWM 62,5 kHz
static void pwm_init_timer2(void) {
    BUZZ_DDR |= (1 << BUZZ_PIN);
    TCCR2A = (1 << COM2B1) | (1 << WGM21) | (1 << WGM20); //Fast PWM, non inverting
    TCCR2B = (1 << CS20); // no prescaler, 62,5 kHz
    OCR2B = 0;
    TIMSK2 = (1 << TOIE2);
}

ISR(TIMER2_OVF_vect) {}

//Audiotoon, Timer0 in CTC, toggelt COM2B1
static void tone_init_timer0(uint16_t freq_hz) {
    TCCR0A = (1 << WGM01); // CTC
    TCCR0B = (1 << CS02); // prescaler 256
    uint16_t ocr = (F_CPU / (2UL * 256UL * freq_hz)) - 1UL;
    if (ocr > 255) ocr = 255;
    if (ocr < 1) ocr = 1;
    OCR0A = (uint8_t)ocr;
    TIMSK0 = (1 << OCIE0A); // COMPA interrupt
}

ISR(TIMER0_COMPA_vect) {
    TCCR2A ^= (1 << COM2B1); // 50 procent audioplicht
    OCR2B = g_vol_raw255; // duty updaten
}

//Knoppen PD4 en PD5
static volatile uint8_t g_filter_size = 5; //1-15
static volatile uint8_t btn_last = 0x30;

static void buttons_init_pd4_pd5_pcint(void) {
    DDRD &= ~((1 << PD4) | (1 << PD5));
    PORTD |= ((1 << PD4) | (1 << PD5));
    PCICR |= (1 << PCIE2);
    PCMSK2 |= (1 << PCINT20) | (1 << PCINT21);
}

ISR(PCINT2_vect) {
    uint8_t now = PIND & 0x30;
    uint8_t changed = now ^ btn_last;
    if ((changed & (1 << PD4)) && !(now & (1 << PD4))) {
        if (g_filter_size < 15) g_filter_size++;
    }
    if ((changed & (1 << PD5)) && !(now & (1 << PD5))) {
        if (g_filter_size > 1) g_filter_size--;
    }
    btn_last = now;
}

//Ping sensor via Timer1 Input Capture
typedef enum { PING_IDLE, PING_TRIG_HI, PING_WAIT_ECHO } ping_state_t;
static volatile ping_state_t ping_state = PING_IDLE;
static volatile uint16_t icr_rise = 0, icr_fall = 0;
static volatile bool echo_done = false; // 32 bit tijd via overflow teller, tick = 0,5 us bij prescaler 8
static volatile uint32_t t1_overflows = 0;

ISR(TIMER1_OVF_vect) {
    t1_overflows++;
}

static inline uint32_t t1_now_ticks(void) {
    uint16_t now = TCNT1;
    uint32_t ovf = t1_overflows;
    if ((TIFR1 & (1 << TOV1)) && now < 65535) {
        ovf++;
    }
    return (ovf << 16) | now;
}

static void ping_init_icp1(void) {
    TRIG_DDR |= (1 << TRIG_PIN);
    TRIG_PORT &= ~(1 << TRIG_PIN);
    DDRB &= ~(1 << PB0); // ICP1 input
    // Timer1, prescaler 8, noise cancel aan, start op rising edge
    TCCR1A = 0x00;
    TCCR1B = (1 << ICNC1) | (1 << CS11) | (1 << ICES1);
    TIMSK1 = (1 << ICIE1) | (1 << TOIE1); // capture en overflow interrupt
    TCNT1 = 0;
    t1_overflows = 0;
}

static inline void ping_trigger_start(void) {
    TRIG_PORT |= (1 << TRIG_PIN);
    ping_state = PING_TRIG_HI;
}

// minimaal 60 ms tussen pings
#define PING_INTERVAL_TICKS 120000UL // 60 ms bij 0,5 us per tick
static uint32_t next_ping_at = 0;

static inline void ping_state_machine(void) {
    uint32_t now = t1_now_ticks();
    switch (ping_state) {
        case PING_IDLE:
            if ((int32_t)(now - next_ping_at) >= 0) {
                ping_trigger_start();
            }
            break;
        case PING_TRIG_HI:
            _delay_us(10);
            TRIG_PORT &= ~(1 << TRIG_PIN);
            ping_state = PING_WAIT_ECHO;
            break;
        case PING_WAIT_ECHO:
        default:
            break;
    }
}

ISR(TIMER1_CAPT_vect) {
    static bool want_fall = false;
    if (!want_fall) {
        icr_rise = ICR1;
        TCCR1B &= ~(1 << ICES1); // nu falling edge
        want_fall = true;
    } else {
        icr_fall = ICR1;
        TCCR1B |= (1 << ICES1); // terug naar rising
        want_fall = false;
        echo_done = true;
        next_ping_at = t1_now_ticks() + PING_INTERVAL_TICKS;
    }
}

//haal meting op, true als nieuwe afstand beschikbaar is
static bool ping_get_distance_mm(uint32_t *mm_out) {
    if (!echo_done) return false;
    echo_done = false;
    ping_state = PING_IDLE;
    uint16_t ticks = (uint16_t)(icr_fall - icr_rise); /* 0,5 us per tick */
    uint32_t mm = ((uint32_t)ticks * 5U + 29U) / 58U;
    *mm_out = mm;
    return true;
}

//7-segment display
#define SEGMENT_ADDR 0x21  // I2C-adres van het 7-segment display

const uint8_t hex_digits[] = {
    ~0x3F, // 0
    ~0x06, // 1
    ~0x5B, // 2
    ~0x4F, // 3
    ~0x66, // 4
    ~0x6D, // 5
    ~0x7D, // 6
    ~0x07, // 7
    ~0x7F, // 8
    ~0x6F, // 9
    ~0x77, // A
    ~0x7C, // b
    ~0x58, // C
    ~0x5E, // d
    ~0x79, // E
    ~0x71  // F
};

// Zet een hexadecimaal cijfer op de 7-segment display
static void display_hex_digit(uint8_t digit, uint8_t position) {
    if (digit > 15) return;
    
    uint8_t segments = hex_digits[digit];  // Haal de 7-segment code voor het cijfer op
    
    uint8_t i2c_addr = SEGMENT_ADDR;
    
    TWI_start();
    TWI_write((i2c_addr << 1) | 0);  // Schrijfmodus
    
    TWI_write(position);             // Displaypositie
    TWI_write(segments);             // Displaysegmenteer

    TWI_stop();
}

//Frequentiemapping
static inline uint16_t freq_from_cm(uint16_t cm) {
    if (cm < 5) cm = 5;
    if (cm > 65) cm = 65;

    float freq = 1400.0f - ((1400.0f - 230.0f) / 60.0f) * (cm - 5.0f);
    
    if (freq > 1400.0f) freq = 1400.0f;
    if (freq < 230.0f) freq = 230.0f;

    uint16_t ocr = (F_CPU / (2UL * 256UL * (uint16_t)freq)) - 1;
    if (ocr > 255) ocr = 255;

    OCR0A = (uint8_t)ocr;

    return (uint16_t)freq;
}

//main
int main(void) {
    
    TWI_init_100k();
    lcd_init();
    lcd_backlight(1);
    adc_init_freerun_pc0_8bit();
    pwm_init_timer2();
    tone_init_timer0(440);
    ping_init_icp1();
    buttons_init_pd4_pd5_pcint();
    sei();

    lcd_set_cursor(0, 0); 
    lcd_print("D:   cm M:");  // Eerste deel van de bovenste regel voor afstand en filtergrootte
    lcd_set_cursor(0, 1); 
    lcd_print("V:   % Hz:");   // Tweede deel van de onderste regel voor volume en frequentie

    TRIG_PORT &= ~(1 << TRIG_PIN);
    ping_state = PING_IDLE;
    next_ping_at = t1_now_ticks();

    uint16_t last_freq = 440;

    while (1) {
        ping_state_machine();

        uint32_t mm;
        if (ping_get_distance_mm(&mm)) {
            uint16_t cm = (uint16_t)(mm / 10U);
            if (cm < 5) cm = 5;
            if (cm > 65) cm = 65;
            
            uint16_t f = freq_from_cm(cm);
            if (f != last_freq) {
                uint16_t ocr = (F_CPU / (2UL * 256UL * (uint32_t)f)) - 1UL;
                if (ocr > 255) ocr = 255;
                if (ocr < 1) ocr = 1;
                OCR0A = (uint8_t)ocr;
                last_freq = f;
            }

            lcd_set_cursor(3, 0); 
            lcd_print("   ");      
            lcd_set_cursor(3, 0);  
            lcd_print_u32(cm);     
            lcd_print("cm");

            lcd_set_cursor(8, 0);  
            lcd_print("     ");
            lcd_set_cursor(8, 0);  
            lcd_print_u32(g_filter_size);

            // Update het 7-segment display met de filtergrootte in hex
            if (g_filter_size != last_filter_size) {
            display_hex_digit(g_filter_size, 0);  // Zet de filtergrootte op het display
            last_filter_size = g_filter_size;    // Werk de vorige waarde bij
            }
        // Zet de filtergrootte op het display

            lcd_set_cursor(3, 1);  
            lcd_print("   ");      
            lcd_set_cursor(3, 1);  
            lcd_print_u32(g_volume_pct);  
            lcd_print("%");

            lcd_set_cursor(6, 1);  
            lcd_print("     ");    
            lcd_set_cursor(6, 1);  
            lcd_print_u32(f);      
            lcd_print("Hz");
        }

        OCR2B = g_vol_raw255;  
    }
}

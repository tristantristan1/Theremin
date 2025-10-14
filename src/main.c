#include <stdio.h>

// Pin mapping>
#define BUZZER_PORT   PORTD
#define BUZZER_DDR    DDRD
#define BUZZER_PINR   PIND
#define BUZZER_PIN    PD3
#define BUZZER_BIT    (1 << PD3)

// HC-SR04 (Trigger: PB1, Echo: PB0)
#define US_TRIG_PORT  PORTB
#define US_TRIG_DDR   DDRB
#define US_TRIG_PINR  PINB
#define US_TRIG_PIN   PB1
#define US_TRIG_BIT   (1 << PB1)

#define US_ECHO_PORT  PORTB
#define US_ECHO_DDR   DDRB
#define US_ECHO_PINR  PINB
#define US_ECHO_PIN   PB0
#define US_ECHO_BIT   (1 << PB0)

// Buttons (filter +/-) met interne pull-ups (PD4, PD5)
#define BTN_DEC_PORT  PORTD
#define BTN_DEC_DDR   DDRD
#define BTN_DEC_PINR  PIND
#define BTN_DEC_PIN   PD4
#define BTN_DEC_BIT   (1 << PD4)

#define BTN_INC_PORT  PORTD
#define BTN_INC_DDR   DDRD
#define BTN_INC_PINR  PIND
#define BTN_INC_PIN   PD5
#define BTN_INC_BIT   (1 << PD5)

// Potmeter (volume) — ADC0 op PC0
#define POT_ADC_CH    0
#define POT_ADC_PIN   PC0

#define TWI_SDA_PIN   PC4
#define TWI_SCL_PIN   PC5
#define LCD_I2C_ADDR  0x27

int myFunction(int, int);

int main(void) {
    int result = myFunction(2, 3);
    printf("Resultaat: %d\n", result);
    return 0;
}

int myFunction(int x, int y) {
    return x + y;
}
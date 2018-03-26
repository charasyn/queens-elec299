#include <avr/io.h>
long t;

#define SAVE_REG(x) uint16_t bak_##x = x;
#define LOAD_REG(x) x = bak_##x;
void pwnage(void) {
    SAVE_REG(TCCR2A)
    SAVE_REG(TCCR2B)
    SAVE_REG(TCNT2)
    SAVE_REG(OCR2A)
    SAVE_REG(OCR2B)
    SAVE_REG(TIMSK2)
    SAVE_REG(TIFR2)

    TCCR2A = 0x23;
    TCCR2B = 0x01;
    TCNT2 = 0;
    OCR2A = 0;
    TIMSK2 = 0;

    #define playSquareNote(freq, len, duty, volume) \
        for(uint16_t i = 0; i < len;) { \
            OCR2B = volume; \
            delayMicroseconds(10000 * duty / freq); \
            OCR2B = 0; \
            delayMicroseconds(10000 * (100 - duty) / freq); \
            i += 1000 / freq; \
        }
    
    playSquareNote(659.25, 133, 20, 255)
    playSquareNote(783.99, 133, 20, 255)
    playSquareNote(1318.5, 133, 20, 255)
    playSquareNote(1046.5, 133, 20, 255)
    playSquareNote(1174.7, 133, 20, 255)
    playSquareNote(1568.0, 133, 20, 255)
    
    LOAD_REG(TCCR2A)
    LOAD_REG(TCCR2B)
    LOAD_REG(TCNT2)
    LOAD_REG(OCR2A)
    LOAD_REG(OCR2B)
    LOAD_REG(TIMSK2)
    LOAD_REG(TIFR2)
}

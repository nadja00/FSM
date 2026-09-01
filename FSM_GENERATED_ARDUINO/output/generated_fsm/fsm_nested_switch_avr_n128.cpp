/*
 * AUTO-GENERISANO fajlom fsm_generator.py
 * Pattern: nested_switch | Platforma: avr | N_STATES=128 | N_EVENTS=3
 * NE MENJATI RUCNO - regenerisati skriptom radi konzistentnosti sweep-a.
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdlib.h>

#define BAUD 9600
#define UBRR_VAL ((F_CPU / (16UL * BAUD)) - 1)

static void uart_init(void) {
    UBRR0H = (uint8_t)(UBRR_VAL >> 8);
    UBRR0L = (uint8_t)UBRR_VAL;
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}
static void uart_putc(char c) { while (!(UCSR0A & (1 << UDRE0))); UDR0 = c; }
static void uart_puts(const char *s) { while (*s) uart_putc(*s++); }
static void uart_put_uint(uint16_t v) {
    char buf[6]; uint8_t i = 0;
    if (v == 0) { uart_putc('0'); return; }
    while (v > 0) { buf[i++] = '0' + (v % 10); v /= 10; }
    while (i > 0) uart_putc(buf[--i]);
}
static uint8_t uart_available(void) { return (UCSR0A & (1 << RXC0)) != 0; }
static char uart_getc(void) { while (!(UCSR0A & (1 << RXC0))); return UDR0; }

static inline void cycles_start(void) { TCCR1B = 0; TCNT1 = 0; TCCR1B = (1 << CS10); }
static inline uint16_t cycles_stop(void) { TCCR1B = 0; return TCNT1; }
typedef uint16_t cycle_t;


#define NUM_STATES 128
#define NUM_EVENTS 3
enum Event {EV_0, EV_1, EV_2};

static volatile uint8_t current_state = 0;

static uint8_t fsm_transition(uint8_t state, uint8_t event) {
    switch (state) {
    case 0:
        switch (event) {
        case EV_0: return 1;
        case EV_1: return 0;
        case EV_2: return 0;
        default: return 0;
        }
    case 1:
        switch (event) {
        case EV_0: return 2;
        case EV_1: return 1;
        case EV_2: return 1;
        default: return 1;
        }
    case 2:
        switch (event) {
        case EV_0: return 3;
        case EV_1: return 2;
        case EV_2: return 2;
        default: return 2;
        }
    case 3:
        switch (event) {
        case EV_0: return 4;
        case EV_1: return 3;
        case EV_2: return 3;
        default: return 3;
        }
    case 4:
        switch (event) {
        case EV_0: return 5;
        case EV_1: return 4;
        case EV_2: return 4;
        default: return 4;
        }
    case 5:
        switch (event) {
        case EV_0: return 6;
        case EV_1: return 5;
        case EV_2: return 5;
        default: return 5;
        }
    case 6:
        switch (event) {
        case EV_0: return 7;
        case EV_1: return 6;
        case EV_2: return 6;
        default: return 6;
        }
    case 7:
        switch (event) {
        case EV_0: return 8;
        case EV_1: return 7;
        case EV_2: return 7;
        default: return 7;
        }
    case 8:
        switch (event) {
        case EV_0: return 9;
        case EV_1: return 8;
        case EV_2: return 8;
        default: return 8;
        }
    case 9:
        switch (event) {
        case EV_0: return 10;
        case EV_1: return 9;
        case EV_2: return 9;
        default: return 9;
        }
    case 10:
        switch (event) {
        case EV_0: return 11;
        case EV_1: return 10;
        case EV_2: return 10;
        default: return 10;
        }
    case 11:
        switch (event) {
        case EV_0: return 12;
        case EV_1: return 11;
        case EV_2: return 11;
        default: return 11;
        }
    case 12:
        switch (event) {
        case EV_0: return 13;
        case EV_1: return 12;
        case EV_2: return 12;
        default: return 12;
        }
    case 13:
        switch (event) {
        case EV_0: return 14;
        case EV_1: return 13;
        case EV_2: return 13;
        default: return 13;
        }
    case 14:
        switch (event) {
        case EV_0: return 15;
        case EV_1: return 14;
        case EV_2: return 14;
        default: return 14;
        }
    case 15:
        switch (event) {
        case EV_0: return 16;
        case EV_1: return 15;
        case EV_2: return 15;
        default: return 15;
        }
    case 16:
        switch (event) {
        case EV_0: return 17;
        case EV_1: return 16;
        case EV_2: return 16;
        default: return 16;
        }
    case 17:
        switch (event) {
        case EV_0: return 18;
        case EV_1: return 17;
        case EV_2: return 17;
        default: return 17;
        }
    case 18:
        switch (event) {
        case EV_0: return 19;
        case EV_1: return 18;
        case EV_2: return 18;
        default: return 18;
        }
    case 19:
        switch (event) {
        case EV_0: return 20;
        case EV_1: return 19;
        case EV_2: return 19;
        default: return 19;
        }
    case 20:
        switch (event) {
        case EV_0: return 21;
        case EV_1: return 20;
        case EV_2: return 20;
        default: return 20;
        }
    case 21:
        switch (event) {
        case EV_0: return 22;
        case EV_1: return 21;
        case EV_2: return 21;
        default: return 21;
        }
    case 22:
        switch (event) {
        case EV_0: return 23;
        case EV_1: return 22;
        case EV_2: return 22;
        default: return 22;
        }
    case 23:
        switch (event) {
        case EV_0: return 24;
        case EV_1: return 23;
        case EV_2: return 23;
        default: return 23;
        }
    case 24:
        switch (event) {
        case EV_0: return 25;
        case EV_1: return 24;
        case EV_2: return 24;
        default: return 24;
        }
    case 25:
        switch (event) {
        case EV_0: return 26;
        case EV_1: return 25;
        case EV_2: return 25;
        default: return 25;
        }
    case 26:
        switch (event) {
        case EV_0: return 27;
        case EV_1: return 26;
        case EV_2: return 26;
        default: return 26;
        }
    case 27:
        switch (event) {
        case EV_0: return 28;
        case EV_1: return 27;
        case EV_2: return 27;
        default: return 27;
        }
    case 28:
        switch (event) {
        case EV_0: return 29;
        case EV_1: return 28;
        case EV_2: return 28;
        default: return 28;
        }
    case 29:
        switch (event) {
        case EV_0: return 30;
        case EV_1: return 29;
        case EV_2: return 29;
        default: return 29;
        }
    case 30:
        switch (event) {
        case EV_0: return 31;
        case EV_1: return 30;
        case EV_2: return 30;
        default: return 30;
        }
    case 31:
        switch (event) {
        case EV_0: return 32;
        case EV_1: return 31;
        case EV_2: return 31;
        default: return 31;
        }
    case 32:
        switch (event) {
        case EV_0: return 33;
        case EV_1: return 32;
        case EV_2: return 32;
        default: return 32;
        }
    case 33:
        switch (event) {
        case EV_0: return 34;
        case EV_1: return 33;
        case EV_2: return 33;
        default: return 33;
        }
    case 34:
        switch (event) {
        case EV_0: return 35;
        case EV_1: return 34;
        case EV_2: return 34;
        default: return 34;
        }
    case 35:
        switch (event) {
        case EV_0: return 36;
        case EV_1: return 35;
        case EV_2: return 35;
        default: return 35;
        }
    case 36:
        switch (event) {
        case EV_0: return 37;
        case EV_1: return 36;
        case EV_2: return 36;
        default: return 36;
        }
    case 37:
        switch (event) {
        case EV_0: return 38;
        case EV_1: return 37;
        case EV_2: return 37;
        default: return 37;
        }
    case 38:
        switch (event) {
        case EV_0: return 39;
        case EV_1: return 38;
        case EV_2: return 38;
        default: return 38;
        }
    case 39:
        switch (event) {
        case EV_0: return 40;
        case EV_1: return 39;
        case EV_2: return 39;
        default: return 39;
        }
    case 40:
        switch (event) {
        case EV_0: return 41;
        case EV_1: return 40;
        case EV_2: return 40;
        default: return 40;
        }
    case 41:
        switch (event) {
        case EV_0: return 42;
        case EV_1: return 41;
        case EV_2: return 41;
        default: return 41;
        }
    case 42:
        switch (event) {
        case EV_0: return 43;
        case EV_1: return 42;
        case EV_2: return 42;
        default: return 42;
        }
    case 43:
        switch (event) {
        case EV_0: return 44;
        case EV_1: return 43;
        case EV_2: return 43;
        default: return 43;
        }
    case 44:
        switch (event) {
        case EV_0: return 45;
        case EV_1: return 44;
        case EV_2: return 44;
        default: return 44;
        }
    case 45:
        switch (event) {
        case EV_0: return 46;
        case EV_1: return 45;
        case EV_2: return 45;
        default: return 45;
        }
    case 46:
        switch (event) {
        case EV_0: return 47;
        case EV_1: return 46;
        case EV_2: return 46;
        default: return 46;
        }
    case 47:
        switch (event) {
        case EV_0: return 48;
        case EV_1: return 47;
        case EV_2: return 47;
        default: return 47;
        }
    case 48:
        switch (event) {
        case EV_0: return 49;
        case EV_1: return 48;
        case EV_2: return 48;
        default: return 48;
        }
    case 49:
        switch (event) {
        case EV_0: return 50;
        case EV_1: return 49;
        case EV_2: return 49;
        default: return 49;
        }
    case 50:
        switch (event) {
        case EV_0: return 51;
        case EV_1: return 50;
        case EV_2: return 50;
        default: return 50;
        }
    case 51:
        switch (event) {
        case EV_0: return 52;
        case EV_1: return 51;
        case EV_2: return 51;
        default: return 51;
        }
    case 52:
        switch (event) {
        case EV_0: return 53;
        case EV_1: return 52;
        case EV_2: return 52;
        default: return 52;
        }
    case 53:
        switch (event) {
        case EV_0: return 54;
        case EV_1: return 53;
        case EV_2: return 53;
        default: return 53;
        }
    case 54:
        switch (event) {
        case EV_0: return 55;
        case EV_1: return 54;
        case EV_2: return 54;
        default: return 54;
        }
    case 55:
        switch (event) {
        case EV_0: return 56;
        case EV_1: return 55;
        case EV_2: return 55;
        default: return 55;
        }
    case 56:
        switch (event) {
        case EV_0: return 57;
        case EV_1: return 56;
        case EV_2: return 56;
        default: return 56;
        }
    case 57:
        switch (event) {
        case EV_0: return 58;
        case EV_1: return 57;
        case EV_2: return 57;
        default: return 57;
        }
    case 58:
        switch (event) {
        case EV_0: return 59;
        case EV_1: return 58;
        case EV_2: return 58;
        default: return 58;
        }
    case 59:
        switch (event) {
        case EV_0: return 60;
        case EV_1: return 59;
        case EV_2: return 59;
        default: return 59;
        }
    case 60:
        switch (event) {
        case EV_0: return 61;
        case EV_1: return 60;
        case EV_2: return 60;
        default: return 60;
        }
    case 61:
        switch (event) {
        case EV_0: return 62;
        case EV_1: return 61;
        case EV_2: return 61;
        default: return 61;
        }
    case 62:
        switch (event) {
        case EV_0: return 63;
        case EV_1: return 62;
        case EV_2: return 62;
        default: return 62;
        }
    case 63:
        switch (event) {
        case EV_0: return 64;
        case EV_1: return 63;
        case EV_2: return 63;
        default: return 63;
        }
    case 64:
        switch (event) {
        case EV_0: return 65;
        case EV_1: return 64;
        case EV_2: return 64;
        default: return 64;
        }
    case 65:
        switch (event) {
        case EV_0: return 66;
        case EV_1: return 65;
        case EV_2: return 65;
        default: return 65;
        }
    case 66:
        switch (event) {
        case EV_0: return 67;
        case EV_1: return 66;
        case EV_2: return 66;
        default: return 66;
        }
    case 67:
        switch (event) {
        case EV_0: return 68;
        case EV_1: return 67;
        case EV_2: return 67;
        default: return 67;
        }
    case 68:
        switch (event) {
        case EV_0: return 69;
        case EV_1: return 68;
        case EV_2: return 68;
        default: return 68;
        }
    case 69:
        switch (event) {
        case EV_0: return 70;
        case EV_1: return 69;
        case EV_2: return 69;
        default: return 69;
        }
    case 70:
        switch (event) {
        case EV_0: return 71;
        case EV_1: return 70;
        case EV_2: return 70;
        default: return 70;
        }
    case 71:
        switch (event) {
        case EV_0: return 72;
        case EV_1: return 71;
        case EV_2: return 71;
        default: return 71;
        }
    case 72:
        switch (event) {
        case EV_0: return 73;
        case EV_1: return 72;
        case EV_2: return 72;
        default: return 72;
        }
    case 73:
        switch (event) {
        case EV_0: return 74;
        case EV_1: return 73;
        case EV_2: return 73;
        default: return 73;
        }
    case 74:
        switch (event) {
        case EV_0: return 75;
        case EV_1: return 74;
        case EV_2: return 74;
        default: return 74;
        }
    case 75:
        switch (event) {
        case EV_0: return 76;
        case EV_1: return 75;
        case EV_2: return 75;
        default: return 75;
        }
    case 76:
        switch (event) {
        case EV_0: return 77;
        case EV_1: return 76;
        case EV_2: return 76;
        default: return 76;
        }
    case 77:
        switch (event) {
        case EV_0: return 78;
        case EV_1: return 77;
        case EV_2: return 77;
        default: return 77;
        }
    case 78:
        switch (event) {
        case EV_0: return 79;
        case EV_1: return 78;
        case EV_2: return 78;
        default: return 78;
        }
    case 79:
        switch (event) {
        case EV_0: return 80;
        case EV_1: return 79;
        case EV_2: return 79;
        default: return 79;
        }
    case 80:
        switch (event) {
        case EV_0: return 81;
        case EV_1: return 80;
        case EV_2: return 80;
        default: return 80;
        }
    case 81:
        switch (event) {
        case EV_0: return 82;
        case EV_1: return 81;
        case EV_2: return 81;
        default: return 81;
        }
    case 82:
        switch (event) {
        case EV_0: return 83;
        case EV_1: return 82;
        case EV_2: return 82;
        default: return 82;
        }
    case 83:
        switch (event) {
        case EV_0: return 84;
        case EV_1: return 83;
        case EV_2: return 83;
        default: return 83;
        }
    case 84:
        switch (event) {
        case EV_0: return 85;
        case EV_1: return 84;
        case EV_2: return 84;
        default: return 84;
        }
    case 85:
        switch (event) {
        case EV_0: return 86;
        case EV_1: return 85;
        case EV_2: return 85;
        default: return 85;
        }
    case 86:
        switch (event) {
        case EV_0: return 87;
        case EV_1: return 86;
        case EV_2: return 86;
        default: return 86;
        }
    case 87:
        switch (event) {
        case EV_0: return 88;
        case EV_1: return 87;
        case EV_2: return 87;
        default: return 87;
        }
    case 88:
        switch (event) {
        case EV_0: return 89;
        case EV_1: return 88;
        case EV_2: return 88;
        default: return 88;
        }
    case 89:
        switch (event) {
        case EV_0: return 90;
        case EV_1: return 89;
        case EV_2: return 89;
        default: return 89;
        }
    case 90:
        switch (event) {
        case EV_0: return 91;
        case EV_1: return 90;
        case EV_2: return 90;
        default: return 90;
        }
    case 91:
        switch (event) {
        case EV_0: return 92;
        case EV_1: return 91;
        case EV_2: return 91;
        default: return 91;
        }
    case 92:
        switch (event) {
        case EV_0: return 93;
        case EV_1: return 92;
        case EV_2: return 92;
        default: return 92;
        }
    case 93:
        switch (event) {
        case EV_0: return 94;
        case EV_1: return 93;
        case EV_2: return 93;
        default: return 93;
        }
    case 94:
        switch (event) {
        case EV_0: return 95;
        case EV_1: return 94;
        case EV_2: return 94;
        default: return 94;
        }
    case 95:
        switch (event) {
        case EV_0: return 96;
        case EV_1: return 95;
        case EV_2: return 95;
        default: return 95;
        }
    case 96:
        switch (event) {
        case EV_0: return 97;
        case EV_1: return 96;
        case EV_2: return 96;
        default: return 96;
        }
    case 97:
        switch (event) {
        case EV_0: return 98;
        case EV_1: return 97;
        case EV_2: return 97;
        default: return 97;
        }
    case 98:
        switch (event) {
        case EV_0: return 99;
        case EV_1: return 98;
        case EV_2: return 98;
        default: return 98;
        }
    case 99:
        switch (event) {
        case EV_0: return 100;
        case EV_1: return 99;
        case EV_2: return 99;
        default: return 99;
        }
    case 100:
        switch (event) {
        case EV_0: return 101;
        case EV_1: return 100;
        case EV_2: return 100;
        default: return 100;
        }
    case 101:
        switch (event) {
        case EV_0: return 102;
        case EV_1: return 101;
        case EV_2: return 101;
        default: return 101;
        }
    case 102:
        switch (event) {
        case EV_0: return 103;
        case EV_1: return 102;
        case EV_2: return 102;
        default: return 102;
        }
    case 103:
        switch (event) {
        case EV_0: return 104;
        case EV_1: return 103;
        case EV_2: return 103;
        default: return 103;
        }
    case 104:
        switch (event) {
        case EV_0: return 105;
        case EV_1: return 104;
        case EV_2: return 104;
        default: return 104;
        }
    case 105:
        switch (event) {
        case EV_0: return 106;
        case EV_1: return 105;
        case EV_2: return 105;
        default: return 105;
        }
    case 106:
        switch (event) {
        case EV_0: return 107;
        case EV_1: return 106;
        case EV_2: return 106;
        default: return 106;
        }
    case 107:
        switch (event) {
        case EV_0: return 108;
        case EV_1: return 107;
        case EV_2: return 107;
        default: return 107;
        }
    case 108:
        switch (event) {
        case EV_0: return 109;
        case EV_1: return 108;
        case EV_2: return 108;
        default: return 108;
        }
    case 109:
        switch (event) {
        case EV_0: return 110;
        case EV_1: return 109;
        case EV_2: return 109;
        default: return 109;
        }
    case 110:
        switch (event) {
        case EV_0: return 111;
        case EV_1: return 110;
        case EV_2: return 110;
        default: return 110;
        }
    case 111:
        switch (event) {
        case EV_0: return 112;
        case EV_1: return 111;
        case EV_2: return 111;
        default: return 111;
        }
    case 112:
        switch (event) {
        case EV_0: return 113;
        case EV_1: return 112;
        case EV_2: return 112;
        default: return 112;
        }
    case 113:
        switch (event) {
        case EV_0: return 114;
        case EV_1: return 113;
        case EV_2: return 113;
        default: return 113;
        }
    case 114:
        switch (event) {
        case EV_0: return 115;
        case EV_1: return 114;
        case EV_2: return 114;
        default: return 114;
        }
    case 115:
        switch (event) {
        case EV_0: return 116;
        case EV_1: return 115;
        case EV_2: return 115;
        default: return 115;
        }
    case 116:
        switch (event) {
        case EV_0: return 117;
        case EV_1: return 116;
        case EV_2: return 116;
        default: return 116;
        }
    case 117:
        switch (event) {
        case EV_0: return 118;
        case EV_1: return 117;
        case EV_2: return 117;
        default: return 117;
        }
    case 118:
        switch (event) {
        case EV_0: return 119;
        case EV_1: return 118;
        case EV_2: return 118;
        default: return 118;
        }
    case 119:
        switch (event) {
        case EV_0: return 120;
        case EV_1: return 119;
        case EV_2: return 119;
        default: return 119;
        }
    case 120:
        switch (event) {
        case EV_0: return 121;
        case EV_1: return 120;
        case EV_2: return 120;
        default: return 120;
        }
    case 121:
        switch (event) {
        case EV_0: return 122;
        case EV_1: return 121;
        case EV_2: return 121;
        default: return 121;
        }
    case 122:
        switch (event) {
        case EV_0: return 123;
        case EV_1: return 122;
        case EV_2: return 122;
        default: return 122;
        }
    case 123:
        switch (event) {
        case EV_0: return 124;
        case EV_1: return 123;
        case EV_2: return 123;
        default: return 123;
        }
    case 124:
        switch (event) {
        case EV_0: return 125;
        case EV_1: return 124;
        case EV_2: return 124;
        default: return 124;
        }
    case 125:
        switch (event) {
        case EV_0: return 126;
        case EV_1: return 125;
        case EV_2: return 125;
        default: return 125;
        }
    case 126:
        switch (event) {
        case EV_0: return 127;
        case EV_1: return 126;
        case EV_2: return 126;
        default: return 126;
        }
    case 127:
        switch (event) {
        case EV_0: return 0;
        case EV_1: return 127;
        case EV_2: return 127;
        default: return 127;
        }
    default: return state;
    }
}

static cycle_t measured_transition(uint8_t event) {
    cli();
    cycles_start();
    uint8_t next = fsm_transition(current_state, event);
    cycle_t cycles = cycles_stop();
    sei();
    current_state = next;
    return cycles;
}

static void run_benchmark(void) {
    const uint16_t ITER = 1000;
    cycle_t min_c = 0xFFFFFFFF, max_c = 0;
    uint32_t sum_c = 0;

    uart_puts("Running benchmark (");
    uart_put_uint(ITER);
    uart_puts(" transitions)...\r\n");

    for (uint16_t i = 0; i < ITER; i++) {
        cycle_t c = measured_transition(EV_0);
        if (c < min_c) min_c = c;
        if (c > max_c) max_c = c;
        sum_c += c;
    }

    uart_puts("N_STATES: "); uart_put_uint(NUM_STATES); uart_puts("\r\n");
    uart_puts("N_EVENTS: "); uart_put_uint(NUM_EVENTS); uart_puts("\r\n");
    uart_puts("Min cycles: "); uart_put_uint((uint32_t)min_c); uart_puts("\r\n");
    uart_puts("Max cycles: "); uart_put_uint((uint32_t)max_c); uart_puts("\r\n");
    uart_puts("Avg cycles: "); uart_put_uint((uint32_t)(sum_c / ITER)); uart_puts("\r\n");
}

void setup() {
    uart_init();
    uart_puts("\r\nParametrized FSM ready.\r\n");
}

void loop() {
    if (uart_available()) {
        char c = uart_getc();
        if (c == 'b') { run_benchmark(); return; }
    }
}

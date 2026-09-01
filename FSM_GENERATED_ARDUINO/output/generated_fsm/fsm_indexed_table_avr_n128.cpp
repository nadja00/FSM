/*
 * AUTO-GENERISANO fajlom fsm_generator.py
 * Pattern: indexed_table | Platforma: avr | N_STATES=128 | N_EVENTS=3
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

static const uint8_t transition_table[NUM_STATES][NUM_EVENTS] = {
    { 1, 0, 0 },
    { 2, 1, 1 },
    { 3, 2, 2 },
    { 4, 3, 3 },
    { 5, 4, 4 },
    { 6, 5, 5 },
    { 7, 6, 6 },
    { 8, 7, 7 },
    { 9, 8, 8 },
    { 10, 9, 9 },
    { 11, 10, 10 },
    { 12, 11, 11 },
    { 13, 12, 12 },
    { 14, 13, 13 },
    { 15, 14, 14 },
    { 16, 15, 15 },
    { 17, 16, 16 },
    { 18, 17, 17 },
    { 19, 18, 18 },
    { 20, 19, 19 },
    { 21, 20, 20 },
    { 22, 21, 21 },
    { 23, 22, 22 },
    { 24, 23, 23 },
    { 25, 24, 24 },
    { 26, 25, 25 },
    { 27, 26, 26 },
    { 28, 27, 27 },
    { 29, 28, 28 },
    { 30, 29, 29 },
    { 31, 30, 30 },
    { 32, 31, 31 },
    { 33, 32, 32 },
    { 34, 33, 33 },
    { 35, 34, 34 },
    { 36, 35, 35 },
    { 37, 36, 36 },
    { 38, 37, 37 },
    { 39, 38, 38 },
    { 40, 39, 39 },
    { 41, 40, 40 },
    { 42, 41, 41 },
    { 43, 42, 42 },
    { 44, 43, 43 },
    { 45, 44, 44 },
    { 46, 45, 45 },
    { 47, 46, 46 },
    { 48, 47, 47 },
    { 49, 48, 48 },
    { 50, 49, 49 },
    { 51, 50, 50 },
    { 52, 51, 51 },
    { 53, 52, 52 },
    { 54, 53, 53 },
    { 55, 54, 54 },
    { 56, 55, 55 },
    { 57, 56, 56 },
    { 58, 57, 57 },
    { 59, 58, 58 },
    { 60, 59, 59 },
    { 61, 60, 60 },
    { 62, 61, 61 },
    { 63, 62, 62 },
    { 64, 63, 63 },
    { 65, 64, 64 },
    { 66, 65, 65 },
    { 67, 66, 66 },
    { 68, 67, 67 },
    { 69, 68, 68 },
    { 70, 69, 69 },
    { 71, 70, 70 },
    { 72, 71, 71 },
    { 73, 72, 72 },
    { 74, 73, 73 },
    { 75, 74, 74 },
    { 76, 75, 75 },
    { 77, 76, 76 },
    { 78, 77, 77 },
    { 79, 78, 78 },
    { 80, 79, 79 },
    { 81, 80, 80 },
    { 82, 81, 81 },
    { 83, 82, 82 },
    { 84, 83, 83 },
    { 85, 84, 84 },
    { 86, 85, 85 },
    { 87, 86, 86 },
    { 88, 87, 87 },
    { 89, 88, 88 },
    { 90, 89, 89 },
    { 91, 90, 90 },
    { 92, 91, 91 },
    { 93, 92, 92 },
    { 94, 93, 93 },
    { 95, 94, 94 },
    { 96, 95, 95 },
    { 97, 96, 96 },
    { 98, 97, 97 },
    { 99, 98, 98 },
    { 100, 99, 99 },
    { 101, 100, 100 },
    { 102, 101, 101 },
    { 103, 102, 102 },
    { 104, 103, 103 },
    { 105, 104, 104 },
    { 106, 105, 105 },
    { 107, 106, 106 },
    { 108, 107, 107 },
    { 109, 108, 108 },
    { 110, 109, 109 },
    { 111, 110, 110 },
    { 112, 111, 111 },
    { 113, 112, 112 },
    { 114, 113, 113 },
    { 115, 114, 114 },
    { 116, 115, 115 },
    { 117, 116, 116 },
    { 118, 117, 117 },
    { 119, 118, 118 },
    { 120, 119, 119 },
    { 121, 120, 120 },
    { 122, 121, 121 },
    { 123, 122, 122 },
    { 124, 123, 123 },
    { 125, 124, 124 },
    { 126, 125, 125 },
    { 127, 126, 126 },
    { 0, 127, 127 },
};

static inline uint8_t fsm_transition(uint8_t state, uint8_t event) {
    return transition_table[state][event];
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

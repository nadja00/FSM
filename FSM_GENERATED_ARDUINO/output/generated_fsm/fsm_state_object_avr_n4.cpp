/*
 * AUTO-GENERISANO fajlom fsm_generator.py
 * Pattern: state_object | Platforma: avr | N_STATES=4 | N_EVENTS=3
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


#define NUM_STATES 4
#define NUM_EVENTS 3
enum Event {EV_0, EV_1, EV_2};

static volatile uint8_t current_state = 0;

static uint8_t state_0_handle(uint8_t event) {
    if (event == EV_0) return 1;
    return 0;
}
static uint8_t state_1_handle(uint8_t event) {
    if (event == EV_0) return 2;
    return 1;
}
static uint8_t state_2_handle(uint8_t event) {
    if (event == EV_0) return 3;
    return 2;
}
static uint8_t state_3_handle(uint8_t event) {
    if (event == EV_0) return 0;
    return 3;
}

typedef uint8_t (*StateHandler)(uint8_t event);
static StateHandler const state_handlers[NUM_STATES] = {
    state_0_handle,
    state_1_handle,
    state_2_handle,
    state_3_handle
};

static inline uint8_t fsm_transition(uint8_t state, uint8_t event) {
    return state_handlers[state](event);
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

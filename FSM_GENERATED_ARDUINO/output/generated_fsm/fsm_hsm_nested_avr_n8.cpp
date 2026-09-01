/*
 * AUTO-GENERISANO fajlom fsm_generator.py
 * Pattern: hsm_nested | Platforma: avr | N_STATES=8 | N_EVENTS=3
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


#define NUM_STATES 8
#define NUM_EVENTS 3
enum Event {EV_0, EV_1, EV_2};

static volatile uint8_t current_state = 0;

#define EVENT_UNHANDLED 0xFF
typedef uint8_t (*StateHandler)(uint8_t event);
typedef struct { StateHandler handler; int16_t parent; } StateNode;

static uint8_t state_0_handle(uint8_t event) {
    return EVENT_UNHANDLED;
}
static uint8_t state_1_handle(uint8_t event) {
    return EVENT_UNHANDLED;
}
static uint8_t state_2_handle(uint8_t event) {
    return EVENT_UNHANDLED;
}
static uint8_t state_3_handle(uint8_t event) {
    return EVENT_UNHANDLED;
}
static uint8_t state_4_handle(uint8_t event) {
    return EVENT_UNHANDLED;
}
static uint8_t state_5_handle(uint8_t event) {
    return EVENT_UNHANDLED;
}
static uint8_t state_6_handle(uint8_t event) {
    return EVENT_UNHANDLED;
}
static uint8_t state_7_handle(uint8_t event) {
    if (event == EV_0) return 0;
    return EVENT_UNHANDLED;
}

static const StateNode state_table[NUM_STATES] = {
    { state_0_handle, 1 },
    { state_1_handle, 2 },
    { state_2_handle, 3 },
    { state_3_handle, 4 },
    { state_4_handle, 5 },
    { state_5_handle, 6 },
    { state_6_handle, 7 },
    { state_7_handle, -1 },
};

static uint8_t fsm_transition(uint8_t state, uint8_t event) {
    int16_t node = state;
    while (node != -1) {
        uint8_t result = state_table[node].handler(event);
        if (result != EVENT_UNHANDLED) return result;
        node = state_table[node].parent;
    }
    return state;
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

/*
 * AUTO-GENERISANO fajlom fsm_generator.py
 * Pattern: array_of_structs | Platforma: esp32 | N_STATES=32 | N_EVENTS=3
 * NE MENJATI RUCNO - regenerisati skriptom radi konzistentnosti sweep-a.
 */
#include <Arduino.h>
#include <stdint.h>

static void uart_init(void) { Serial.begin(9600); while (!Serial) {} }
static void uart_putc(char c) { Serial.write(c); }
static void uart_puts(const char *s) { Serial.print(s); }
static void uart_put_uint(uint32_t v) { Serial.print(v); }
static uint8_t uart_available(void) { return Serial.available() > 0; }
static char uart_getc(void) { while (!Serial.available()) {} return Serial.read(); }

static inline uint32_t cycles_start(void) { return ESP.getCycleCount(); }
static inline uint32_t cycles_stop(uint32_t start) { return ESP.getCycleCount() - start; }
typedef uint32_t cycle_t;


#define NUM_STATES 32
#define NUM_EVENTS 3
enum Event {EV_0, EV_1, EV_2};

static volatile uint8_t current_state = 0;

typedef struct { uint8_t state; uint8_t event; uint8_t next_state; } Transition;

static const Transition transition_table[] = {
    { 0, EV_0, 1 },
    { 1, EV_0, 2 },
    { 2, EV_0, 3 },
    { 3, EV_0, 4 },
    { 4, EV_0, 5 },
    { 5, EV_0, 6 },
    { 6, EV_0, 7 },
    { 7, EV_0, 8 },
    { 8, EV_0, 9 },
    { 9, EV_0, 10 },
    { 10, EV_0, 11 },
    { 11, EV_0, 12 },
    { 12, EV_0, 13 },
    { 13, EV_0, 14 },
    { 14, EV_0, 15 },
    { 15, EV_0, 16 },
    { 16, EV_0, 17 },
    { 17, EV_0, 18 },
    { 18, EV_0, 19 },
    { 19, EV_0, 20 },
    { 20, EV_0, 21 },
    { 21, EV_0, 22 },
    { 22, EV_0, 23 },
    { 23, EV_0, 24 },
    { 24, EV_0, 25 },
    { 25, EV_0, 26 },
    { 26, EV_0, 27 },
    { 27, EV_0, 28 },
    { 28, EV_0, 29 },
    { 29, EV_0, 30 },
    { 30, EV_0, 31 },
    { 31, EV_0, 0 },
};
#define TABLE_SIZE (sizeof(transition_table) / sizeof(Transition))

static uint8_t fsm_transition(uint8_t state, uint8_t event) {
    for (uint16_t i = 0; i < TABLE_SIZE; i++) {
        if (transition_table[i].state == state && transition_table[i].event == event) {
            return transition_table[i].next_state;
        }
    }
    return state;
}

static cycle_t measured_transition(uint8_t event) {
    noInterrupts();
    uint32_t t0 = cycles_start();
    uint8_t next = fsm_transition(current_state, event);
    cycle_t cycles = cycles_stop(t0);
    interrupts();
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
        if (c == 'b') { run_benchmark(); }
    }
}

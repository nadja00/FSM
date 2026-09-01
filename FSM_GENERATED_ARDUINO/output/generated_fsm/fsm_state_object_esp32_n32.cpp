/*
 * AUTO-GENERISANO fajlom fsm_generator.py
 * Pattern: state_object | Platforma: esp32 | N_STATES=32 | N_EVENTS=3
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
    if (event == EV_0) return 4;
    return 3;
}
static uint8_t state_4_handle(uint8_t event) {
    if (event == EV_0) return 5;
    return 4;
}
static uint8_t state_5_handle(uint8_t event) {
    if (event == EV_0) return 6;
    return 5;
}
static uint8_t state_6_handle(uint8_t event) {
    if (event == EV_0) return 7;
    return 6;
}
static uint8_t state_7_handle(uint8_t event) {
    if (event == EV_0) return 8;
    return 7;
}
static uint8_t state_8_handle(uint8_t event) {
    if (event == EV_0) return 9;
    return 8;
}
static uint8_t state_9_handle(uint8_t event) {
    if (event == EV_0) return 10;
    return 9;
}
static uint8_t state_10_handle(uint8_t event) {
    if (event == EV_0) return 11;
    return 10;
}
static uint8_t state_11_handle(uint8_t event) {
    if (event == EV_0) return 12;
    return 11;
}
static uint8_t state_12_handle(uint8_t event) {
    if (event == EV_0) return 13;
    return 12;
}
static uint8_t state_13_handle(uint8_t event) {
    if (event == EV_0) return 14;
    return 13;
}
static uint8_t state_14_handle(uint8_t event) {
    if (event == EV_0) return 15;
    return 14;
}
static uint8_t state_15_handle(uint8_t event) {
    if (event == EV_0) return 16;
    return 15;
}
static uint8_t state_16_handle(uint8_t event) {
    if (event == EV_0) return 17;
    return 16;
}
static uint8_t state_17_handle(uint8_t event) {
    if (event == EV_0) return 18;
    return 17;
}
static uint8_t state_18_handle(uint8_t event) {
    if (event == EV_0) return 19;
    return 18;
}
static uint8_t state_19_handle(uint8_t event) {
    if (event == EV_0) return 20;
    return 19;
}
static uint8_t state_20_handle(uint8_t event) {
    if (event == EV_0) return 21;
    return 20;
}
static uint8_t state_21_handle(uint8_t event) {
    if (event == EV_0) return 22;
    return 21;
}
static uint8_t state_22_handle(uint8_t event) {
    if (event == EV_0) return 23;
    return 22;
}
static uint8_t state_23_handle(uint8_t event) {
    if (event == EV_0) return 24;
    return 23;
}
static uint8_t state_24_handle(uint8_t event) {
    if (event == EV_0) return 25;
    return 24;
}
static uint8_t state_25_handle(uint8_t event) {
    if (event == EV_0) return 26;
    return 25;
}
static uint8_t state_26_handle(uint8_t event) {
    if (event == EV_0) return 27;
    return 26;
}
static uint8_t state_27_handle(uint8_t event) {
    if (event == EV_0) return 28;
    return 27;
}
static uint8_t state_28_handle(uint8_t event) {
    if (event == EV_0) return 29;
    return 28;
}
static uint8_t state_29_handle(uint8_t event) {
    if (event == EV_0) return 30;
    return 29;
}
static uint8_t state_30_handle(uint8_t event) {
    if (event == EV_0) return 31;
    return 30;
}
static uint8_t state_31_handle(uint8_t event) {
    if (event == EV_0) return 0;
    return 31;
}

typedef uint8_t (*StateHandler)(uint8_t event);
static StateHandler const state_handlers[NUM_STATES] = {
    state_0_handle,
    state_1_handle,
    state_2_handle,
    state_3_handle,
    state_4_handle,
    state_5_handle,
    state_6_handle,
    state_7_handle,
    state_8_handle,
    state_9_handle,
    state_10_handle,
    state_11_handle,
    state_12_handle,
    state_13_handle,
    state_14_handle,
    state_15_handle,
    state_16_handle,
    state_17_handle,
    state_18_handle,
    state_19_handle,
    state_20_handle,
    state_21_handle,
    state_22_handle,
    state_23_handle,
    state_24_handle,
    state_25_handle,
    state_26_handle,
    state_27_handle,
    state_28_handle,
    state_29_handle,
    state_30_handle,
    state_31_handle
};

static inline uint8_t fsm_transition(uint8_t state, uint8_t event) {
    return state_handlers[state](event);
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

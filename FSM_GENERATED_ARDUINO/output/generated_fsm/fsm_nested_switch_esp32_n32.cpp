/*
 * AUTO-GENERISANO fajlom fsm_generator.py
 * Pattern: nested_switch | Platforma: esp32 | N_STATES=32 | N_EVENTS=3
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
        case EV_0: return 0;
        case EV_1: return 31;
        case EV_2: return 31;
        default: return 31;
        }
    default: return state;
    }
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

/*
 * Access Control FSM - Manual HSM Pattern, Test 4a: FLAT (no real hierarchy)
 * Platform: Arduino Uno (ATmega328P), bare-metal register access, no Arduino libs.
 * Measurement: Timer1 free-running cycle counter (TCNT1, no prescaler = 1 cycle @16MHz).
 *
 * FSM: 4 states - IDLE, CHECKING, GRANTED, DENIED (identical to previous versions)
 * Events: EV_VALID, EV_INVALID, EV_TIMEOUT
 *
 * This implements the general HSM dispatch mechanism (event bubbling to parent
 * state if the current state's handler does not consume the event), exactly as
 * used in QP/Harel-style statecharts. However, in THIS test every state's parent
 * is NULL (no real hierarchy) - so this measures the pure overhead of the HSM
 * dispatch mechanism with zero benefit from inheritance. Compare directly with
 * Test 4b, where a real parent/child relationship is introduced.
 *
 * UART commands (9600 baud):
 *   '1' -> EV_VALID
 *   '0' -> EV_INVALID
 *   't' -> EV_TIMEOUT
 *   'b' -> run automatic benchmark (1000 transitions), prints min/avg/max cycles
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

// ---------- States & Events (identical enums to previous versions) ----------
enum State { STATE_IDLE = 0, STATE_CHECKING, STATE_GRANTED, STATE_DENIED, NUM_STATES };
enum Event { EV_VALID = 0, EV_INVALID, EV_TIMEOUT };

#define EVENT_UNHANDLED 0xFF  // sentinel: "this handler does not consume this event"

static volatile uint8_t current_state = STATE_IDLE;

// ---------- UART (raw register access, no Serial lib) ----------
#define BAUD 9600
#define UBRR_VAL ((F_CPU / (16UL * BAUD)) - 1)

static void uart_init(void) {
    UBRR0H = (uint8_t)(UBRR_VAL >> 8);
    UBRR0L = (uint8_t)UBRR_VAL;
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8N1
}

static void uart_putc(char c) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

static void uart_puts(const char *s) {
    while (*s) uart_putc(*s++);
}

static void uart_put_uint(uint16_t v) {
    char buf[6];
    uint8_t i = 0;
    if (v == 0) { uart_putc('0'); return; }
    while (v > 0) { buf[i++] = '0' + (v % 10); v /= 10; }
    while (i > 0) uart_putc(buf[--i]);
}

static uint8_t uart_available(void) {
    return (UCSR0A & (1 << RXC0)) != 0;
}

static char uart_getc(void) {
    while (!(UCSR0A & (1 << RXC0)));
    return UDR0;
}

// ---------- Timer1 cycle counter ----------
static inline void cycles_start(void) {
    TCCR1B = 0;
    TCNT1 = 0;
    TCCR1B = (1 << CS10); // no prescaler
}

static inline uint16_t cycles_stop(void) {
    TCCR1B = 0;
    return TCNT1;
}

// ---------- HSM state descriptor ----------
typedef uint8_t (*StateHandler)(uint8_t event);

typedef struct {
    StateHandler handler;
    int8_t parent; // index into state_table[], or -1 if no parent (top state)
} StateNode;

// ---------- Leaf state handlers ----------
// Each handler returns EVENT_UNHANDLED if it does not process the given event,
// which triggers bubbling to the parent state (if any).

static uint8_t state_idle_handle(uint8_t event) {
    if (event == EV_VALID) return STATE_CHECKING;
    return EVENT_UNHANDLED;
}

static uint8_t state_checking_handle(uint8_t event) {
    if (event == EV_VALID)   return STATE_GRANTED;
    if (event == EV_INVALID) return STATE_DENIED;
    return EVENT_UNHANDLED;
}

static uint8_t state_granted_handle(uint8_t event) {
    if (event == EV_TIMEOUT) return STATE_IDLE;
    return EVENT_UNHANDLED;
}

static uint8_t state_denied_handle(uint8_t event) {
    if (event == EV_TIMEOUT) return STATE_IDLE;
    return EVENT_UNHANDLED;
}

// ---------- State table: every parent is -1 (NO real hierarchy in this test) ----------
static const StateNode state_table[NUM_STATES] = {
    { state_idle_handle,     -1 },
    { state_checking_handle, -1 },
    { state_granted_handle,  -1 },
    { state_denied_handle,   -1 },
};

// ---------- HSM dispatch: try current state, bubble up to parent if unhandled ----------
static uint8_t fsm_transition(uint8_t state, uint8_t event) {
    int8_t node = state;
    uint8_t result;

    while (node != -1) {
        result = state_table[node].handler(event);
        if (result != EVENT_UNHANDLED) {
            return result; // consumed by this state (or an ancestor)
        }
        node = state_table[node].parent; // bubble up
    }
    return state; // nobody in the chain handled it -> stay in current state
}

static const char* state_name(uint8_t s) {
    switch (s) {
        case STATE_IDLE:     return "IDLE";
        case STATE_CHECKING: return "CHECKING";
        case STATE_GRANTED:  return "GRANTED";
        case STATE_DENIED:   return "DENIED";
        default:             return "UNKNOWN";
    }
}

// ---------- Single measured transition ----------
static uint16_t measured_transition(uint8_t event) {
    cli();
    cycles_start();
    uint8_t next = fsm_transition(current_state, event);
    uint16_t cycles = cycles_stop();
    sei();
    current_state = next;
    return cycles;
}

// ---------- Automatic benchmark ----------
static void run_benchmark(void) {
    const uint16_t N = 1000;
    uint16_t min_c = 0xFFFF, max_c = 0;
    uint32_t sum_c = 0;
    uint8_t ev_cycle[3] = { EV_VALID, EV_VALID, EV_TIMEOUT }; // IDLE->CHECKING->GRANTED->IDLE

    uart_puts("Running benchmark (");
    uart_put_uint(N);
    uart_puts(" transitions)...\r\n");

    for (uint16_t i = 0; i < N; i++) {
        uint8_t event = ev_cycle[i % 3];
        uint16_t c = measured_transition(event);
        if (c < min_c) min_c = c;
        if (c > max_c) max_c = c;
        sum_c += c;
    }

    uart_puts("Min cycles: ");  uart_put_uint(min_c); uart_puts("\r\n");
    uart_puts("Max cycles: ");  uart_put_uint(max_c); uart_puts("\r\n");
    uart_puts("Avg cycles: ");  uart_put_uint((uint16_t)(sum_c / N)); uart_puts("\r\n");
}

// ---------- Setup / Loop ----------
void setup() {
    uart_init();
    uart_puts("\r\nAccess FSM - Manual HSM (FLAT, Test 4a) ready.\r\n");
    uart_puts("Commands: 1=VALID 0=INVALID t=TIMEOUT b=BENCHMARK\r\n");
    uart_puts("Current state: IDLE\r\n");
}

void loop() {
    if (uart_available()) {
        char c = uart_getc();
        uint8_t event;

        if (c == 'b') {
            run_benchmark();
            return;
        } else if (c == '1') {
            event = EV_VALID;
        } else if (c == '0') {
            event = EV_INVALID;
        } else if (c == 't') {
            event = EV_TIMEOUT;
        } else {
            return;
        }

        uint16_t cycles = measured_transition(event);

        uart_puts("Event handled -> State: ");
        uart_puts(state_name(current_state));
        uart_puts(" | Cycles: ");
        uart_put_uint(cycles);
        uart_puts("\r\n");
    }
}

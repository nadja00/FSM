/*
 * Access Control FSM - Manual HSM Pattern, Test 4b: NESTED (real hierarchy)
 * Platform: Arduino Uno (ATmega328P), bare-metal register access, no Arduino libs.
 * Measurement: Timer1 free-running cycle counter (TCNT1, no prescaler = 1 cycle @16MHz).
 *
 * FSM: IDLE, CHECKING, DENIED (unchanged) + GRANTED is now a SUPERSTATE with two
 * substates: NORMAL_ACCESS and ADMIN_ACCESS.
 *
 *   GRANTED (super)
 *     - EV_TIMEOUT -> IDLE   (defined ONCE on the parent, inherited by both children)
 *     |-- NORMAL_ACCESS  -- EV_ADMIN -> ADMIN_ACCESS
 *     |-- ADMIN_ACCESS   -- EV_ADMIN -> NORMAL_ACCESS (toggle, just for demo)
 *
 * Compare against Test 4a: there, EV_TIMEOUT->IDLE would have to be duplicated
 * in every child handler under a flat design. Here it is written ONCE on GRANTED
 * and both NORMAL_ACCESS and ADMIN_ACCESS inherit it automatically via bubbling.
 *
 * Events: EV_VALID, EV_INVALID, EV_TIMEOUT, EV_ADMIN
 *
 * UART commands (9600 baud):
 *   '1' -> EV_VALID
 *   '0' -> EV_INVALID
 *   't' -> EV_TIMEOUT
 *   'a' -> EV_ADMIN (toggle NORMAL_ACCESS <-> ADMIN_ACCESS while inside GRANTED)
 *   'b' -> run automatic benchmark (1000 transitions through NORMAL_ACCESS<->ADMIN_ACCESS,
 *          measuring the INHERITED EV_TIMEOUT transition cost)
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

// ---------- States & Events ----------
enum State {
    STATE_IDLE = 0,
    STATE_CHECKING,
    STATE_DENIED,
    STATE_GRANTED,        // superstate (has its own handler for EV_TIMEOUT only)
    STATE_NORMAL_ACCESS,  // substate of GRANTED
    STATE_ADMIN_ACCESS,   // substate of GRANTED
    NUM_STATES
};
enum Event { EV_VALID = 0, EV_INVALID, EV_TIMEOUT, EV_ADMIN };

#define EVENT_UNHANDLED 0xFF

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
    TCCR1B = (1 << CS10);
}

static inline uint16_t cycles_stop(void) {
    TCCR1B = 0;
    return TCNT1;
}

// ---------- HSM state descriptor ----------
typedef uint8_t (*StateHandler)(uint8_t event);

typedef struct {
    StateHandler handler;
    int8_t parent; // -1 = top state (no parent)
} StateNode;

// ---------- Handlers ----------
static uint8_t state_idle_handle(uint8_t event) {
    if (event == EV_VALID) return STATE_CHECKING;
    return EVENT_UNHANDLED;
}

static uint8_t state_checking_handle(uint8_t event) {
    if (event == EV_VALID)   return STATE_NORMAL_ACCESS; // enter GRANTED via its default substate
    if (event == EV_INVALID) return STATE_DENIED;
    return EVENT_UNHANDLED;
}

static uint8_t state_denied_handle(uint8_t event) {
    if (event == EV_TIMEOUT) return STATE_IDLE;
    return EVENT_UNHANDLED;
}

// GRANTED superstate handler: owns the transition shared by BOTH substates.
// Neither NORMAL_ACCESS nor ADMIN_ACCESS need to repeat this logic themselves.
static uint8_t state_granted_handle(uint8_t event) {
    if (event == EV_TIMEOUT) return STATE_IDLE;
    return EVENT_UNHANDLED;
}

// NORMAL_ACCESS substate: only handles what is specific to it (EV_ADMIN).
// EV_TIMEOUT is NOT handled here - it bubbles up to state_granted_handle().
static uint8_t state_normal_access_handle(uint8_t event) {
    if (event == EV_ADMIN) return STATE_ADMIN_ACCESS;
    return EVENT_UNHANDLED;
}

// ADMIN_ACCESS substate: same idea, its own specific behavior only.
static uint8_t state_admin_access_handle(uint8_t event) {
    if (event == EV_ADMIN) return STATE_NORMAL_ACCESS;
    return EVENT_UNHANDLED;
}

// ---------- State table with REAL parent/child relationships ----------
static const StateNode state_table[NUM_STATES] = {
    /* STATE_IDLE          */ { state_idle_handle,          -1 },
    /* STATE_CHECKING      */ { state_checking_handle,      -1 },
    /* STATE_DENIED        */ { state_denied_handle,        -1 },
    /* STATE_GRANTED       */ { state_granted_handle,       -1 },
    /* STATE_NORMAL_ACCESS */ { state_normal_access_handle, STATE_GRANTED }, // child of GRANTED
    /* STATE_ADMIN_ACCESS  */ { state_admin_access_handle,  STATE_GRANTED }, // child of GRANTED
};

// ---------- HSM dispatch: identical mechanism to Test 4a ----------
static uint8_t fsm_transition(uint8_t state, uint8_t event) {
    int8_t node = state;
    uint8_t result;

    while (node != -1) {
        result = state_table[node].handler(event);
        if (result != EVENT_UNHANDLED) {
            return result;
        }
        node = state_table[node].parent;
    }
    return state;
}

static const char* state_name(uint8_t s) {
    switch (s) {
        case STATE_IDLE:          return "IDLE";
        case STATE_CHECKING:      return "CHECKING";
        case STATE_DENIED:        return "DENIED";
        case STATE_GRANTED:       return "GRANTED";
        case STATE_NORMAL_ACCESS: return "NORMAL_ACCESS";
        case STATE_ADMIN_ACCESS:  return "ADMIN_ACCESS";
        default:                  return "UNKNOWN";
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
// Measures the INHERITED transition: bounce NORMAL_ACCESS <-> ADMIN_ACCESS via
// EV_ADMIN (handled locally, no bubbling), interleaved with EV_TIMEOUT (handled
// by bubbling up to the GRANTED parent) - this isolates the bubbling overhead.
static void run_benchmark(void) {
    const uint16_t N = 1000;
    uint16_t min_c = 0xFFFF, max_c = 0;
    uint32_t sum_c = 0;

    uart_puts("Running benchmark (");
    uart_put_uint(N);
    uart_puts(" transitions, forcing state=NORMAL_ACCESS, event=EV_TIMEOUT each time)...\r\n");

    for (uint16_t i = 0; i < N; i++) {
        current_state = STATE_NORMAL_ACCESS; // force substate before each measured bubble-up
        uint16_t c = measured_transition(EV_TIMEOUT);
        if (c < min_c) min_c = c;
        if (c > max_c) max_c = c;
        sum_c += c;
    }

    uart_puts("Min cycles: ");  uart_put_uint(min_c); uart_puts("\r\n");
    uart_puts("Max cycles: ");  uart_put_uint(max_c); uart_puts("\r\n");
    uart_puts("Avg cycles: ");  uart_put_uint((uint16_t)(sum_c / N)); uart_puts("\r\n");
    current_state = STATE_IDLE; // reset after benchmark
}

// ---------- Setup / Loop ----------
void setup() {
    uart_init();
    uart_puts("\r\nAccess FSM - Manual HSM (NESTED, Test 4b) ready.\r\n");
    uart_puts("Commands: 1=VALID 0=INVALID t=TIMEOUT a=ADMIN_TOGGLE b=BENCHMARK\r\n");
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
        } else if (c == 'a') {
            event = EV_ADMIN;
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

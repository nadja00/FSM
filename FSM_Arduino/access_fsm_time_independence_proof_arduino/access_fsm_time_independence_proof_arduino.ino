/*
 * Access Control FSM - PROOF: transition cost is independent of time spent in state
 * Platform: Arduino Uno (ATmega328P), bare-metal register access, no Arduino libs.
 *
 * This sketch experimentally demonstrates that the cost of fsm_transition()
 * (measured in Timer1 cycles) does NOT depend on how long the FSM waited in the
 * previous state before the event arrived. Two scenarios are compared:
 *
 *   Scenario A: CHECKING -> GRANTED, event arrives IMMEDIATELY (no wait).
 *   Scenario B: GRANTED  -> IDLE,    event arrives after an ARTIFICIAL ~3 second
 *               busy-wait delay (simulating a long state duration, e.g. a traffic
 *               light's red phase or a long access-granted timeout).
 *
 * The busy-wait delay uses a SEPARATE mechanism (a manual cycle-counting loop,
 * not Timer1) so it never contaminates the Timer1 measurement window. Timer1
 * is only started right before calling fsm_transition() and stopped right after,
 * exactly as in all previous tests.
 *
 * Expected result: cycle counts for both transitions should be in the same range
 * as previously measured (Nested Switch: ~12-18 cycles), regardless of the delay.
 *
 * UART commands (9600 baud):
 *   'a' -> run Scenario A (immediate transition from CHECKING)
 *   'b' -> run Scenario B (transition from GRANTED after ~3s artificial wait)
 *   'r' -> repeat both scenarios N times and print min/avg/max for each
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <util/delay.h>

// ---------- States & Events (identical to original access-control FSM) ----------
enum State { STATE_IDLE = 0, STATE_CHECKING, STATE_GRANTED, STATE_DENIED };
enum Event { EV_VALID = 0, EV_INVALID, EV_TIMEOUT };

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

// ---------- Timer1 cycle counter (used ONLY for measuring fsm_transition) ----------
static inline void cycles_start(void) {
    TCCR1B = 0;
    TCNT1 = 0;
    TCCR1B = (1 << CS10); // no prescaler
}

static inline uint16_t cycles_stop(void) {
    TCCR1B = 0;
    return TCNT1;
}

// ---------- Artificial "state duration" delay - COMPLETELY SEPARATE from Timer1 ----------
// Uses _delay_ms() (compile-time busy-wait loop, avr/util/delay.h), which does
// NOT touch Timer1 or any of its registers. This simulates "waiting in a state"
// (e.g. GRANTED for 3 seconds) without interfering with the cycle measurement.
static void simulate_state_duration_ms(uint16_t ms) {
    for (uint16_t i = 0; i < ms; i++) {
        _delay_ms(1);
    }
}

// ---------- Nested Switch FSM transition (identical logic to the original test) ----------
static uint8_t fsm_transition(uint8_t state, uint8_t event) {
    switch (state) {
        case STATE_IDLE:
            switch (event) {
                case EV_VALID:   return STATE_CHECKING;
                default:         return STATE_IDLE;
            }
        case STATE_CHECKING:
            switch (event) {
                case EV_VALID:   return STATE_GRANTED;
                case EV_INVALID: return STATE_DENIED;
                default:         return STATE_CHECKING;
            }
        case STATE_GRANTED:
            switch (event) {
                case EV_TIMEOUT: return STATE_IDLE;
                default:         return STATE_GRANTED;
            }
        case STATE_DENIED:
            switch (event) {
                case EV_TIMEOUT: return STATE_IDLE;
                default:         return STATE_DENIED;
            }
        default:
            return STATE_IDLE;
    }
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

// ---------- Single measured transition (unchanged methodology) ----------
static uint16_t measured_transition(uint8_t event) {
    cli();
    cycles_start();
    uint8_t next = fsm_transition(current_state, event);
    uint16_t cycles = cycles_stop();
    sei();
    current_state = next;
    return cycles;
}

// ---------- Scenario A: immediate transition, no wait ----------
static uint16_t scenario_A(void) {
    current_state = STATE_CHECKING; // force starting state
    return measured_transition(EV_VALID); // CHECKING -> GRANTED, immediately
}

// ---------- Scenario B: transition after ~3s artificial "state duration" ----------
static uint16_t scenario_B(void) {
    current_state = STATE_GRANTED; // force starting state
    simulate_state_duration_ms(3000); // simulate "staying" in GRANTED for 3 seconds
    return measured_transition(EV_TIMEOUT); // GRANTED -> IDLE, AFTER the wait
}

// ---------- Repeat both scenarios and compare statistics ----------
static void run_comparison(uint16_t N) {
    uint16_t minA = 0xFFFF, maxA = 0; uint32_t sumA = 0;
    uint16_t minB = 0xFFFF, maxB = 0; uint32_t sumB = 0;

    uart_puts("Running ");
    uart_put_uint(N);
    uart_puts(" repetitions of each scenario (this will take a while due to the 3s delays)...\r\n");

    for (uint16_t i = 0; i < N; i++) {
        uint16_t a = scenario_A();
        if (a < minA) minA = a;
        if (a > maxA) maxA = a;
        sumA += a;

        uint16_t b = scenario_B();
        if (b < minB) minB = b;
        if (b > maxB) maxB = b;
        sumB += b;
    }

    uart_puts("\r\n--- Scenario A: CHECKING->GRANTED, NO wait ---\r\n");
    uart_puts("Min: "); uart_put_uint(minA); uart_puts("  Avg: "); uart_put_uint((uint16_t)(sumA / N));
    uart_puts("  Max: "); uart_put_uint(maxA); uart_puts("\r\n");

    uart_puts("--- Scenario B: GRANTED->IDLE, AFTER ~3s wait ---\r\n");
    uart_puts("Min: "); uart_put_uint(minB); uart_puts("  Avg: "); uart_put_uint((uint16_t)(sumB / N));
    uart_puts("  Max: "); uart_put_uint(maxB); uart_puts("\r\n");

    uart_puts("\r\nIf the two ranges overlap/match, transition cost is proven\r\n");
    uart_puts("independent of prior state duration.\r\n");
}

// ---------- Setup / Loop ----------
void setup() {
    uart_init();
    uart_puts("\r\nTime-independence proof ready.\r\n");
    uart_puts("Commands: a=Scenario A (immediate)  b=Scenario B (~3s wait)  r=Repeat comparison (5x)\r\n");
}

void loop() {
    if (uart_available()) {
        char c = uart_getc();

        if (c == 'a') {
            uint16_t cyc = scenario_A();
            uart_puts("Scenario A -> State: "); uart_puts(state_name(current_state));
            uart_puts(" | Cycles: "); uart_put_uint(cyc); uart_puts("\r\n");
        } else if (c == 'b') {
            uart_puts("Waiting ~3s (simulating state duration)...\r\n");
            uint16_t cyc = scenario_B();
            uart_puts("Scenario B -> State: "); uart_puts(state_name(current_state));
            uart_puts(" | Cycles: "); uart_put_uint(cyc); uart_puts("\r\n");
        } else if (c == 'r') {
            run_comparison(5); // 5 repetitions each (5 * 3s = 15s total for scenario B alone)
        }
    }
}

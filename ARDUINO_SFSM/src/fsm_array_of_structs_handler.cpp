/*
 * Rad: Carlgren, J., Oskarsson, P. W. (2023). "State Machine Model-To-Code
 * Transformation In C." UPTEC F 23044, Uppsala University - sekcija 3.7
 * "Array of Structs" (Figure 9).
 *
 * Access Control FSM - Array of Structs Pattern, VERNIJA varijanta prema radu.
 * FSM: 4 states - IDLE, CHECKING, GRANTED, DENIED (identicno ostalim testovima)
 * Events: EV_VALID, EV_INVALID, EV_TIMEOUT
 *
 * Razlika od fsm_array_of_structs.cpp: svaki unos u nizu struct-ova ovde ne
 * cuva next_state direktno vec POKAZIVAC NA eventHandler FUNKCIJU (tacno kao
 * "stateMachineEventHandler" polje u radu), koja se poziva i vraca next_state.
 * Ovo dodaje jedan indirektni poziv funkcije po tranziciji koji jednostavnija
 * verzija (fsm_array_of_structs.cpp, po uzoru na A. Kumar) nema.
 *
 * UART commands (9600 baud):
 *   '1' -> EV_VALID
 *   '0' -> EV_INVALID
 *   't' -> EV_TIMEOUT
 *   'b' -> run automatic benchmark (1000 transitions), prints min/avg/max cycles
 */
#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

enum State { STATE_IDLE = 0, STATE_CHECKING, STATE_GRANTED, STATE_DENIED };
enum Event { EV_VALID = 0, EV_INVALID, EV_TIMEOUT };

static volatile uint8_t current_state = STATE_IDLE;

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

static inline void cycles_start(void) {
    TCCR1B = 0;
    TCNT1 = 0;
    TCCR1B = (1 << CS10); // no prescaler
}

static inline uint16_t cycles_stop(void) {
    TCCR1B = 0;
    return TCNT1;
}

// eventHandler funkcije - svaka vraca sledece stanje (kao u Fig. 6/9 rada)
typedef uint8_t (*EventHandler)(void);

static uint8_t handler_idle_valid(void)       { return STATE_CHECKING; }
static uint8_t handler_checking_valid(void)   { return STATE_GRANTED; }
static uint8_t handler_checking_invalid(void) { return STATE_DENIED; }
static uint8_t handler_granted_timeout(void)  { return STATE_IDLE; }
static uint8_t handler_denied_timeout(void)   { return STATE_IDLE; }

typedef struct {
    uint8_t state;
    uint8_t event;
    EventHandler handler; // pokazivac na funkciju, ne next_state konstanta
} Transition;

static const Transition transition_table[] = {
    { STATE_IDLE,     EV_VALID,   handler_idle_valid },
    { STATE_CHECKING, EV_VALID,   handler_checking_valid },
    { STATE_CHECKING, EV_INVALID, handler_checking_invalid },
    { STATE_GRANTED,  EV_TIMEOUT, handler_granted_timeout },
    { STATE_DENIED,   EV_TIMEOUT, handler_denied_timeout },
};

#define TABLE_SIZE (sizeof(transition_table) / sizeof(Transition))

static uint8_t fsm_transition(uint8_t state, uint8_t event) {
    for (uint8_t i = 0; i < TABLE_SIZE; i++) {
        if (transition_table[i].state == state && transition_table[i].event == event) {
            return transition_table[i].handler(); // indirektni poziv - dodatni trosak
        }
    }
    return state; // no match -> stay in current state
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

static uint16_t measured_transition(uint8_t event) {
    cli();
    cycles_start();
    uint8_t next = fsm_transition(current_state, event);
    uint16_t cycles = cycles_stop();
    sei();
    current_state = next;
    return cycles;
}

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

void setup() {
    uart_init();
    uart_puts("\r\nAccess FSM - Array of Structs (eventHandler) pattern ready.\r\n");
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

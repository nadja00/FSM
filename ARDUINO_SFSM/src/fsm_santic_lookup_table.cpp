/*
 * Rad: Santic, J. "Writing Efficient State Machines in C."
 * http://johnsantic.com/comp/state.html
 *
 * Access Control FSM - "Santic" Lookup Table Pattern (void akcijske procedure).
 * FSM: 4 states - IDLE, CHECKING, GRANTED, DENIED 
 * Events: EV_VALID, EV_INVALID, EV_TIMEOUT
 *
 * Razlika od fsm_function_pointers.cpp (Carlgren & Oskarsson, Figure 8):
 * tamo handler VRACA next_state (uint8_t (*)(void)), a current_state upisuje
 * pozivalac spolja. Ovde, tacno po Santic-u, akcijska procedura je void
 * (void (*)(void)) i SAMA upisuje current_state kao sporedni efekat - broj
 * indirektnih poziva je isti (jedan), razlikuje se samo MEHANIZAM prenosa
 * sledeceg stanja (return vrednost naspram direktnog upisa globalne
 * promenljive iznutra). Ocekivanje: razlika u ciklusima treba da bude vrlo
 * mala ili nemerljiva, za razliku od svih ostalih poredjenja u ovom radu.
 *
 * Po Santicevom upozorenju da tabela pretrage - za
 * razliku od switch-a - nema default granu za nevalidne indekse (sto bi
 * dovelo do neodredjenog ponasanja/pada programa), ovde je zadrzana njegova
 * eksplicitna provera granica pre poziva iz tabele.
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

#define NUM_STATES 4
#define NUM_EVENTS 3

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

// (void) - svaka sama upisuje current_state
// Tacno po Santic-u: "In an action procedure, you do whatever processing is
// required... you might have to set a new state." Procedure koje ne menjaju
// stanje su prazne (no-op), umesto da vracaju trenutno stanje nazad.

static void action_idle_valid(void)       { current_state = STATE_CHECKING; }
static void action_idle_noop(void)        { /* ostaje u IDLE, nema sta da se radi */ }

static void action_checking_valid(void)   { current_state = STATE_GRANTED; }
static void action_checking_invalid(void) { current_state = STATE_DENIED; }
static void action_checking_noop(void)    { /* ostaje u CHECKING */ }

static void action_granted_noop(void)     { /* ostaje u GRANTED */ }
static void action_granted_timeout(void)  { current_state = STATE_IDLE; }

static void action_denied_noop(void)      { /* ostaje u DENIED */ }
static void action_denied_timeout(void)   { current_state = STATE_IDLE; }

typedef void (*ActionProcedure)(void);

// 2D niz pokazivaca na void procedure - svaka celija se poziva radi sporednog
// efekta (upisa current_state), ne zbog povratne vrednosti
static const ActionProcedure state_table[NUM_STATES][NUM_EVENTS] = {
    /*                     EV_VALID                EV_INVALID                EV_TIMEOUT             */
    /* STATE_IDLE     */ { action_idle_valid,      action_idle_noop,         action_idle_noop       },
    /* STATE_CHECKING */ { action_checking_valid,  action_checking_invalid,  action_checking_noop  },
    /* STATE_GRANTED  */ { action_granted_noop,    action_granted_noop,      action_granted_timeout },
    /* STATE_DENIED   */ { action_denied_noop,     action_denied_noop,       action_denied_timeout  },
};

static const char* state_name(uint8_t s) {
    switch (s) {
        case STATE_IDLE:     
			return "IDLE";
        case STATE_CHECKING: 
			return "CHECKING";
        case STATE_GRANTED:  
			return "GRANTED";
        case STATE_DENIED:   
			return "DENIED";
        default:             
			return "UNKNOWN";
    }
}

// Za razliku od ostalih implementacija u ovom radu, ovde nema fsm_transition()
// koja vraca next_state - tranzicija je sporedni efekat poziva iz tabele.
// Eksplicitna provera granica pre poziva odrazava Santicevo upozorenje da
// tabela pretrage, za razliku od switch-a, nema default granu.
static uint16_t measured_transition(uint8_t event) {
    cli();
    cycles_start();
    if (((event < NUM_EVENTS)) && ((current_state < NUM_STATES))) {
        state_table[current_state][event](); // poziv radi sporednog efekta na current_state
    }
    uint16_t cycles = cycles_stop();
    sei();
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
    uart_puts("\r\nAccess FSM - Santic Lookup Table (void action procedures) pattern ready.\r\n");
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

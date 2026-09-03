/*
 * Radovi:
 *  - Harel, D. (1987). "Statecharts: a Visual Formalism for Complex Systems."
 *    Science of Computer Programming, Vol. 8, pp. 231-274. (entry/exit akcije,
 *    hijerarhijske tranzicije koje se racunaju preko najnizeg zajednickog
 *    pretka - LCA).
 *  - Sunitha, E. V., Samuel, P. (2019). "Automatic Code Generation From UML
 *    State Chart Diagrams." IEEE Access, 7:8591-8608 - implementirano i u
 *    Carlgren & Oskarsson (2023) UPTEC F 23044, sek. 3.11 "Hierarchical State
 *    Pattern" (Figure 12).
 *  - Adamczyk, P. "The Anthology of the Finite State Machine Design Patterns"
 *    - paterni "Basic Statechart" i "Hierarchical Statechart" (entry()/exit()
 *    metode po stanju).
 *
 * Access Control FSM - Manual HSM sa PUNIM entry()/exit() lancem akcija.
 * Razlika od fsm_hsm_nested.cpp: taj fajl radi SAMO bubbling dogadjaja bez
 * ikakvih entry/exit poziva (current_state = next; i gotovo). Ovde se, kao u
 * pravoj statechart semantici, pri svakoj tranziciji:
 *   1) izlazi (exit()) iz svih stanja od trenutnog lista pa do (ukljucujuci)
 *      stanja cija je funkcija hendlera stvarno obradila dogadjaj (owning_node),
 *   2) ulazi (enter()) u sva stanja od najnizeg zajednickog pretka (LCA) pa do
 *      ciljnog lista.
 * Ovo je verniji, ali i skuplji model HSM-a - meri se DODATNI trosak entry/exit
 * poziva koji fsm_hsm_nested.cpp namerno izostavlja.
 *
 * FSM: IDLE, CHECKING, DENIED (top-level) + GRANTED (superstate) sa decom
 * NORMAL_ACCESS i ADMIN_ACCESS - identicna hijerarhija kao u fsm_hsm_nested.cpp.
 * Events: EV_VALID, EV_INVALID, EV_TIMEOUT, EV_ADMIN
 *
 * UART commands (9600 baud):
 *   '1' -> EV_VALID
 *   '0' -> EV_INVALID
 *   't' -> EV_TIMEOUT
 *   'a' -> EV_ADMIN (toggle NORMAL_ACCESS <-> ADMIN_ACCESS unutar GRANTED)
 *   'b' -> run automatic benchmark (1000 transitions), meri INHERITED EV_TIMEOUT
 *          trosak (bubbling + pun exit/entry lanac), za direktno poredjenje sa
 *          fsm_hsm_nested.cpp
 */
#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

enum State {
    STATE_IDLE = 0,
    STATE_CHECKING,
    STATE_DENIED,
    STATE_GRANTED,        // superstate
    STATE_NORMAL_ACCESS,  // substate of GRANTED
    STATE_ADMIN_ACCESS,   // substate of GRANTED
    NUM_STATES
};
enum Event { EV_VALID = 0, EV_INVALID, EV_TIMEOUT, EV_ADMIN };

#define EVENT_UNHANDLED 0xFF

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
    TCCR1B = (1 << CS10);
}

static inline uint16_t cycles_stop(void) {
    TCCR1B = 0;
    return TCNT1;
}

typedef uint8_t (*StateHandler)(uint8_t event);
typedef void (*StateAction)(void);

typedef struct {
    StateHandler handler;
    StateAction enter;
    StateAction exit;
    int8_t parent; // -1 = top state (nema roditelja)
} StateNode;

static uint8_t state_idle_handle(uint8_t event) {
    if (event == EV_VALID) return STATE_CHECKING;
    return EVENT_UNHANDLED;
}

static uint8_t state_checking_handle(uint8_t event) {
    if (event == EV_VALID)   return STATE_NORMAL_ACCESS; // ulazak u GRANTED preko default substate-a
    if (event == EV_INVALID) return STATE_DENIED;
    return EVENT_UNHANDLED;
}

static uint8_t state_denied_handle(uint8_t event) {
    if (event == EV_TIMEOUT) return STATE_IDLE;
    return EVENT_UNHANDLED;
}

static uint8_t state_granted_handle(uint8_t event) {
    if (event == EV_TIMEOUT) return STATE_IDLE;
    return EVENT_UNHANDLED;
}

static uint8_t state_normal_access_handle(uint8_t event) {
    if (event == EV_ADMIN) return STATE_ADMIN_ACCESS;
    return EVENT_UNHANDLED;
}

static uint8_t state_admin_access_handle(uint8_t event) {
    if (event == EV_ADMIN) return STATE_NORMAL_ACCESS;
    return EVENT_UNHANDLED;
}

// entry/exit akcije su namerno prazne (no-op) - zanima nas SAMO trosak poziva,
// ne njihov sadrzaj (u pravoj statechart implementaciji bi ovde bio npr. LED
// upalio/ugasio, tajmer pokrenuo/zaustavio itd.)
static void enter_idle(void)          {}
static void exit_idle(void)           {}
static void enter_checking(void)      {}
static void exit_checking(void)       {}
static void enter_denied(void)        {}
static void exit_denied(void)         {}
static void enter_granted(void)       {}
static void exit_granted(void)        {}
static void enter_normal_access(void) {}
static void exit_normal_access(void)  {}
static void enter_admin_access(void)  {}
static void exit_admin_access(void)   {}

static const StateNode state_table[NUM_STATES] = {
    /* STATE_IDLE          */ { state_idle_handle,          enter_idle,          exit_idle,          -1 },
    /* STATE_CHECKING      */ { state_checking_handle,      enter_checking,      exit_checking,      -1 },
    /* STATE_DENIED        */ { state_denied_handle,        enter_denied,        exit_denied,        -1 },
    /* STATE_GRANTED       */ { state_granted_handle,       enter_granted,       exit_granted,       -1 },
    /* STATE_NORMAL_ACCESS */ { state_normal_access_handle, enter_normal_access, exit_normal_access, STATE_GRANTED },
    /* STATE_ADMIN_ACCESS  */ { state_admin_access_handle,  enter_admin_access,  exit_admin_access,  STATE_GRANTED },
};

// HSM dispatch sa punim LCA-baziranim exit/entry lancem (Harel/statechart semantika)
static uint8_t fsm_transition(uint8_t state, uint8_t event) {
    // 1) bubbling: nadji stanje ciji handler stvarno obradi dogadjaj
    int8_t owning_node = state;
    uint8_t result = EVENT_UNHANDLED;
    while (owning_node != -1) {
        result = state_table[owning_node].handler(event);
        if (result != EVENT_UNHANDLED) break;
        owning_node = state_table[owning_node].parent;
    }
    if (owning_node == -1) {
        return state; // niko nije obradio dogadjaj -> bez exit/entry poziva
    }

    uint8_t next_state = result;

    // 2) exit lanac: od trenutnog lista pa do (ukljucujuci) owning_node
    int8_t node = state;
    while (1) {
        state_table[node].exit();
        if (node == owning_node) break;
        node = state_table[node].parent;
    }

    // 3) entry lanac: od cvora ciji je roditelj == roditelj(owning_node), do next_state (top-down)
    int8_t preserved_ancestor = state_table[owning_node].parent;
    int8_t path[NUM_STATES];
    uint8_t count = 0;
    int8_t n = next_state;
    while (state_table[n].parent != preserved_ancestor) {
        path[count++] = n;
        n = state_table[n].parent;
    }
    path[count++] = n;
    for (int8_t i = count - 1; i >= 0; i--) {
        state_table[path[i]].enter();
    }

    return next_state;
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

static uint16_t measured_transition(uint8_t event) {
    cli();
    cycles_start();
    uint8_t next = fsm_transition(current_state, event);
    uint16_t cycles = cycles_stop();
    sei();
    current_state = next;
    return cycles;
}

// Isti scenario kao u fsm_hsm_nested.cpp radi direktnog poredjenja: forsiraj
// NORMAL_ACCESS pa izmeri EV_TIMEOUT (bubbling do GRANTED + pun exit/entry lanac).
static void run_benchmark(void) {
    const uint16_t N = 1000;
    uint16_t min_c = 0xFFFF, max_c = 0;
    uint32_t sum_c = 0;

    uart_puts("Running benchmark (");
    uart_put_uint(N);
    uart_puts(" transitions, forcing state=NORMAL_ACCESS, event=EV_TIMEOUT each time)...\r\n");

    for (uint16_t i = 0; i < N; i++) {
        current_state = STATE_NORMAL_ACCESS;
        uint16_t c = measured_transition(EV_TIMEOUT);
        if (c < min_c) min_c = c;
        if (c > max_c) max_c = c;
        sum_c += c;
    }

    uart_puts("Min cycles: ");  uart_put_uint(min_c); uart_puts("\r\n");
    uart_puts("Max cycles: ");  uart_put_uint(max_c); uart_puts("\r\n");
    uart_puts("Avg cycles: ");  uart_put_uint((uint16_t)(sum_c / N)); uart_puts("\r\n");
    current_state = STATE_IDLE;
}

void setup() {
    uart_init();
    uart_puts("\r\nAccess FSM - HSM Statechart (entry/exit) pattern ready.\r\n");
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

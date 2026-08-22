/*
 * SWITCH-CASE BREAK
 * FSM: 4 states - START, M_STATE, A_STATE, T_STATE
 * Events: INV_CHAR, M_FOUND, A_FOUND, T_FOUND
 * 
 * UART commands (9600 baud):
 *   't' -> T_FOUND
 *   'a' -> A_FOUND
 *   'm' -> M_FOUND
 *   '0' -> INV_CHAR
 *   'b' -> run automatic benchmark (1000 transitions), prints min/avg/max cycles
 * 
 */

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdio.h>

enum State {START, M_STATE, A_STATE, T_STATE};
enum Event {INV_CHAR = 0, M_FOUND = 'm', A_FOUND = 'a', T_FOUND = 't'};

static volatile uint8_t current_state = START;

#define BAUD 9600
#define UBRR_VAL ((F_CPU / (16UL * BAUD)) - 1)  //ovde ispadne oko 9615 BR

static void uart_init(void) {
    UBRR0H = (uint8_t)(UBRR_VAL >> 8);
    UBRR0L = (uint8_t)UBRR_VAL;
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8N1 SA 1 STOP BITOM
}

static void uart_putc(char c) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

static void uart_puts(const char *s) {
    while (*s) 
        uart_putc(*s++);
}

static void uart_put_uint(uint16_t v) {
    char buf[6];
    uint8_t i = 0;
    if (v == 0) { 
        uart_putc('0'); 
        return; 
    }

    while (v > 0) {
        buf[i++] = '0' + (v % 10); 
        v /= 10; 
    }
    

    while (i > 0) 
        uart_putc(buf[--i]);
}

static uint8_t uart_available(void) {
    return (UCSR0A & (1 << RXC0)) != 0;
}

static char uart_getc(void) {
    while (!(UCSR0A & (1 << RXC0)));
    return UDR0;
}

static inline void cycles_start(void) {
    TCCR1B = 0;         // stop timer
    TCNT1 = 0;           // reset counter
    TCCR1B = (1 << CS10); // start, no prescaler (1 cycle per tick)
}

static inline uint16_t cycles_stop(void) {
    TCCR1B = 0;          // stop timer
    return TCNT1;         // read elapsed cycles
}

static uint8_t fsm_transition(uint8_t state, uint8_t event) {
    uint8_t next_state = START;

    switch (state) {
        case START:
            switch (event) {
                case M_FOUND:
                    next_state = M_STATE;
                    break;
                default:
                    next_state = START;
                    break;
            }
            break;
        case M_STATE:
            switch (event) {
                case A_FOUND:
                    next_state = A_STATE;
                    break;
                case INV_CHAR:
                    next_state = START;
                    break;
                default:
                    next_state = M_STATE;
                    break;
            }
            break;
        case A_STATE:
            switch (event) {
                case T_FOUND:
                    next_state = T_STATE;
                    break;
                default:
                    next_state = START;
                    break;
            }
            break;
        case T_STATE:
            next_state = START;
            break;
        default:
            next_state = START;
            break;
    }

    return next_state;
}

static const char* state_name(uint8_t s) {
    const char* state = "START";

    switch (s) {
        case START:
            state = "START";
            break;
        case M_STATE:
            state = "M_STATE";
            break;
        case A_STATE:
            state = "A_STATE";
            break;
        case T_STATE:
            state = "T_STATE";
            break;
        default:
            state = "UNKNOWN";
            break;
    }

    return state;
}

static uint8_t measured_transition(uint8_t event) {
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
    uint8_t ev_cycle[4] = { M_FOUND, A_FOUND, T_FOUND, INV_CHAR };

    uart_puts("Running benchmark (");
    uart_put_uint(N);
    uart_puts("transitions)...\r\n");

    for (uint16_t i = 0; i < N; i++) {
        uint8_t event = ev_cycle[i % 4];
        uint16_t c = measured_transition(event);

        if (c < min_c) min_c = c;
        if (c > max_c) max_c = c;

        sum_c += c;
    }

    uart_puts("Min cycles: ");  
    uart_put_uint(min_c); 
    uart_puts("\r\n");
    uart_puts("Max cycles: ");  
    uart_put_uint(max_c); 
    uart_puts("\r\n");
    uart_puts("Avg cycles: ");  
    uart_put_uint((uint16_t)(sum_c / N)); 
    uart_puts("\r\n");
}

void setup() {
    uart_init();
    uart_puts("\r\nString FSM - Nested Switch pattern ready.\r\n");
    uart_puts("Commands: m=M_FOUND a=A_FOUND t=T_FOUND 0=INV_CHAR b=BENCHMARK\r\n");
    uart_puts("Current state: START\r\n");
}

void loop() {
    if (uart_available()) {
        char c = uart_getc();
        uint8_t event;

        if (c == 'b') {
            run_benchmark();
            return;
        } else if (c == 'm') {
            event = M_FOUND;
        } else if (c == 'a') {
            event = A_FOUND;
        } else if (c == 't') {
            event = T_FOUND;
        } else {
            return; // ignore unknown input
        }

        uint16_t cycles = measured_transition(event);

        uart_puts("Event handled -> State: ");
        uart_puts(state_name(current_state));
        uart_puts(" | Cycles: ");
        uart_put_uint(cycles);
        uart_puts("\r\n");
    }
}

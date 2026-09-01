/*
 * fsm_hsm_flat_MAT.cpp
 *
 * Flat FSM za prepoznavanje stringa "mat", izvrsena preko HSM-kompatibilnog
 * dispatcher-a. Ovo NIJE pravi hijerarhijski HSM: svako stanje ima parent=-1.
 * Koristi se kao bazna/kontrolna verzija za merenje overhead-a dispatcher-a
 * pre poredjenja sa fsm_hsm_nested_MAT.cpp.
 *
 * UART, 9600 baud:
 *   m  -> M_FOUND
 *   a  -> A_FOUND
 *   t  -> T_FOUND
 *   bilo koji drugi znak -> INV_CHAR
 *   b  -> benchmark: 1000 normalnih prelaza
 */

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdio.h>

enum State : uint8_t {
    START = 0,
    M_STATE,
    A_STATE,
    T_STATE,
    NUM_STATES
};

enum Event : uint8_t {
    INV_CHAR = 0,
    M_FOUND,
    A_FOUND,
    T_FOUND
};

#define EVENT_UNHANDLED 0xFFu
#define BAUD 9600UL
#define UBRR_VAL ((F_CPU / (16UL * BAUD)) - 1UL)

static volatile uint8_t current_state = START;

typedef uint8_t (*HsmHandler)(uint8_t event);

typedef struct {
    HsmHandler handler;
    int8_t parent;
} HsmState;

static void uart_init(void) {
    UBRR0H = (uint8_t)(UBRR_VAL >> 8);
    UBRR0L = (uint8_t)UBRR_VAL;
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

static void uart_putc(char c) {
    while (!(UCSR0A & (1 << UDRE0))) {
    }
    UDR0 = c;
}

static void uart_puts(const char *s) {
    while (*s) {
        uart_putc(*s++);
    }
}

static void uart_put_uint(uint16_t value) {
    char buf[6];
    uint8_t i = 0;

    if (value == 0) {
        uart_putc('0');
        return;
    }

    while (value > 0) {
        buf[i++] = (char)('0' + (value % 10));
        value /= 10;
    }

    while (i > 0) {
        uart_putc(buf[--i]);
    }
}

static uint8_t uart_available(void) {
    return (UCSR0A & (1 << RXC0)) != 0;
}

static char uart_getc(void) {
    while (!(UCSR0A & (1 << RXC0))) {
    }
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

static uint8_t start_handle(uint8_t event) {
    if (event == M_FOUND) {
        return M_STATE;
    }
    if (event == INV_CHAR || event == A_FOUND || event == T_FOUND) {
        return START;
    }
    return EVENT_UNHANDLED;
}

static uint8_t m_state_handle(uint8_t event) {
    if (event == M_FOUND) {
        return M_STATE;
    }
    if (event == A_FOUND) {
        return A_STATE;
    }
    if (event == INV_CHAR || event == T_FOUND) {
        return START;
    }
    return EVENT_UNHANDLED;
}

static uint8_t a_state_handle(uint8_t event) {
    if (event == T_FOUND) {
        return T_STATE;
    }
    if (event == INV_CHAR || event == M_FOUND || event == A_FOUND) {
        return START;
    }
    return EVENT_UNHANDLED;
}

static uint8_t t_state_handle(uint8_t event) {
    (void)event;
    return START;
}

static const HsmState hsm_states[NUM_STATES] = {
    { start_handle,   -1 },
    { m_state_handle, -1 },
    { a_state_handle, -1 },
    { t_state_handle, -1 }
};

static uint8_t fsm_transition(uint8_t state, uint8_t event) {
    if (state >= NUM_STATES) {
        return START;
    }

    int8_t s = (int8_t)state;

    while (s >= 0) {
        uint8_t next = hsm_states[(uint8_t)s].handler(event);

        if (next != EVENT_UNHANDLED) {
            return next;
        }

        s = hsm_states[(uint8_t)s].parent;
    }

    return START;
}

static const char *state_name(uint8_t state) {
    switch (state) {
        case START:   return "START";
        case M_STATE: return "M_STATE";
        case A_STATE: return "A_STATE";
        case T_STATE: return "T_STATE";
        default:      return "UNKNOWN";
    }
}

static uint16_t measured_transition(uint8_t event) {
    uint8_t next;
    uint16_t cycles;

    cli();
    cycles_start();
    next = fsm_transition(current_state, event);
    cycles = cycles_stop();
    sei();

    current_state = next;
    return cycles;
}

static void report_result(uint8_t previous_state, uint16_t cycles) {
    uart_puts("Event handled: ");
    uart_puts(state_name(previous_state));
    uart_puts(" -> ");
    uart_puts(state_name(current_state));

    if (current_state == T_STATE) {
        uart_puts(" | MATCH: mat");
    }

    uart_puts(" | Cycles: ");
    uart_put_uint(cycles);
    uart_puts("\r\n");
}

static void run_benchmark(void) {
    const uint16_t N = 1000;
    const uint8_t event_cycle[4] = { M_FOUND, A_FOUND, T_FOUND, INV_CHAR };
    uint16_t min_cycles = 0xFFFF;
    uint16_t max_cycles = 0;
    uint32_t sum_cycles = 0;

    current_state = START;

    uart_puts("Running benchmark: ");
    uart_put_uint(N);
    uart_puts(" flat-dispatch transitions...\r\n");

    for (uint16_t i = 0; i < N; ++i) {
        uint16_t cycles = measured_transition(event_cycle[i % 4]);

        if (cycles < min_cycles) {
            min_cycles = cycles;
        }
        if (cycles > max_cycles) {
            max_cycles = cycles;
        }
        sum_cycles += cycles;
    }

    uart_puts("Min cycles: ");
    uart_put_uint(min_cycles);
    uart_puts("\r\nMax cycles: ");
    uart_put_uint(max_cycles);
    uart_puts("\r\nAvg cycles: ");
    uart_put_uint((uint16_t)(sum_cycles / N));
    uart_puts("\r\n");
}

void setup(void) {
    uart_init();
    uart_puts("\r\nHSM-capable dispatcher, flat MAT FSM ready.\r\n");
    uart_puts("No state has a parent; this file is the flat baseline.\r\n");
    uart_puts("Commands: m, a, t; every other char is invalid; b=benchmark.\r\n");
    uart_puts("Current state: START\r\n");
}

void loop(void) {
    if (!uart_available()) {
        return;
    }

    char c = uart_getc();

    if (c == 'b') {
        run_benchmark();
        return;
    }

    uint8_t event;
    if (c == 'm') {
        event = M_FOUND;
    } else if (c == 'a') {
        event = A_FOUND;
    } else if (c == 't') {
        event = T_FOUND;
    } else {
        event = INV_CHAR;
    }

    uint8_t previous_state = current_state;
    uint16_t cycles = measured_transition(event);
    report_result(previous_state, cycles);
}

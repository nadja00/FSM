/*
 * Rad: A. Kumar, "How to implement finite state machine in C," aticleworld.com,
 * avgust 2017 - originalni clanak na kom je zasnovan ovaj pristup (direktno
 * next_state u struct-u, linearna pretraga). Pominje se i u: Carlgren,
 * Oskarsson (2023) UPTEC F 23044, sek. 2.8.2 "Array of Structs" (referenca [2]
 * = isti A. Kumar clanak). Za varijantu sa eventHandler pokazivacem na funkciju
 * (Figure 9 u radu), vidi fsm_array_of_structs_handler.cpp.
 *
 * Access Control FSM - Array of Structs Pattern
 * FSM: 4 states - IDLE, CHECKING, GRANTED, DENIED (identical to Nested Switch version)
 * Events: EV_VALID, EV_INVALID, EV_TIMEOUT
 *
 * Difference from Nested Switch: transition lookup is a LINEAR SEARCH over an
 * array of {state, event, next_state} structs, instead of two nested switch statements.
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
#include <time.h>
#include <stdlib.h>
#include <string.h>

enum State
{
    S0,
    S1,
    S2,
    S3,
    S4,
    S5,
    S6,
    S7,
    S8,
    S9,
    S10,
    S11,
    S12,
    S13,
    S14,
    S15
};
enum Event
{
    ConnectC2,
    ConnectC1WithWill,
    PublishQoS0C2,
    PublishQoS1C1,
    SubscribeC1,
    UnSubScribeC1,
    SubscribeC2,
    UnSubScribeC2,
    DisconnectTCPC1,
};

#define NUM_EVENTS 9

static volatile uint8_t current_state = S0;

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

typedef struct {
    uint8_t state;
    uint8_t event;
    uint8_t next_state;
} Transition;

static const Transition transition_table[] = {
    // State S0 transitions
    { S0, ConnectC2         , S3 },
    { S0, ConnectC1WithWill , S1 },
    { S0, PublishQoS0C2     , S0 },
    { S0, PublishQoS1C1     , S0 },
    { S0, SubscribeC1       , S0 },
    { S0, UnSubScribeC1     , S0 },
    { S0, SubscribeC2       , S0 },
    { S0, UnSubScribeC2     , S0 },
    { S0, DisconnectTCPC1   , S0 },

    // State S1 transitions
    { S1, ConnectC2         , S2 },
    { S1, ConnectC1WithWill , S4 },
    { S1, PublishQoS0C2     , S1 },
    { S1, PublishQoS1C1     , S1 },
    { S1, SubscribeC1       , S14 },
    { S1, UnSubScribeC1     , S1 },
    { S1, SubscribeC2       , S1 },
    { S1, UnSubScribeC2     , S1 },
    { S1, DisconnectTCPC1   , S0 },

    // State S2 transitions
    { S2, ConnectC2         , S8 },
    { S2, ConnectC1WithWill , S5 },
    { S2, PublishQoS0C2     , S2 },
    { S2, PublishQoS1C1     , S2 },
    { S2, SubscribeC1       , S11 },
    { S2, UnSubScribeC1     , S2 },
    { S2, SubscribeC2       , S6 },
    { S2, UnSubScribeC2     , S2 },
    { S2, DisconnectTCPC1   , S3 },

    // State S3 transitions
    { S3, ConnectC2         , S9 },
    { S3, ConnectC1WithWill , S2 },
    { S3, PublishQoS0C2     , S3 },
    { S3, PublishQoS1C1     , S3 },
    { S3, SubscribeC1       , S3 },
    { S3, UnSubScribeC1     , S3 },
    { S3, SubscribeC2       , S13 },
    { S3, UnSubScribeC2     , S3 },
    { S3, DisconnectTCPC1   , S3 },

    // State S4 transitions
    { S4, ConnectC2         , S5 },
    { S4, ConnectC1WithWill , S1 },
    { S4, PublishQoS0C2     , S4 },
    { S4, PublishQoS1C1     , S4 },
    { S4, SubscribeC1       , S4 },
    { S4, UnSubScribeC1     , S4 },
    { S4, SubscribeC2       , S4 },
    { S4, UnSubScribeC2     , S4 },
    { S4, DisconnectTCPC1   , S0 },

    // State S5 transitions
    { S5, ConnectC2         , S12 },
    { S5, ConnectC1WithWill , S2 },
    { S5, PublishQoS0C2     , S5 },
    { S5, PublishQoS1C1     , S5 },
    { S5, SubscribeC1       , S5 },
    { S5, UnSubScribeC1     , S5 },
    { S5, SubscribeC2       , S7 },
    { S5, UnSubScribeC2     , S5 },
    { S5, DisconnectTCPC1   , S3 },

    // State S6 transitions
    { S6, ConnectC2         , S8 },
    { S6, ConnectC1WithWill , S7 },
    { S6, PublishQoS0C2     , S6 },
    { S6, PublishQoS1C1     , S6 },
    { S6, SubscribeC1       , S10 },
    { S6, UnSubScribeC1     , S6 },
    { S6, SubscribeC2       , S6 },
    { S6, UnSubScribeC2     , S2 },
    { S6, DisconnectTCPC1   , S13 },

    // State S7 transitions
    { S7, ConnectC2         , S12 },
    { S7, ConnectC1WithWill , S6 },
    { S7, PublishQoS0C2     , S7 },
    { S7, PublishQoS1C1     , S7 },
    { S7, SubscribeC1       , S7 },
    { S7, UnSubScribeC1     , S7 },
    { S7, SubscribeC2       , S7 },
    { S7, UnSubScribeC2     , S5 },
    { S7, DisconnectTCPC1   , S13 },

    // State S8 transitions
    { S8, ConnectC2         , S2 },
    { S8, ConnectC1WithWill , S12 },
    { S8, PublishQoS0C2     , S8 },
    { S8, PublishQoS1C1     , S8 },
    { S8, SubscribeC1       , S15 },
    { S8, UnSubScribeC1     , S8 },
    { S8, SubscribeC2       , S8 },
    { S8, UnSubScribeC2     , S8 },
    { S8, DisconnectTCPC1   , S9 },

    // State S9 transitions
    { S9, ConnectC2         , S3 },
    { S9, ConnectC1WithWill , S8 },
    { S9, PublishQoS0C2     , S9 },
    { S9, PublishQoS1C1     , S9 },
    { S9, SubscribeC1       , S9 },
    { S9, UnSubScribeC1     , S9 },
    { S9, SubscribeC2       , S9 },
    { S9, UnSubScribeC2     , S9 },
    { S9, DisconnectTCPC1   , S9 },

    // State S10 transitions
    { S10, ConnectC2         , S15 },
    { S10, ConnectC1WithWill , S7 },
    { S10, PublishQoS0C2     , S10 },
    { S10, PublishQoS1C1     , S10 },
    { S10, SubscribeC1       , S10 },
    { S10, UnSubScribeC1     , S6 },
    { S10, SubscribeC2       , S10 },
    { S10, UnSubScribeC2     , S11 },
    { S10, DisconnectTCPC1   , S13 },

    // State S11 transitions
    { S11, ConnectC2         , S15 },
    { S11, ConnectC1WithWill , S5 },
    { S11, PublishQoS0C2     , S11 },
    { S11, PublishQoS1C1     , S11 },
    { S11, SubscribeC1       , S11 },
    { S11, UnSubScribeC1     , S2 },
    { S11, SubscribeC2       , S10 },
    { S11, UnSubScribeC2     , S11 },
    { S11, DisconnectTCPC1   , S3 },

    // State S12 transitions
    { S12, ConnectC2         , S5 },
    { S12, ConnectC1WithWill , S8 },
    { S12, PublishQoS0C2     , S12 },
    { S12, PublishQoS1C1     , S12 },
    { S12, SubscribeC1       , S12 },
    { S12, UnSubScribeC1     , S12 },
    { S12, SubscribeC2       , S12 },
    { S12, UnSubScribeC2     , S12 },
    { S12, DisconnectTCPC1   , S9 },

    // State S13 transitions
    { S13, ConnectC2         , S9 },
    { S13, ConnectC1WithWill , S6 },
    { S13, PublishQoS0C2     , S13 },
    { S13, PublishQoS1C1     , S13 },
    { S13, SubscribeC1       , S13 },
    { S13, UnSubScribeC1     , S13 },
    { S13, SubscribeC2       , S13 },
    { S13, UnSubScribeC2     , S3 },
    { S13, DisconnectTCPC1   , S13 },

    // State S14 transitions
    { S14, ConnectC2         , S11 },
    { S14, ConnectC1WithWill , S4 },
    { S14, PublishQoS0C2     , S14 },
    { S14, PublishQoS1C1     , S14 },
    { S14, SubscribeC1       , S14 },
    { S14, UnSubScribeC1     , S1 },
    { S14, SubscribeC2       , S14 },
    { S14, UnSubScribeC2     , S14 },
    { S14, DisconnectTCPC1   , S0 },

    // State S15 transitions
    { S15, ConnectC2         , S11 },
    { S15, ConnectC1WithWill , S12 },
    { S15, PublishQoS0C2     , S15 },
    { S15, PublishQoS1C1     , S15 },
    { S15, SubscribeC1       , S15 },
    { S15, UnSubScribeC1     , S8 },
    { S15, SubscribeC2       , S15 },
    { S15, UnSubScribeC2     , S15 },
    { S15, DisconnectTCPC1   , S9 },
};

#define TABLE_SIZE (sizeof(transition_table) / sizeof(Transition))

static uint8_t fsm_transition(uint8_t state, uint8_t event) {
    for (uint8_t i = 0; i < TABLE_SIZE; i++) {
        if (transition_table[i].state == state && transition_table[i].event == event) {
            return transition_table[i].next_state;
        }
    }
    return state; // no match -> stay in current state
}

static const char *state_name(uint8_t s)
{
    switch(s) {
        case S0:  return "S0";
        case S1:  return "S1";
        case S2:  return "S2";
        case S3:  return "S3";
        case S4:  return "S4";
        case S5:  return "S5";
        case S6:  return "S6";
        case S7:  return "S7";
        case S8:  return "S8";
        case S9:  return "S9";
        case S10: return "S10";
        case S11: return "S11";
        case S12: return "S12";
        case S13: return "S13";
        case S14: return "S14";
        case S15: return "S15";
        default:  return "UNKNOWN";
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
    uint8_t event = rand() % NUM_EVENTS;

    uart_puts("Running benchmark (");
    uart_put_uint(N);
    uart_puts(" transitions)...\r\n");

    for (uint16_t i = 0; i < N; i++) {
        event = rand() % NUM_EVENTS;
        uint16_t c = measured_transition(event);
        if (c < min_c) min_c = c;
        if (c > max_c) max_c = c;
        sum_c += c;
    }

    uart_puts("Min cycles: ");  uart_put_uint(min_c); uart_puts("\r\n");
    uart_puts("Max cycles: ");  uart_put_uint(max_c); uart_puts("\r\n");
    uart_puts("Avg cycles: ");  uart_put_uint((uint16_t)(sum_c / N)); uart_puts("\r\n");
}

void setup()
{
    uart_init();
    uart_puts("\r\nMQTT FSM - Nested Switch pattern ready.\r\n");
    uart_puts("Commands: num(dec)=STATE_NUM b=BENCHMARK\r\n");
    uart_puts("Current state: S0\r\n");

    srand(time(0));
}

void loop()
{
    if (uart_available())
    {
        char c = uart_getc();
        uint8_t event;

        if (c == 'b')
        {
            run_benchmark();
            return;
        }

        event = atoi(&c);

        if (event < 0 && event >= NUM_EVENTS)
        {
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

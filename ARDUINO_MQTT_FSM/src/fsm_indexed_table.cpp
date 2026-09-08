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
    S15,
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

#define NUM_STATES 16
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

static const uint8_t transition_table[NUM_STATES][NUM_EVENTS] = {
    /*             ConnectC2 ConnectC1WithWill PublishQoS0C2 PublishQoS1C1 SubscribeC1 UnSubScribeC1 SubscribeC2 UnSubScribeC2 DisconnectTCPC1 */
    /* S0    */ { S3, S1, S0, S0, S0, S0, S0, S0, S0 },
    /* S1    */ { S2, S4, S1, S1, S14, S1, S1, S1, S0 },
    /* S2    */ { S8, S5, S2, S2, S11, S2, S6, S2, S3 },
    /* S3    */ { S9, S2, S3, S3, S3, S3, S13, S3, S3 },
    /* S4    */ { S5, S1, S4, S4, S4, S4, S4, S4, S0 },
    /* S5    */ { S12, S2, S5, S5, S5, S5, S7, S5, S3 },
    /* S6    */ { S8, S7, S6, S6, S10, S6, S6, S2, S13 },
    /* S7    */ { S12, S6, S7, S7, S7, S7, S7, S5, S13 },
    /* S8    */ { S2, S12, S8, S8, S15, S8, S8, S8, S9 },
    /* S9    */ { S3, S8, S9, S9, S9, S9, S9, S9, S9 },
    /* S10   */ { S15, S7, S10, S10, S10, S6, S10, S11, S13 },
    /* S11   */ { S15, S5, S11, S11, S11, S2, S10, S11, S3 },
    /* S12   */ { S5, S8, S12, S12, S12, S12, S12, S12, S9 },
    /* S13   */ { S9, S6, S13, S13, S13, S13, S13, S3, S13 },
    /* S14   */ { S11, S4, S14, S14, S14, S1, S14, S14, S0 },
    /* S15   */ { S11, S12, S15, S15, S15, S8, S15, S15, S9 },
};

static inline uint8_t fsm_transition(uint8_t state, uint8_t event) {
    return transition_table[state][event];
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

    // Unapred generisan niz dogadjaja, van merenog prozora sprecava
    // da -flto premesti rand()%NUM_EVENTS deljenje unutar cli()/sei() bloka, sto bi
    // moglo uticati na merenje ciklusa.
    static uint8_t precomputed_events[N];
    for (uint16_t i = 0; i < N; i++) {
        precomputed_events[i] = rand() % NUM_EVENTS;
    }

    uart_puts("Running benchmark (");
    uart_put_uint(N);
    uart_puts(" transitions)...\r\n");

    for (uint16_t i = 0; i < N; i++) {
        uint16_t c = measured_transition(precomputed_events[i]);
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

        if (event < 0 || event >= NUM_EVENTS)
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
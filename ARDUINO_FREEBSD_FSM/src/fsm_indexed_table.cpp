#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>

enum State
{
    S0, S1, S2, S3, S4, S5, S6, S7, S8, S9, S10, S11, S12, S13, S14, S15, S16,
    S17, S18, S19, S20, S21, S22, S23, S24, S25, S26, S27, S28, S29, S30, S31,
    S32, S33, S34, S35, S36, S37, S38, S39, S40, S41, S42, S43, S44, S45, S46,
    S47, S48, S49, S50, S51, S52, S53, S54
};

enum Event
{
    CLOSECONNECTION, ACK_PSH_VV1, SYN_ACK_VV0, RST_VV0, ACCEPT, FIN_ACK_VV0, LISTEN, SYN_VV0,
    RCV, ACK_RST_VV0, CLOSE, SEND, ACK_VV0 
};

#define NUM_STATES 55
#define NUM_EVENTS 13

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
    /* S0  */ { S0, S0, S0, S0, S0, S0, S1, S0, S0, S0, S2, S0, S0 },
    /* S1  */ { S1, S1, S1, S1, S4, S1, S1, S3, S1, S1, S2, S1, S1 },
    /* S2  */ { S2, S2, S2, S2, S2, S2, S2, S2, S2, S2, S2, S2, S2 },
    /* S3  */ { S3, S8, S6, S1, S9, S7, S3, S3, S3, S10, S5, S3, S8 },
    /* S4  */ { S1, S4, S4, S4, S4, S4, S4, S9, S4, S4, S2, S4, S4 },
    /* S5  */ { S5, S2, S5, S2, S5, S2, S5, S2, S5, S2, S5, S5, S2 },
    /* S6  */ { S6, S1, S6, S1, S11, S1, S6, S3, S6, S1, S5, S6, S1 },
    /* S7  */ { S7, S7, S12, S12, S13, S7, S7, S12, S7, S12, S2, S7, S7 },
    /* S8  */ { S8, S8, S12, S12, S14, S7, S8, S12, S8, S12, S2, S8, S8 },
    /* S9  */ { S3, S14, S11, S4, S9, S13, S9, S9, S9, S15, S5, S9, S14 },
    /* S10 */ { S10, S1, S1, S10, S15, S1, S10, S10, S10, S10, S2, S10, S1 },
    /* S11 */ { S6, S4, S11, S4, S11, S4, S11, S9, S11, S4, S5, S11, S4 },
    /* S12 */ { S12, S12, S12, S12, S1, S12, S12, S16, S12, S12, S2, S12, S12 },
    /* S13 */ { S18, S13, S19, S19, S13, S13, S13, S19, S13, S19, S17, S13, S13 },
    /* S14 */ { S21, S14, S19, S19, S14, S13, S14, S19, S14, S19, S20, S14, S14 },
    /* S15 */ { S10, S4, S4, S15, S15, S4, S15, S15, S15, S15, S2, S15, S4 },
    /* S16 */ { S16, S23, S22, S12, S3, S24, S16, S16, S16, S25, S5, S16, S23 },
    /* S17 */ { S26, S17, S2, S2, S17, S17, S17, S2, S17, S2, S17, S17, S17 },
    /* S18 */ { S18, S1, S1, S1, S27, S6, S18, S1, S18, S1, S26, S18, S6 },
    /* S19 */ { S1, S19, S19, S19, S19, S19, S19, S28, S19, S19, S2, S19, S19 },
    /* S20 */ { S29, S20, S2, S2, S20, S17, S20, S2, S20, S2, S20, S20, S20 },
    /* S21 */ { S21, S1, S1, S1, S30, S31, S21, S1, S21, S1, S29, S21, S21 },
    /* S22 */ { S22, S12, S22, S12, S6, S12, S22, S16, S22, S12, S5, S22, S12 },
    /* S23 */ { S23, S23, S32, S32, S8, S24, S23, S32, S23, S32, S2, S23, S23 },
    /* S24 */ { S24, S24, S32, S32, S7, S24, S24, S32, S24, S32, S2, S24, S24 },
    /* S25 */ { S25, S12, S12, S25, S10, S12, S25, S25, S25, S25, S2, S25, S12 },
    /* S26 */ { S26, S2, S2, S2, S26, S5, S26, S2, S26, S2, S26, S26, S5 },
    /* S27 */ { S18, S4, S4, S4, S27, S11, S27, S4, S27, S4, S26, S27, S11 },
    /* S28 */ { S3, S34, S36, S19, S28, S33, S28, S28, S28, S35, S5, S28, S34 },
    /* S29 */ { S29, S2, S2, S2, S29, S37, S29, S2, S29, S2, S29, S29, S29 },
    /* S30 */ { S21, S4, S4, S4, S30, S38, S30, S4, S30, S4, S29, S30, S30 },
    /* S31 */ { S31, S31, S31, S39, S38, S31, S31, S31, S31, S39, S37, S31, S31 },
    /* S32 */ { S32, S32, S32, S32, S12, S32, S32, S40, S32, S32, S2, S32, S32 },
    /* S33 */ { S7, S33, S41, S41, S33, S33, S33, S41, S33, S41, S2, S33, S33 },
    /* S34 */ { S8, S34, S41, S41, S34, S33, S34, S41, S34, S41, S2, S34, S34 },
    /* S35 */ { S10, S19, S19, S35, S35, S19, S35, S35, S35, S35, S2, S35, S19 },
    /* S36 */ { S6, S19, S36, S19, S36, S19, S36, S28, S36, S19, S5, S36, S19 },
    /* S37 */ { S37, S37, S37, S42, S37, S37, S37, S37, S37, S42, S37, S37, S37 },
    /* S38 */ { S31, S38, S38, S43, S38, S38, S38, S38, S38, S43, S37, S38, S38 },
    /* S39 */ { S39, S39, S39, S39, S43, S39, S39, S3, S39, S39, S42, S39, S39 },
    /* S40 */ { S40, S32, S44, S32, S16, S32, S40, S40, S40, S45, S5, S40, S32 },
    /* S41 */ { S12, S41, S41, S41, S41, S41, S41, S46, S41, S41, S2, S41, S41 },
    /* S42 */ { S42, S42, S42, S42, S42, S42, S42, S2, S42, S42, S42, S42, S42 },
    /* S43 */ { S39, S43, S43, S43, S43, S43, S43, S9, S43, S43, S42, S43, S43 },
    /* S44 */ { S44, S32, S44, S32, S22, S32, S44, S40, S44, S32, S5, S44, S32 },
    /* S45 */ { S45, S32, S32, S45, S25, S32, S45, S45, S45, S45, S2, S45, S32 },
    /* S46 */ { S16, S48, S49, S41, S46, S50, S46, S46, S46, S47, S5, S46, S48 },
    /* S47 */ { S25, S41, S41, S47, S47, S41, S47, S47, S47, S47, S2, S47, S41 },
    /* S48 */ { S23, S48, S51, S51, S48, S50, S48, S51, S48, S51, S2, S48, S48 },
    /* S49 */ { S22, S41, S49, S41, S49, S41, S49, S46, S49, S41, S5, S49, S41 },
    /* S50 */ { S24, S50, S51, S51, S50, S50, S50, S51, S50, S51, S2, S50, S50 },
    /* S51 */ { S32, S51, S51, S51, S51, S51, S51, S52, S51, S51, S2, S51, S51 },
    /* S52 */ { S40, S51, S53, S51, S52, S51, S52, S52, S52, S54, S5, S52, S51 },
    /* S53 */ { S44, S51, S53, S51, S53, S51, S53, S52, S53, S51, S5, S53, S51 },
    /* S54 */ { S45, S51, S51, S54, S54, S51, S54, S54, S54, S54, S2, S54, S51 }
};

static inline uint8_t fsm_transition(uint8_t state, uint8_t event) {
    return transition_table[state][event];
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
        uart_puts(" | Cycles: ");
        uart_put_uint(cycles);
        uart_puts("\r\n");
    }
}
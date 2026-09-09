#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>   // Neophodno zbog memorijskog zauzeca SRAM-a
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

typedef struct {
    uint8_t state;
    uint8_t event;
    uint8_t next_state;
    uint8_t dummy;
} Transition;

static const Transition transition_table[] PROGMEM = {
    // State S0 transitions
    { S0, CLOSECONNECTION, S0 },
    { S0, ACK_PSH_VV1    , S0 },
    { S0, SYN_ACK_VV0    , S0 },
    { S0, RST_VV0        , S0 },
    { S0, ACCEPT         , S0 },
    { S0, FIN_ACK_VV0    , S0 },
    { S0, LISTEN         , S1 },
    { S0, SYN_VV0        , S0 },
    { S0, RCV            , S0 },
    { S0, ACK_RST_VV0    , S0 },
    { S0, CLOSE          , S2 },
    { S0, SEND           , S0 },
    { S0, ACK_VV0        , S0 },

    // State S1 transitions
    { S1, CLOSECONNECTION, S1 },
    { S1, ACK_PSH_VV1    , S1 },
    { S1, SYN_ACK_VV0    , S1 },
    { S1, RST_VV0        , S1 },
    { S1, ACCEPT         , S4 },
    { S1, FIN_ACK_VV0    , S1 },
    { S1, LISTEN         , S1 },
    { S1, SYN_VV0        , S3 },
    { S1, RCV            , S1 },
    { S1, ACK_RST_VV0    , S1 },
    { S1, CLOSE          , S2 },
    { S1, SEND           , S1 },
    { S1, ACK_VV0        , S1 },

    // State S2 transitions
    { S2, CLOSECONNECTION, S2 },
    { S2, ACK_PSH_VV1    , S2 },
    { S2, SYN_ACK_VV0    , S2 },
    { S2, RST_VV0        , S2 },
    { S2, ACCEPT         , S2 },
    { S2, FIN_ACK_VV0    , S2 },
    { S2, LISTEN         , S2 },
    { S2, SYN_VV0        , S2 },
    { S2, RCV            , S2 },
    { S2, ACK_RST_VV0    , S2 },
    { S2, CLOSE          , S2 },
    { S2, SEND           , S2 },
    { S2, ACK_VV0        , S2 },

    // State S3 transitions
    { S3, CLOSECONNECTION, S3 },
    { S3, ACK_PSH_VV1    , S8 },
    { S3, SYN_ACK_VV0    , S6 },
    { S3, RST_VV0        , S1 },
    { S3, ACCEPT         , S9 },
    { S3, FIN_ACK_VV0    , S7 },
    { S3, LISTEN         , S3 },
    { S3, SYN_VV0        , S3 },
    { S3, RCV            , S3 },
    { S3, ACK_RST_VV0    , S10 },
    { S3, CLOSE          , S5 },
    { S3, SEND           , S3 },
    { S3, ACK_VV0        , S8 },

    // State S4 transitions
    { S4, CLOSECONNECTION, S1 },
    { S4, ACK_PSH_VV1    , S4 },
    { S4, SYN_ACK_VV0    , S4 },
    { S4, RST_VV0        , S4 },
    { S4, ACCEPT         , S4 },
    { S4, FIN_ACK_VV0    , S4 },
    { S4, LISTEN         , S4 },
    { S4, SYN_VV0        , S9 },
    { S4, RCV            , S4 },
    { S4, ACK_RST_VV0    , S4 },
    { S4, CLOSE          , S2 },
    { S4, SEND           , S4 },
    { S4, ACK_VV0        , S4 },

    // State S5 transitions
    { S5, CLOSECONNECTION, S5 },
    { S5, ACK_PSH_VV1    , S2 },
    { S5, SYN_ACK_VV0    , S5 },
    { S5, RST_VV0        , S2 },
    { S5, ACCEPT         , S5 },
    { S5, FIN_ACK_VV0    , S2 },
    { S5, LISTEN         , S5 },
    { S5, SYN_VV0        , S2 },
    { S5, RCV            , S5 },
    { S5, ACK_RST_VV0    , S2 },
    { S5, CLOSE          , S5 },
    { S5, SEND           , S5 },
    { S5, ACK_VV0        , S2 },

    // State S6 transitions
    { S6, CLOSECONNECTION, S6 },
    { S6, ACK_PSH_VV1    , S1 },
    { S6, SYN_ACK_VV0    , S6 },
    { S6, RST_VV0        , S1 },
    { S6, ACCEPT         , S11 },
    { S6, FIN_ACK_VV0    , S1 },
    { S6, LISTEN         , S6 },
    { S6, SYN_VV0        , S3 },
    { S6, RCV            , S6 },
    { S6, ACK_RST_VV0    , S1 },
    { S6, CLOSE          , S5 },
    { S6, SEND           , S6 },
    { S6, ACK_VV0        , S1 },

    // State S7 transitions
    { S7, CLOSECONNECTION, S7 },
    { S7, ACK_PSH_VV1    , S7 },
    { S7, SYN_ACK_VV0    , S12 },
    { S7, RST_VV0        , S12 },
    { S7, ACCEPT         , S13 },
    { S7, FIN_ACK_VV0    , S7 },
    { S7, LISTEN         , S7 },
    { S7, SYN_VV0        , S12 },
    { S7, RCV            , S7 },
    { S7, ACK_RST_VV0    , S12 },
    { S7, CLOSE          , S2 },
    { S7, SEND           , S7 },
    { S7, ACK_VV0        , S7 },

    // State S8 transitions
    { S8, CLOSECONNECTION, S8 },
    { S8, ACK_PSH_VV1    , S8 },
    { S8, SYN_ACK_VV0    , S12 },
    { S8, RST_VV0        , S12 },
    { S8, ACCEPT         , S14 },
    { S8, FIN_ACK_VV0    , S7 },
    { S8, LISTEN         , S8 },
    { S8, SYN_VV0        , S12 },
    { S8, RCV            , S8 },
    { S8, ACK_RST_VV0    , S12 },
    { S8, CLOSE          , S2 },
    { S8, SEND           , S8 },
    { S8, ACK_VV0        , S8 },

    // State S9 transitions
    { S9, CLOSECONNECTION, S3 },
    { S9, ACK_PSH_VV1    , S14 },
    { S9, SYN_ACK_VV0    , S11 },
    { S9, RST_VV0        , S4 },
    { S9, ACCEPT         , S9 },
    { S9, FIN_ACK_VV0    , S13 },
    { S9, LISTEN         , S9 },
    { S9, SYN_VV0        , S9 },
    { S9, RCV            , S9 },
    { S9, ACK_RST_VV0    , S15 },
    { S9, CLOSE          , S5 },
    { S9, SEND           , S9 },
    { S9, ACK_VV0        , S14 },

    // State S10 transitions
    { S10, CLOSECONNECTION, S10 },
    { S10, ACK_PSH_VV1    , S1 },
    { S10, SYN_ACK_VV0    , S1 },
    { S10, RST_VV0        , S10 },
    { S10, ACCEPT         , S15 },
    { S10, FIN_ACK_VV0    , S1 },
    { S10, LISTEN         , S10 },
    { S10, SYN_VV0        , S10 },
    { S10, RCV            , S10 },
    { S10, ACK_RST_VV0    , S10 },
    { S10, CLOSE          , S2 },
    { S10, SEND           , S10 },
    { S10, ACK_VV0        , S1 },

    // State S11 transitions
    { S11, CLOSECONNECTION, S6 },
    { S11, ACK_PSH_VV1    , S4 },
    { S11, SYN_ACK_VV0    , S11 },
    { S11, RST_VV0        , S4 },
    { S11, ACCEPT         , S11 },
    { S11, FIN_ACK_VV0    , S4 },
    { S11, LISTEN         , S11 },
    { S11, SYN_VV0        , S9 },
    { S11, RCV            , S11 },
    { S11, ACK_RST_VV0    , S4 },
    { S11, CLOSE          , S5 },
    { S11, SEND           , S11 },
    { S11, ACK_VV0        , S4 },

    // State S12 transitions
    { S12, CLOSECONNECTION, S12 },
    { S12, ACK_PSH_VV1    , S12 },
    { S12, SYN_ACK_VV0    , S12 },
    { S12, RST_VV0        , S12 },
    { S12, ACCEPT         , S1 },
    { S12, FIN_ACK_VV0    , S12 },
    { S12, LISTEN         , S12 },
    { S12, SYN_VV0        , S16 },
    { S12, RCV            , S12 },
    { S12, ACK_RST_VV0    , S12 },
    { S12, CLOSE          , S2 },
    { S12, SEND           , S12 },
    { S12, ACK_VV0        , S12 },

    // State S13 transitions
    { S13, CLOSECONNECTION, S18 },
    { S13, ACK_PSH_VV1    , S13 },
    { S13, SYN_ACK_VV0    , S19 },
    { S13, RST_VV0        , S19 },
    { S13, ACCEPT         , S13 },
    { S13, FIN_ACK_VV0    , S13 },
    { S13, LISTEN         , S13 },
    { S13, SYN_VV0        , S19 },
    { S13, RCV            , S13 },
    { S13, ACK_RST_VV0    , S19 },
    { S13, CLOSE          , S17 },
    { S13, SEND           , S13 },
    { S13, ACK_VV0        , S13 },

    // State S14 transitions
    { S14, CLOSECONNECTION, S21 },
    { S14, ACK_PSH_VV1    , S14 },
    { S14, SYN_ACK_VV0    , S19 },
    { S14, RST_VV0        , S19 },
    { S14, ACCEPT         , S14 },
    { S14, FIN_ACK_VV0    , S13 },
    { S14, LISTEN         , S14 },
    { S14, SYN_VV0        , S19 },
    { S14, RCV            , S14 },
    { S14, ACK_RST_VV0    , S19 },
    { S14, CLOSE          , S20 },
    { S14, SEND           , S14 },
    { S14, ACK_VV0        , S14 },

    // State S15 transitions
    { S15, CLOSECONNECTION, S10 },
    { S15, ACK_PSH_VV1    , S4 },
    { S15, SYN_ACK_VV0    , S4 },
    { S15, RST_VV0        , S15 },
    { S15, ACCEPT         , S15 },
    { S15, FIN_ACK_VV0    , S4 },
    { S15, LISTEN         , S15 },
    { S15, SYN_VV0        , S15 },
    { S15, RCV            , S15 },
    { S15, ACK_RST_VV0    , S15 },
    { S15, CLOSE          , S2 },
    { S15, SEND           , S15 },
    { S15, ACK_VV0        , S4 },

    // State S16 transitions
    { S16, CLOSECONNECTION, S16 },
    { S16, ACK_PSH_VV1    , S23 },
    { S16, SYN_ACK_VV0    , S22 },
    { S16, RST_VV0        , S12 },
    { S16, ACCEPT         , S3 },
    { S16, FIN_ACK_VV0    , S24 },
    { S16, LISTEN         , S16 },
    { S16, SYN_VV0        , S16 },
    { S16, RCV            , S16 },
    { S16, ACK_RST_VV0    , S25 },
    { S16, CLOSE          , S5 },
    { S16, SEND           , S16 },
    { S16, ACK_VV0        , S23 },

    // State S17 transitions
    { S17, CLOSECONNECTION, S26 },
    { S17, ACK_PSH_VV1    , S17 },
    { S17, SYN_ACK_VV0    , S2 },
    { S17, RST_VV0        , S2 },
    { S17, ACCEPT         , S17 },
    { S17, FIN_ACK_VV0    , S17 },
    { S17, LISTEN         , S17 },
    { S17, SYN_VV0        , S2 },
    { S17, RCV            , S17 },
    { S17, ACK_RST_VV0    , S2 },
    { S17, CLOSE          , S17 },
    { S17, SEND           , S17 },
    { S17, ACK_VV0        , S17 },

    // State S18 transitions
    { S18, CLOSECONNECTION, S18 },
    { S18, ACK_PSH_VV1    , S1 },
    { S18, SYN_ACK_VV0    , S1 },
    { S18, RST_VV0        , S1 },
    { S18, ACCEPT         , S27 },
    { S18, FIN_ACK_VV0    , S6 },
    { S18, LISTEN         , S18 },
    { S18, SYN_VV0        , S1 },
    { S18, RCV            , S18 },
    { S18, ACK_RST_VV0    , S1 },
    { S18, CLOSE          , S26 },
    { S18, SEND           , S18 },
    { S18, ACK_VV0        , S6 },

    // State S19 transitions
    { S19, CLOSECONNECTION, S1 },
    { S19, ACK_PSH_VV1    , S19 },
    { S19, SYN_ACK_VV0    , S19 },
    { S19, RST_VV0        , S19 },
    { S19, ACCEPT         , S19 },
    { S19, FIN_ACK_VV0    , S19 },
    { S19, LISTEN         , S19 },
    { S19, SYN_VV0        , S28 },
    { S19, RCV            , S19 },
    { S19, ACK_RST_VV0    , S19 },
    { S19, CLOSE          , S2 },
    { S19, SEND           , S19 },
    { S19, ACK_VV0        , S19 },

    // State S20 transitions
    { S20, CLOSECONNECTION, S29 },
    { S20, ACK_PSH_VV1    , S20 },
    { S20, SYN_ACK_VV0    , S2 },
    { S20, RST_VV0        , S2 },
    { S20, ACCEPT         , S20 },
    { S20, FIN_ACK_VV0    , S17 },
    { S20, LISTEN         , S20 },
    { S20, SYN_VV0        , S2 },
    { S20, RCV            , S20 },
    { S20, ACK_RST_VV0    , S2 },
    { S20, CLOSE          , S20 },
    { S20, SEND           , S20 },
    { S20, ACK_VV0        , S20 },

    // State S21 transitions
    { S21, CLOSECONNECTION, S21 },
    { S21, ACK_PSH_VV1    , S1 },
    { S21, SYN_ACK_VV0    , S1 },
    { S21, RST_VV0        , S1 },
    { S21, ACCEPT         , S30 },
    { S21, FIN_ACK_VV0    , S31 },
    { S21, LISTEN         , S21 },
    { S21, SYN_VV0        , S1 },
    { S21, RCV            , S21 },
    { S21, ACK_RST_VV0    , S1 },
    { S21, CLOSE          , S29 },
    { S21, SEND           , S21 },
    { S21, ACK_VV0        , S21 },

    // State S22 transitions
    { S22, CLOSECONNECTION, S22 },
    { S22, ACK_PSH_VV1    , S12 },
    { S22, SYN_ACK_VV0    , S22 },
    { S22, RST_VV0        , S12 },
    { S22, ACCEPT         , S6 },
    { S22, FIN_ACK_VV0    , S12 },
    { S22, LISTEN         , S22 },
    { S22, SYN_VV0        , S16 },
    { S22, RCV            , S22 },
    { S22, ACK_RST_VV0    , S12 },
    { S22, CLOSE          , S5 },
    { S22, SEND           , S22 },
    { S22, ACK_VV0        , S12 },

    // State S23 transitions
    { S23, CLOSECONNECTION, S23 },
    { S23, ACK_PSH_VV1    , S23 },
    { S23, SYN_ACK_VV0    , S32 },
    { S23, RST_VV0        , S32 },
    { S23, ACCEPT         , S8 },
    { S23, FIN_ACK_VV0    , S24 },
    { S23, LISTEN         , S23 },
    { S23, SYN_VV0        , S32 },
    { S23, RCV            , S23 },
    { S23, ACK_RST_VV0    , S32 },
    { S23, CLOSE          , S2 },
    { S23, SEND           , S23 },
    { S23, ACK_VV0        , S23 },

    // State S24 transitions
    { S24, CLOSECONNECTION, S24 },
    { S24, ACK_PSH_VV1    , S24 },
    { S24, SYN_ACK_VV0    , S32 },
    { S24, RST_VV0        , S32 },
    { S24, ACCEPT         , S7 },
    { S24, FIN_ACK_VV0    , S24 },
    { S24, LISTEN         , S24 },
    { S24, SYN_VV0        , S32 },
    { S24, RCV            , S24 },
    { S24, ACK_RST_VV0    , S32 },
    { S24, CLOSE          , S2 },
    { S24, SEND           , S24 },
    { S24, ACK_VV0        , S24 },

    // State S25 transitions
    { S25, CLOSECONNECTION, S25 },
    { S25, ACK_PSH_VV1    , S12 },
    { S25, SYN_ACK_VV0    , S12 },
    { S25, RST_VV0        , S25 },
    { S25, ACCEPT         , S10 },
    { S25, FIN_ACK_VV0    , S12 },
    { S25, LISTEN         , S25 },
    { S25, SYN_VV0        , S25 },
    { S25, RCV            , S25 },
    { S25, ACK_RST_VV0    , S25 },
    { S25, CLOSE          , S2 },
    { S25, SEND           , S25 },
    { S25, ACK_VV0        , S12 },

    // State S26 transitions
    { S26, CLOSECONNECTION, S26 },
    { S26, ACK_PSH_VV1    , S2 },
    { S26, SYN_ACK_VV0    , S2 },
    { S26, RST_VV0        , S2 },
    { S26, ACCEPT         , S26 },
    { S26, FIN_ACK_VV0    , S5 },
    { S26, LISTEN         , S26 },
    { S26, SYN_VV0        , S2 },
    { S26, RCV            , S26 },
    { S26, ACK_RST_VV0    , S2 },
    { S26, CLOSE          , S26 },
    { S26, SEND           , S26 },
    { S26, ACK_VV0        , S5 },

    // State S27 transitions
    { S27, CLOSECONNECTION, S18 },
    { S27, ACK_PSH_VV1    , S4 },
    { S27, SYN_ACK_VV0    , S4 },
    { S27, RST_VV0        , S4 },
    { S27, ACCEPT         , S27 },
    { S27, FIN_ACK_VV0    , S11 },
    { S27, LISTEN         , S27 },
    { S27, SYN_VV0        , S4 },
    { S27, RCV            , S27 },
    { S27, ACK_RST_VV0    , S4 },
    { S27, CLOSE          , S26 },
    { S27, SEND           , S27 },
    { S27, ACK_VV0        , S11 },

    // State S28 transitions
    { S28, CLOSECONNECTION, S3 },
    { S28, ACK_PSH_VV1    , S34 },
    { S28, SYN_ACK_VV0    , S36 },
    { S28, RST_VV0        , S19 },
    { S28, ACCEPT         , S28 },
    { S28, FIN_ACK_VV0    , S33 },
    { S28, LISTEN         , S28 },
    { S28, SYN_VV0        , S28 },
    { S28, RCV            , S28 },
    { S28, ACK_RST_VV0    , S35 },
    { S28, CLOSE          , S5 },
    { S28, SEND           , S28 },
    { S28, ACK_VV0        , S34 },

    // State S29 transitions
    { S29, CLOSECONNECTION, S29 },
    { S29, ACK_PSH_VV1    , S2 },
    { S29, SYN_ACK_VV0    , S2 },
    { S29, RST_VV0        , S2 },
    { S29, ACCEPT         , S29 },
    { S29, FIN_ACK_VV0    , S37 },
    { S29, LISTEN         , S29 },
    { S29, SYN_VV0        , S2 },
    { S29, RCV            , S29 },
    { S29, ACK_RST_VV0    , S2 },
    { S29, CLOSE          , S29 },
    { S29, SEND           , S29 },
    { S29, ACK_VV0        , S29 },

    // State S30 transitions
    { S30, CLOSECONNECTION, S21 },
    { S30, ACK_PSH_VV1    , S4 },
    { S30, SYN_ACK_VV0    , S4 },
    { S30, RST_VV0        , S4 },
    { S30, ACCEPT         , S30 },
    { S30, FIN_ACK_VV0    , S38 },
    { S30, LISTEN         , S30 },
    { S30, SYN_VV0        , S4 },
    { S30, RCV            , S30 },
    { S30, ACK_RST_VV0    , S4 },
    { S30, CLOSE          , S29 },
    { S30, SEND           , S30 },
    { S30, ACK_VV0        , S30 },

    // State S31 transitions
    { S31, CLOSECONNECTION, S31 },
    { S31, ACK_PSH_VV1    , S31 },
    { S31, SYN_ACK_VV0    , S31 },
    { S31, RST_VV0        , S39 },
    { S31, ACCEPT         , S38 },
    { S31, FIN_ACK_VV0    , S31 },
    { S31, LISTEN         , S31 },
    { S31, SYN_VV0        , S31 },
    { S31, RCV            , S31 },
    { S31, ACK_RST_VV0    , S39 },
    { S31, CLOSE          , S37 },
    { S31, SEND           , S31 },
    { S31, ACK_VV0        , S31 },

    // State S32 transitions
    { S32, CLOSECONNECTION, S32 },
    { S32, ACK_PSH_VV1    , S32 },
    { S32, SYN_ACK_VV0    , S32 },
    { S32, RST_VV0        , S32 },
    { S32, ACCEPT         , S12 },
    { S32, FIN_ACK_VV0    , S32 },
    { S32, LISTEN         , S32 },
    { S32, SYN_VV0        , S40 },
    { S32, RCV            , S32 },
    { S32, ACK_RST_VV0    , S32 },
    { S32, CLOSE          , S2 },
    { S32, SEND           , S32 },
    { S32, ACK_VV0        , S32 },

    // State S33 transitions
    { S33, CLOSECONNECTION, S7 },
    { S33, ACK_PSH_VV1    , S33 },
    { S33, SYN_ACK_VV0    , S41 },
    { S33, RST_VV0        , S41 },
    { S33, ACCEPT         , S33 },
    { S33, FIN_ACK_VV0    , S33 },
    { S33, LISTEN         , S33 },
    { S33, SYN_VV0        , S41 },
    { S33, RCV            , S33 },
    { S33, ACK_RST_VV0    , S41 },
    { S33, CLOSE          , S2 },
    { S33, SEND           , S33 },
    { S33, ACK_VV0        , S33 },

    // State S34 transitions
    { S34, CLOSECONNECTION, S8 },
    { S34, ACK_PSH_VV1    , S34 },
    { S34, SYN_ACK_VV0    , S41 },
    { S34, RST_VV0        , S41 },
    { S34, ACCEPT         , S34 },
    { S34, FIN_ACK_VV0    , S33 },
    { S34, LISTEN         , S34 },
    { S34, SYN_VV0        , S41 },
    { S34, RCV            , S34 },
    { S34, ACK_RST_VV0    , S41 },
    { S34, CLOSE          , S2 },
    { S34, SEND           , S34 },
    { S34, ACK_VV0        , S34 },

    // State S35 transitions
    { S35, CLOSECONNECTION, S10 },
    { S35, ACK_PSH_VV1    , S19 },
    { S35, SYN_ACK_VV0    , S19 },
    { S35, RST_VV0        , S35 },
    { S35, ACCEPT         , S35 },
    { S35, FIN_ACK_VV0    , S19 },
    { S35, LISTEN         , S35 },
    { S35, SYN_VV0        , S35 },
    { S35, RCV            , S35 },
    { S35, ACK_RST_VV0    , S35 },
    { S35, CLOSE          , S2 },
    { S35, SEND           , S35 },
    { S35, ACK_VV0        , S19 },

    // State S36 transitions
    { S36, CLOSECONNECTION, S6 },
    { S36, ACK_PSH_VV1    , S19 },
    { S36, SYN_ACK_VV0    , S36 },
    { S36, RST_VV0        , S19 },
    { S36, ACCEPT         , S36 },
    { S36, FIN_ACK_VV0    , S19 },
    { S36, LISTEN         , S36 },
    { S36, SYN_VV0        , S28 },
    { S36, RCV            , S36 },
    { S36, ACK_RST_VV0    , S19 },
    { S36, CLOSE          , S5 },
    { S36, SEND           , S36 },
    { S36, ACK_VV0        , S19 },

    // State S37 transitions
    { S37, CLOSECONNECTION, S37 },
    { S37, ACK_PSH_VV1    , S37 },
    { S37, SYN_ACK_VV0    , S37 },
    { S37, RST_VV0        , S42 },
    { S37, ACCEPT         , S37 },
    { S37, FIN_ACK_VV0    , S37 },
    { S37, LISTEN         , S37 },
    { S37, SYN_VV0        , S37 },
    { S37, RCV            , S37 },
    { S37, ACK_RST_VV0    , S42 },
    { S37, CLOSE          , S37 },
    { S37, SEND           , S37 },
    { S37, ACK_VV0        , S37 },

    // State S38 transitions
    { S38, CLOSECONNECTION, S31 },
    { S38, ACK_PSH_VV1    , S38 },
    { S38, SYN_ACK_VV0    , S38 },
    { S38, RST_VV0        , S43 },
    { S38, ACCEPT         , S38 },
    { S38, FIN_ACK_VV0    , S38 },
    { S38, LISTEN         , S38 },
    { S38, SYN_VV0        , S38 },
    { S38, RCV            , S38 },
    { S38, ACK_RST_VV0    , S43 },
    { S38, CLOSE          , S37 },
    { S38, SEND           , S38 },
    { S38, ACK_VV0        , S38 },

    // State S39 transitions
    { S39, CLOSECONNECTION, S39 },
    { S39, ACK_PSH_VV1    , S39 },
    { S39, SYN_ACK_VV0    , S39 },
    { S39, RST_VV0        , S39 },
    { S39, ACCEPT         , S43 },
    { S39, FIN_ACK_VV0    , S39 },
    { S39, LISTEN         , S39 },
    { S39, SYN_VV0        , S3 },
    { S39, RCV            , S39 },
    { S39, ACK_RST_VV0    , S39 },
    { S39, CLOSE          , S42 },
    { S39, SEND           , S39 },
    { S39, ACK_VV0        , S39 },

    // State S40 transitions
    { S40, CLOSECONNECTION, S40 },
    { S40, ACK_PSH_VV1    , S32 },
    { S40, SYN_ACK_VV0    , S44 },
    { S40, RST_VV0        , S32 },
    { S40, ACCEPT         , S16 },
    { S40, FIN_ACK_VV0    , S32 },
    { S40, LISTEN         , S40 },
    { S40, SYN_VV0        , S40 },
    { S40, RCV            , S40 },
    { S40, ACK_RST_VV0    , S45 },
    { S40, CLOSE          , S5 },
    { S40, SEND           , S40 },
    { S40, ACK_VV0        , S32 },

    // State S41 transitions
    { S41, CLOSECONNECTION, S12 },
    { S41, ACK_PSH_VV1    , S41 },
    { S41, SYN_ACK_VV0    , S41 },
    { S41, RST_VV0        , S41 },
    { S41, ACCEPT         , S41 },
    { S41, FIN_ACK_VV0    , S41 },
    { S41, LISTEN         , S41 },
    { S41, SYN_VV0        , S46 },
    { S41, RCV            , S41 },
    { S41, ACK_RST_VV0    , S41 },
    { S41, CLOSE          , S2 },
    { S41, SEND           , S41 },
    { S41, ACK_VV0        , S41 },

    // State S42 transitions
    { S42, CLOSECONNECTION, S42 },
    { S42, ACK_PSH_VV1    , S42 },
    { S42, SYN_ACK_VV0    , S42 },
    { S42, RST_VV0        , S42 },
    { S42, ACCEPT         , S42 },
    { S42, FIN_ACK_VV0    , S42 },
    { S42, LISTEN         , S42 },
    { S42, SYN_VV0        , S2 },
    { S42, RCV            , S42 },
    { S42, ACK_RST_VV0    , S42 },
    { S42, CLOSE          , S42 },
    { S42, SEND           , S42 },
    { S42, ACK_VV0        , S42 },

    // State S43 transitions
    { S43, CLOSECONNECTION, S39 },
    { S43, ACK_PSH_VV1    , S43 },
    { S43, SYN_ACK_VV0    , S43 },
    { S43, RST_VV0        , S43 },
    { S43, ACCEPT         , S43 },
    { S43, FIN_ACK_VV0    , S43 },
    { S43, LISTEN         , S43 },
    { S43, SYN_VV0        , S9 },
    { S43, RCV            , S43 },
    { S43, ACK_RST_VV0    , S43 },
    { S43, CLOSE          , S42 },
    { S43, SEND           , S43 },
    { S43, ACK_VV0        , S43 },

    // State S44 transitions
    { S44, CLOSECONNECTION, S44 },
    { S44, ACK_PSH_VV1    , S32 },
    { S44, SYN_ACK_VV0    , S44 },
    { S44, RST_VV0        , S32 },
    { S44, ACCEPT         , S22 },
    { S44, FIN_ACK_VV0    , S32 },
    { S44, LISTEN         , S44 },
    { S44, SYN_VV0        , S40 },
    { S44, RCV            , S44 },
    { S44, ACK_RST_VV0    , S32 },
    { S44, CLOSE          , S5 },
    { S44, SEND           , S44 },
    { S44, ACK_VV0        , S32 },

    // State S45 transitions
    { S45, CLOSECONNECTION, S45 },
    { S45, ACK_PSH_VV1    , S32 },
    { S45, SYN_ACK_VV0    , S32 },
    { S45, RST_VV0        , S45 },
    { S45, ACCEPT         , S25 },
    { S45, FIN_ACK_VV0    , S32 },
    { S45, LISTEN         , S45 },
    { S45, SYN_VV0        , S45 },
    { S45, RCV            , S45 },
    { S45, ACK_RST_VV0    , S45 },
    { S45, CLOSE          , S2 },
    { S45, SEND           , S45 },
    { S45, ACK_VV0        , S32 },

    // State S46 transitions
    { S46, CLOSECONNECTION, S16 },
    { S46, ACK_PSH_VV1    , S48 },
    { S46, SYN_ACK_VV0    , S49 },
    { S46, RST_VV0        , S41 },
    { S46, ACCEPT         , S46 },
    { S46, FIN_ACK_VV0    , S50 },
    { S46, LISTEN         , S46 },
    { S46, SYN_VV0        , S46 },
    { S46, RCV            , S46 },
    { S46, ACK_RST_VV0    , S47 },
    { S46, CLOSE          , S5 },
    { S46, SEND           , S46 },
    { S46, ACK_VV0        , S48 },

    // State S47 transitions
    { S47, CLOSECONNECTION, S25 },
    { S47, ACK_PSH_VV1    , S41 },
    { S47, SYN_ACK_VV0    , S41 },
    { S47, RST_VV0        , S47 },
    { S47, ACCEPT         , S47 },
    { S47, FIN_ACK_VV0    , S41 },
    { S47, LISTEN         , S47 },
    { S47, SYN_VV0        , S47 },
    { S47, RCV            , S47 },
    { S47, ACK_RST_VV0    , S47 },
    { S47, CLOSE          , S2 },
    { S47, SEND           , S47 },
    { S47, ACK_VV0        , S41 },

    // State S48 transitions
    { S48, CLOSECONNECTION, S23 },
    { S48, ACK_PSH_VV1    , S48 },
    { S48, SYN_ACK_VV0    , S51 },
    { S48, RST_VV0        , S51 },
    { S48, ACCEPT         , S48 },
    { S48, FIN_ACK_VV0    , S50 },
    { S48, LISTEN         , S48 },
    { S48, SYN_VV0        , S51 },
    { S48, RCV            , S48 },
    { S48, ACK_RST_VV0    , S51 },
    { S48, CLOSE          , S2 },
    { S48, SEND           , S48 },
    { S48, ACK_VV0        , S48 },

    // State S49 transitions
    { S49, CLOSECONNECTION, S22 },
    { S49, ACK_PSH_VV1    , S41 },
    { S49, SYN_ACK_VV0    , S49 },
    { S49, RST_VV0        , S41 },
    { S49, ACCEPT         , S49 },
    { S49, FIN_ACK_VV0    , S41 },
    { S49, LISTEN         , S49 },
    { S49, SYN_VV0        , S46 },
    { S49, RCV            , S49 },
    { S49, ACK_RST_VV0    , S41 },
    { S49, CLOSE          , S5 },
    { S49, SEND           , S49 },
    { S49, ACK_VV0        , S41 },

    // State S50 transitions
    { S50, CLOSECONNECTION, S24 },
    { S50, ACK_PSH_VV1    , S50 },
    { S50, SYN_ACK_VV0    , S51 },
    { S50, RST_VV0        , S51 },
    { S50, ACCEPT         , S50 },
    { S50, FIN_ACK_VV0    , S50 },
    { S50, LISTEN         , S50 },
    { S50, SYN_VV0        , S51 },
    { S50, RCV            , S50 },
    { S50, ACK_RST_VV0    , S51 },
    { S50, CLOSE          , S2 },
    { S50, SEND           , S50 },
    { S50, ACK_VV0        , S50 },

    // State S51 transitions
    { S51, CLOSECONNECTION, S32 },
    { S51, ACK_PSH_VV1    , S51 },
    { S51, SYN_ACK_VV0    , S51 },
    { S51, RST_VV0        , S51 },
    { S51, ACCEPT         , S51 },
    { S51, FIN_ACK_VV0    , S51 },
    { S51, LISTEN         , S51 },
    { S51, SYN_VV0        , S52 },
    { S51, RCV            , S51 },
    { S51, ACK_RST_VV0    , S51 },
    { S51, CLOSE          , S2 },
    { S51, SEND           , S51 },
    { S51, ACK_VV0        , S51 },

    // State S52 transitions
    { S52, CLOSECONNECTION, S40 },
    { S52, ACK_PSH_VV1    , S51 },
    { S52, SYN_ACK_VV0    , S53 },
    { S52, RST_VV0        , S51 },
    { S52, ACCEPT         , S52 },
    { S52, FIN_ACK_VV0    , S51 },
    { S52, LISTEN         , S52 },
    { S52, SYN_VV0        , S52 },
    { S52, RCV            , S52 },
    { S52, ACK_RST_VV0    , S54 },
    { S52, CLOSE          , S5 },
    { S52, SEND           , S52 },
    { S52, ACK_VV0        , S51 },

    // State S53 transitions
    { S53, CLOSECONNECTION, S44 },
    { S53, ACK_PSH_VV1    , S51 },
    { S53, SYN_ACK_VV0    , S53 },
    { S53, RST_VV0        , S51 },
    { S53, ACCEPT         , S53 },
    { S53, FIN_ACK_VV0    , S51 },
    { S53, LISTEN         , S53 },
    { S53, SYN_VV0        , S52 },
    { S53, RCV            , S53 },
    { S53, ACK_RST_VV0    , S51 },
    { S53, CLOSE          , S5 },
    { S53, SEND           , S53 },
    { S53, ACK_VV0        , S51 },

    // State S54 transitions
    { S54, CLOSECONNECTION, S45 },
    { S54, ACK_PSH_VV1    , S51 },
    { S54, SYN_ACK_VV0    , S51 },
    { S54, RST_VV0        , S54 },
    { S54, ACCEPT         , S54 },
    { S54, FIN_ACK_VV0    , S51 },
    { S54, LISTEN         , S54 },
    { S54, SYN_VV0        , S54 },
    { S54, RCV            , S54 },
    { S54, ACK_RST_VV0    , S54 },
    { S54, CLOSE          , S2 },
    { S54, SEND           , S54 },
    { S54, ACK_VV0        , S51 },
};

#define TABLE_SIZE (sizeof(transition_table) / sizeof(Transition))

static uint8_t fsm_transition(uint8_t state, uint8_t event) {
    uint8_t t_state, t_event;
    for (uint8_t i = 0; i < TABLE_SIZE; i++) {
        t_state = pgm_read_byte(&transition_table[i].state);
        t_event = pgm_read_byte(&transition_table[i].event);
        if (t_state == state && t_event == event) {
            return pgm_read_byte(&transition_table[i].next_state);
        }
    }
    return state; // no match -> stay in current state
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

static void run_benchmark(void)
{
    const uint16_t N = 1000;
    uint16_t min_c = 0xFFFF, max_c = 0;
    uint32_t sum_c = 0;

    // Unapred generisan niz dogadjaja, POTPUNO van merenog prozora - sprecava
    // da -flto premesti rand()%NUM_EVENTS deljenje unutar cli()/sei() bloka.
    static uint8_t precomputed_events[N];
    for (uint16_t i = 0; i < N; i++) {
        precomputed_events[i] = rand() % NUM_EVENTS;
    }

    uart_puts("Running benchmark (");
    uart_put_uint(N);
    uart_puts(" transitions)...\r\n");

    for (uint16_t i = 0; i < N; i++)
    {
        uint16_t c = measured_transition(precomputed_events[i]);
        if (c < min_c)
            min_c = c;
        if (c > max_c)
            max_c = c;
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
        uart_puts(" | Cycles: ");
        uart_put_uint(cycles);
        uart_puts("\r\n");
    }
}

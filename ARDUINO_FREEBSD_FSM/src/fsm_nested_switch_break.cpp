#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdint.h>
#include <stdio.h>
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
    uint8_t next_state = S0;

    switch (state)
    {
    case S0:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S0;
            break;
        case ACK_PSH_VV1:
            next_state = S0;
            break;
        case SYN_ACK_VV0:
            next_state = S0;
            break;
        case RST_VV0:
            next_state = S0;
            break;
        case ACCEPT:
            next_state = S0;
            break;
        case FIN_ACK_VV0:
            next_state = S0;
            break;
        case LISTEN:
            next_state = S1;
            break;
        case SYN_VV0:
            next_state = S0;
            break;
        case RCV:
            next_state = S0;
            break;
        case ACK_RST_VV0:
            next_state = S0;
            break;
        case CLOSE:
            next_state = S2;
            break;
        case SEND:
            next_state = S0;
            break;
        case ACK_VV0:
            next_state = S0;
            break;
        default:
            next_state = S0;
            break;
        }
        break;

    case S1:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S1;
            break;
        case ACK_PSH_VV1:
            next_state = S1;
            break;
        case SYN_ACK_VV0:
            next_state = S1;
            break;
        case RST_VV0:
            next_state = S1;
            break;
        case ACCEPT:
            next_state = S4;
            break;
        case FIN_ACK_VV0:
            next_state = S1;
            break;
        case LISTEN:
            next_state = S1;
            break;
        case SYN_VV0:
            next_state = S3;
            break;
        case RCV:
            next_state = S1;
            break;
        case ACK_RST_VV0:
            next_state = S1;
            break;
        case CLOSE:
            next_state = S2;
            break;
        case SEND:
            next_state = S1;
            break;
        case ACK_VV0:
            next_state = S1;
            break;
        default:
            next_state = S1;
            break;
        }
        break;

    case S2:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S2;
            break;
        case ACK_PSH_VV1:
            next_state = S2;
            break;
        case SYN_ACK_VV0:
            next_state = S2;
            break;
        case RST_VV0:
            next_state = S2;
            break;
        case ACCEPT:
            next_state = S2;
            break;
        case FIN_ACK_VV0:
            next_state = S2;
            break;
        case LISTEN:
            next_state = S2;
            break;
        case SYN_VV0:
            next_state = S2;
            break;
        case RCV:
            next_state = S2;
            break;
        case ACK_RST_VV0:
            next_state = S2;
            break;
        case CLOSE:
            next_state = S2;
            break;
        case SEND:
            next_state = S2;
            break;
        case ACK_VV0:
            next_state = S2;
            break;
        default:
            next_state = S2;
            break;
        }
        break;

    case S3:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S3;
            break;
        case ACK_PSH_VV1:
            next_state = S8;
            break;
        case SYN_ACK_VV0:
            next_state = S6;
            break;
        case RST_VV0:
            next_state = S1;
            break;
        case ACCEPT:
            next_state = S9;
            break;
        case FIN_ACK_VV0:
            next_state = S7;
            break;
        case LISTEN:
            next_state = S3;
            break;
        case SYN_VV0:
            next_state = S3;
            break;
        case RCV:
            next_state = S3;
            break;
        case ACK_RST_VV0:
            next_state = S10;
            break;
        case CLOSE:
            next_state = S5;
            break;
        case SEND:
            next_state = S3;
            break;
        case ACK_VV0:
            next_state = S8;
            break;
        default:
            next_state = S3;
            break;
        }
        break;

    case S4:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S1;
            break;
        case ACK_PSH_VV1:
            next_state = S4;
            break;
        case SYN_ACK_VV0:
            next_state = S4;
            break;
        case RST_VV0:
            next_state = S4;
            break;
        case ACCEPT:
            next_state = S4;
            break;
        case FIN_ACK_VV0:
            next_state = S4;
            break;
        case LISTEN:
            next_state = S4;
            break;
        case SYN_VV0:
            next_state = S9;
            break;
        case RCV:
            next_state = S4;
            break;
        case ACK_RST_VV0:
            next_state = S4;
            break;
        case CLOSE:
            next_state = S2;
            break;
        case SEND:
            next_state = S4;
            break;
        case ACK_VV0:
            next_state = S4;
            break;
        default:
            next_state = S4;
            break;
        }
        break;

    case S5:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S5;
            break;
        case ACK_PSH_VV1:
            next_state = S2;
            break;
        case SYN_ACK_VV0:
            next_state = S5;
            break;
        case RST_VV0:
            next_state = S2;
            break;
        case ACCEPT:
            next_state = S5;
            break;
        case FIN_ACK_VV0:
            next_state = S2;
            break;
        case LISTEN:
            next_state = S5;
            break;
        case SYN_VV0:
            next_state = S2;
            break;
        case RCV:
            next_state = S5;
            break;
        case ACK_RST_VV0:
            next_state = S2;
            break;
        case CLOSE:
            next_state = S5;
            break;
        case SEND:
            next_state = S5;
            break;
        case ACK_VV0:
            next_state = S2;
            break;
        default:
            next_state = S5;
            break;
        }
        break;

    case S6:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S6;
            break;
        case ACK_PSH_VV1:
            next_state = S1;
            break;
        case SYN_ACK_VV0:
            next_state = S6;
            break;
        case RST_VV0:
            next_state = S1;
            break;
        case ACCEPT:
            next_state = S11;
            break;
        case FIN_ACK_VV0:
            next_state = S1;
            break;
        case LISTEN:
            next_state = S6;
            break;
        case SYN_VV0:
            next_state = S3;
            break;
        case RCV:
            next_state = S6;
            break;
        case ACK_RST_VV0:
            next_state = S1;
            break;
        case CLOSE:
            next_state = S5;
            break;
        case SEND:
            next_state = S6;
            break;
        case ACK_VV0:
            next_state = S1;
            break;
        default:
            next_state = S6;
            break;
        }
        break;

    case S7:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S7;
            break;
        case ACK_PSH_VV1:
            next_state = S7;
            break;
        case SYN_ACK_VV0:
            next_state = S12;
            break;
        case RST_VV0:
            next_state = S12;
            break;
        case ACCEPT:
            next_state = S13;
            break;
        case FIN_ACK_VV0:
            next_state = S7;
            break;
        case LISTEN:
            next_state = S7;
            break;
        case SYN_VV0:
            next_state = S12;
            break;
        case RCV:
            next_state = S7;
            break;
        case ACK_RST_VV0:
            next_state = S12;
            break;
        case CLOSE:
            next_state = S2;
            break;
        case SEND:
            next_state = S7;
            break;
        case ACK_VV0:
            next_state = S7;
            break;
        default:
            next_state = S7;
            break;
        }
        break;

    case S8:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S8;
            break;
        case ACK_PSH_VV1:
            next_state = S8;
            break;
        case SYN_ACK_VV0:
            next_state = S12;
            break;
        case RST_VV0:
            next_state = S12;
            break;
        case ACCEPT:
            next_state = S14;
            break;
        case FIN_ACK_VV0:
            next_state = S7;
            break;
        case LISTEN:
            next_state = S8;
            break;
        case SYN_VV0:
            next_state = S12;
            break;
        case RCV:
            next_state = S8;
            break;
        case ACK_RST_VV0:
            next_state = S12;
            break;
        case CLOSE:
            next_state = S2;
            break;
        case SEND:
            next_state = S8;
            break;
        case ACK_VV0:
            next_state = S8;
            break;
        default:
            next_state = S8;
            break;
        }
        break;

    case S9:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S3;
            break;
        case ACK_PSH_VV1:
            next_state = S14;
            break;
        case SYN_ACK_VV0:
            next_state = S11;
            break;
        case RST_VV0:
            next_state = S4;
            break;
        case ACCEPT:
            next_state = S9;
            break;
        case FIN_ACK_VV0:
            next_state = S13;
            break;
        case LISTEN:
            next_state = S9;
            break;
        case SYN_VV0:
            next_state = S9;
            break;
        case RCV:
            next_state = S9;
            break;
        case ACK_RST_VV0:
            next_state = S15;
            break;
        case CLOSE:
            next_state = S5;
            break;
        case SEND:
            next_state = S9;
            break;
        case ACK_VV0:
            next_state = S14;
            break;
        default:
            next_state = S9;
            break;
        }
        break;

    case S10:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S10;
            break;
        case ACK_PSH_VV1:
            next_state = S1;
            break;
        case SYN_ACK_VV0:
            next_state = S1;
            break;
        case RST_VV0:
            next_state = S10;
            break;
        case ACCEPT:
            next_state = S15;
            break;
        case FIN_ACK_VV0:
            next_state = S1;
            break;
        case LISTEN:
            next_state = S10;
            break;
        case SYN_VV0:
            next_state = S10;
            break;
        case RCV:
            next_state = S10;
            break;
        case ACK_RST_VV0:
            next_state = S10;
            break;
        case CLOSE:
            next_state = S2;
            break;
        case SEND:
            next_state = S10;
            break;
        case ACK_VV0:
            next_state = S1;
            break;
        default:
            next_state = S10;
            break;
        }
        break;

    case S11:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S6;
            break;
        case ACK_PSH_VV1:
            next_state = S4;
            break;
        case SYN_ACK_VV0:
            next_state = S11;
            break;
        case RST_VV0:
            next_state = S4;
            break;
        case ACCEPT:
            next_state = S11;
            break;
        case FIN_ACK_VV0:
            next_state = S4;
            break;
        case LISTEN:
            next_state = S11;
            break;
        case SYN_VV0:
            next_state = S9;
            break;
        case RCV:
            next_state = S11;
            break;
        case ACK_RST_VV0:
            next_state = S4;
            break;
        case CLOSE:
            next_state = S5;
            break;
        case SEND:
            next_state = S11;
            break;
        case ACK_VV0:
            next_state = S4;
            break;
        default:
            next_state = S11;
            break;
        }
        break;

    case S12:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S12;
            break;
        case ACK_PSH_VV1:
            next_state = S12;
            break;
        case SYN_ACK_VV0:
            next_state = S12;
            break;
        case RST_VV0:
            next_state = S12;
            break;
        case ACCEPT:
            next_state = S1;
            break;
        case FIN_ACK_VV0:
            next_state = S12;
            break;
        case LISTEN:
            next_state = S12;
            break;
        case SYN_VV0:
            next_state = S16;
            break;
        case RCV:
            next_state = S12;
            break;
        case ACK_RST_VV0:
            next_state = S12;
            break;
        case CLOSE:
            next_state = S2;
            break;
        case SEND:
            next_state = S12;
            break;
        case ACK_VV0:
            next_state = S12;
            break;
        default:
            next_state = S12;
            break;
        }
        break;

    case S13:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S18;
            break;
        case ACK_PSH_VV1:
            next_state = S13;
            break;
        case SYN_ACK_VV0:
            next_state = S19;
            break;
        case RST_VV0:
            next_state = S19;
            break;
        case ACCEPT:
            next_state = S13;
            break;
        case FIN_ACK_VV0:
            next_state = S13;
            break;
        case LISTEN:
            next_state = S13;
            break;
        case SYN_VV0:
            next_state = S19;
            break;
        case RCV:
            next_state = S13;
            break;
        case ACK_RST_VV0:
            next_state = S19;
            break;
        case CLOSE:
            next_state = S17;
            break;
        case SEND:
            next_state = S13;
            break;
        case ACK_VV0:
            next_state = S13;
            break;
        default:
            next_state = S13;
            break;
        }
        break;

    case S14:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S21;
            break;
        case ACK_PSH_VV1:
            next_state = S14;
            break;
        case SYN_ACK_VV0:
            next_state = S19;
            break;
        case RST_VV0:
            next_state = S19;
            break;
        case ACCEPT:
            next_state = S14;
            break;
        case FIN_ACK_VV0:
            next_state = S13;
            break;
        case LISTEN:
            next_state = S14;
            break;
        case SYN_VV0:
            next_state = S19;
            break;
        case RCV:
            next_state = S14;
            break;
        case ACK_RST_VV0:
            next_state = S19;
            break;
        case CLOSE:
            next_state = S20;
            break;
        case SEND:
            next_state = S14;
            break;
        case ACK_VV0:
            next_state = S14;
            break;
        default:
            next_state = S14;
            break;
        }
        break;

    case S15:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S10;
            break;
        case ACK_PSH_VV1:
            next_state = S4;
            break;
        case SYN_ACK_VV0:
            next_state = S4;
            break;
        case RST_VV0:
            next_state = S15;
            break;
        case ACCEPT:
            next_state = S15;
            break;
        case FIN_ACK_VV0:
            next_state = S4;
            break;
        case LISTEN:
            next_state = S15;
            break;
        case SYN_VV0:
            next_state = S15;
            break;
        case RCV:
            next_state = S15;
            break;
        case ACK_RST_VV0:
            next_state = S15;
            break;
        case CLOSE:
            next_state = S2;
            break;
        case SEND:
            next_state = S15;
            break;
        case ACK_VV0:
            next_state = S4;
            break;
        default:
            next_state = S15;
            break;
        }
        break;

    case S16:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S16;
            break;
        case ACK_PSH_VV1:
            next_state = S23;
            break;
        case SYN_ACK_VV0:
            next_state = S22;
            break;
        case RST_VV0:
            next_state = S12;
            break;
        case ACCEPT:
            next_state = S3;
            break;
        case FIN_ACK_VV0:
            next_state = S24;
            break;
        case LISTEN:
            next_state = S16;
            break;
        case SYN_VV0:
            next_state = S16;
            break;
        case RCV:
            next_state = S16;
            break;
        case ACK_RST_VV0:
            next_state = S25;
            break;
        case CLOSE:
            next_state = S5;
            break;
        case SEND:
            next_state = S16;
            break;
        case ACK_VV0:
            next_state = S23;
            break;
        default:
            next_state = S16;
            break;
        }
        break;

    case S17:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S26;
            break;
        case ACK_PSH_VV1:
            next_state = S17;
            break;
        case SYN_ACK_VV0:
            next_state = S2;
            break;
        case RST_VV0:
            next_state = S2;
            break;
        case ACCEPT:
            next_state = S17;
            break;
        case FIN_ACK_VV0:
            next_state = S17;
            break;
        case LISTEN:
            next_state = S17;
            break;
        case SYN_VV0:
            next_state = S2;
            break;
        case RCV:
            next_state = S17;
            break;
        case ACK_RST_VV0:
            next_state = S2;
            break;
        case CLOSE:
            next_state = S17;
            break;
        case SEND:
            next_state = S17;
            break;
        case ACK_VV0:
            next_state = S17;
            break;
        default:
            next_state = S17;
            break;
        }
        break;

    case S18:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S18;
            break;
        case ACK_PSH_VV1:
            next_state = S1;
            break;
        case SYN_ACK_VV0:
            next_state = S1;
            break;
        case RST_VV0:
            next_state = S1;
            break;
        case ACCEPT:
            next_state = S27;
            break;
        case FIN_ACK_VV0:
            next_state = S6;
            break;
        case LISTEN:
            next_state = S18;
            break;
        case SYN_VV0:
            next_state = S1;
            break;
        case RCV:
            next_state = S18;
            break;
        case ACK_RST_VV0:
            next_state = S1;
            break;
        case CLOSE:
            next_state = S26;
            break;
        case SEND:
            next_state = S18;
            break;
        case ACK_VV0:
            next_state = S6;
            break;
        default:
            next_state = S18;
            break;
        }
        break;

    case S19:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S1;
            break;
        case ACK_PSH_VV1:
            next_state = S19;
            break;
        case SYN_ACK_VV0:
            next_state = S19;
            break;
        case RST_VV0:
            next_state = S19;
            break;
        case ACCEPT:
            next_state = S19;
            break;
        case FIN_ACK_VV0:
            next_state = S19;
            break;
        case LISTEN:
            next_state = S19;
            break;
        case SYN_VV0:
            next_state = S28;
            break;
        case RCV:
            next_state = S19;
            break;
        case ACK_RST_VV0:
            next_state = S19;
            break;
        case CLOSE:
            next_state = S2;
            break;
        case SEND:
            next_state = S19;
            break;
        case ACK_VV0:
            next_state = S19;
            break;
        default:
            next_state = S19;
            break;
        }
        break;

    case S20:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S29;
            break;
        case ACK_PSH_VV1:
            next_state = S20;
            break;
        case SYN_ACK_VV0:
            next_state = S2;
            break;
        case RST_VV0:
            next_state = S2;
            break;
        case ACCEPT:
            next_state = S20;
            break;
        case FIN_ACK_VV0:
            next_state = S17;
            break;
        case LISTEN:
            next_state = S20;
            break;
        case SYN_VV0:
            next_state = S2;
            break;
        case RCV:
            next_state = S20;
            break;
        case ACK_RST_VV0:
            next_state = S2;
            break;
        case CLOSE:
            next_state = S20;
            break;
        case SEND:
            next_state = S20;
            break;
        case ACK_VV0:
            next_state = S20;
            break;
        default:
            next_state = S20;
            break;
        }
        break;

    case S21:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S21;
            break;
        case ACK_PSH_VV1:
            next_state = S1;
            break;
        case SYN_ACK_VV0:
            next_state = S1;
            break;
        case RST_VV0:
            next_state = S1;
            break;
        case ACCEPT:
            next_state = S30;
            break;
        case FIN_ACK_VV0:
            next_state = S31;
            break;
        case LISTEN:
            next_state = S21;
            break;
        case SYN_VV0:
            next_state = S1;
            break;
        case RCV:
            next_state = S21;
            break;
        case ACK_RST_VV0:
            next_state = S1;
            break;
        case CLOSE:
            next_state = S29;
            break;
        case SEND:
            next_state = S21;
            break;
        case ACK_VV0:
            next_state = S21;
            break;
        default:
            next_state = S21;
            break;
        }
        break;

    case S22:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S22;
            break;
        case ACK_PSH_VV1:
            next_state = S12;
            break;
        case SYN_ACK_VV0:
            next_state = S22;
            break;
        case RST_VV0:
            next_state = S12;
            break;
        case ACCEPT:
            next_state = S6;
            break;
        case FIN_ACK_VV0:
            next_state = S12;
            break;
        case LISTEN:
            next_state = S22;
            break;
        case SYN_VV0:
            next_state = S16;
            break;
        case RCV:
            next_state = S22;
            break;
        case ACK_RST_VV0:
            next_state = S12;
            break;
        case CLOSE:
            next_state = S5;
            break;
        case SEND:
            next_state = S22;
            break;
        case ACK_VV0:
            next_state = S12;
            break;
        default:
            next_state = S22;
            break;
        }
        break;

    case S23:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S23;
            break;
        case ACK_PSH_VV1:
            next_state = S23;
            break;
        case SYN_ACK_VV0:
            next_state = S32;
            break;
        case RST_VV0:
            next_state = S32;
            break;
        case ACCEPT:
            next_state = S8;
            break;
        case FIN_ACK_VV0:
            next_state = S24;
            break;
        case LISTEN:
            next_state = S23;
            break;
        case SYN_VV0:
            next_state = S32;
            break;
        case RCV:
            next_state = S23;
            break;
        case ACK_RST_VV0:
            next_state = S32;
            break;
        case CLOSE:
            next_state = S2;
            break;
        case SEND:
            next_state = S23;
            break;
        case ACK_VV0:
            next_state = S23;
            break;
        default:
            next_state = S23;
            break;
        }
        break;

    case S24:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S24;
            break;
        case ACK_PSH_VV1:
            next_state = S24;
            break;
        case SYN_ACK_VV0:
            next_state = S32;
            break;
        case RST_VV0:
            next_state = S32;
            break;
        case ACCEPT:
            next_state = S7;
            break;
        case FIN_ACK_VV0:
            next_state = S24;
            break;
        case LISTEN:
            next_state = S24;
            break;
        case SYN_VV0:
            next_state = S32;
            break;
        case RCV:
            next_state = S24;
            break;
        case ACK_RST_VV0:
            next_state = S32;
            break;
        case CLOSE:
            next_state = S2;
            break;
        case SEND:
            next_state = S24;
            break;
        case ACK_VV0:
            next_state = S24;
            break;
        default:
            next_state = S24;
            break;
        }
        break;

    case S25:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S25;
            break;
        case ACK_PSH_VV1:
            next_state = S12;
            break;
        case SYN_ACK_VV0:
            next_state = S12;
            break;
        case RST_VV0:
            next_state = S25;
            break;
        case ACCEPT:
            next_state = S10;
            break;
        case FIN_ACK_VV0:
            next_state = S12;
            break;
        case LISTEN:
            next_state = S25;
            break;
        case SYN_VV0:
            next_state = S25;
            break;
        case RCV:
            next_state = S25;
            break;
        case ACK_RST_VV0:
            next_state = S25;
            break;
        case CLOSE:
            next_state = S2;
            break;
        case SEND:
            next_state = S25;
            break;
        case ACK_VV0:
            next_state = S12;
            break;
        default:
            next_state = S25;
            break;
        }
        break;

    case S26:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S26;
            break;
        case ACK_PSH_VV1:
            next_state = S2;
            break;
        case SYN_ACK_VV0:
            next_state = S2;
            break;
        case RST_VV0:
            next_state = S2;
            break;
        case ACCEPT:
            next_state = S26;
            break;
        case FIN_ACK_VV0:
            next_state = S5;
            break;
        case LISTEN:
            next_state = S26;
            break;
        case SYN_VV0:
            next_state = S2;
            break;
        case RCV:
            next_state = S26;
            break;
        case ACK_RST_VV0:
            next_state = S2;
            break;
        case CLOSE:
            next_state = S26;
            break;
        case SEND:
            next_state = S26;
            break;
        case ACK_VV0:
            next_state = S5;
            break;
        default:
            next_state = S26;
            break;
        }
        break;

    case S27:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S18;
            break;
        case ACK_PSH_VV1:
            next_state = S4;
            break;
        case SYN_ACK_VV0:
            next_state = S4;
            break;
        case RST_VV0:
            next_state = S4;
            break;
        case ACCEPT:
            next_state = S27;
            break;
        case FIN_ACK_VV0:
            next_state = S11;
            break;
        case LISTEN:
            next_state = S27;
            break;
        case SYN_VV0:
            next_state = S4;
            break;
        case RCV:
            next_state = S27;
            break;
        case ACK_RST_VV0:
            next_state = S4;
            break;
        case CLOSE:
            next_state = S26;
            break;
        case SEND:
            next_state = S27;
            break;
        case ACK_VV0:
            next_state = S11;
            break;
        default:
            next_state = S27;
            break;
        }
        break;

    case S28:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S3;
            break;
        case ACK_PSH_VV1:
            next_state = S34;
            break;
        case SYN_ACK_VV0:
            next_state = S36;
            break;
        case RST_VV0:
            next_state = S19;
            break;
        case ACCEPT:
            next_state = S28;
            break;
        case FIN_ACK_VV0:
            next_state = S33;
            break;
        case LISTEN:
            next_state = S28;
            break;
        case SYN_VV0:
            next_state = S28;
            break;
        case RCV:
            next_state = S28;
            break;
        case ACK_RST_VV0:
            next_state = S35;
            break;
        case CLOSE:
            next_state = S5;
            break;
        case SEND:
            next_state = S28;
            break;
        case ACK_VV0:
            next_state = S34;
            break;
        default:
            next_state = S28;
            break;
        }
        break;

    case S29:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S29;
            break;
        case ACK_PSH_VV1:
            next_state = S2;
            break;
        case SYN_ACK_VV0:
            next_state = S2;
            break;
        case RST_VV0:
            next_state = S2;
            break;
        case ACCEPT:
            next_state = S29;
            break;
        case FIN_ACK_VV0:
            next_state = S37;
            break;
        case LISTEN:
            next_state = S29;
            break;
        case SYN_VV0:
            next_state = S2;
            break;
        case RCV:
            next_state = S29;
            break;
        case ACK_RST_VV0:
            next_state = S2;
            break;
        case CLOSE:
            next_state = S29;
            break;
        case SEND:
            next_state = S29;
            break;
        case ACK_VV0:
            next_state = S29;
            break;
        default:
            next_state = S29;
            break;
        }
        break;

    case S30:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S21;
            break;
        case ACK_PSH_VV1:
            next_state = S4;
            break;
        case SYN_ACK_VV0:
            next_state = S4;
            break;
        case RST_VV0:
            next_state = S4;
            break;
        case ACCEPT:
            next_state = S30;
            break;
        case FIN_ACK_VV0:
            next_state = S38;
            break;
        case LISTEN:
            next_state = S30;
            break;
        case SYN_VV0:
            next_state = S4;
            break;
        case RCV:
            next_state = S30;
            break;
        case ACK_RST_VV0:
            next_state = S4;
            break;
        case CLOSE:
            next_state = S29;
            break;
        case SEND:
            next_state = S30;
            break;
        case ACK_VV0:
            next_state = S30;
            break;
        default:
            next_state = S30;
            break;
        }
        break;

    case S31:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S31;
            break;
        case ACK_PSH_VV1:
            next_state = S31;
            break;
        case SYN_ACK_VV0:
            next_state = S31;
            break;
        case RST_VV0:
            next_state = S39;
            break;
        case ACCEPT:
            next_state = S38;
            break;
        case FIN_ACK_VV0:
            next_state = S31;
            break;
        case LISTEN:
            next_state = S31;
            break;
        case SYN_VV0:
            next_state = S31;
            break;
        case RCV:
            next_state = S31;
            break;
        case ACK_RST_VV0:
            next_state = S39;
            break;
        case CLOSE:
            next_state = S37;
            break;
        case SEND:
            next_state = S31;
            break;
        case ACK_VV0:
            next_state = S31;
            break;
        default:
            next_state = S31;
            break;
        }
        break;

    case S32:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S32;
            break;
        case ACK_PSH_VV1:
            next_state = S32;
            break;
        case SYN_ACK_VV0:
            next_state = S32;
            break;
        case RST_VV0:
            next_state = S32;
            break;
        case ACCEPT:
            next_state = S12;
            break;
        case FIN_ACK_VV0:
            next_state = S32;
            break;
        case LISTEN:
            next_state = S32;
            break;
        case SYN_VV0:
            next_state = S40;
            break;
        case RCV:
            next_state = S32;
            break;
        case ACK_RST_VV0:
            next_state = S32;
            break;
        case CLOSE:
            next_state = S2;
            break;
        case SEND:
            next_state = S32;
            break;
        case ACK_VV0:
            next_state = S32;
            break;
        default:
            next_state = S32;
            break;
        }
        break;

    case S33:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S7;
            break;
        case ACK_PSH_VV1:
            next_state = S33;
            break;
        case SYN_ACK_VV0:
            next_state = S41;
            break;
        case RST_VV0:
            next_state = S41;
            break;
        case ACCEPT:
            next_state = S33;
            break;
        case FIN_ACK_VV0:
            next_state = S33;
            break;
        case LISTEN:
            next_state = S33;
            break;
        case SYN_VV0:
            next_state = S41;
            break;
        case RCV:
            next_state = S33;
            break;
        case ACK_RST_VV0:
            next_state = S41;
            break;
        case CLOSE:
            next_state = S2;
            break;
        case SEND:
            next_state = S33;
            break;
        case ACK_VV0:
            next_state = S33;
            break;
        default:
            next_state = S33;
            break;
        }
        break;

    case S34:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S8;
            break;
        case ACK_PSH_VV1:
            next_state = S34;
            break;
        case SYN_ACK_VV0:
            next_state = S41;
            break;
        case RST_VV0:
            next_state = S41;
            break;
        case ACCEPT:
            next_state = S34;
            break;
        case FIN_ACK_VV0:
            next_state = S33;
            break;
        case LISTEN:
            next_state = S34;
            break;
        case SYN_VV0:
            next_state = S41;
            break;
        case RCV:
            next_state = S34;
            break;
        case ACK_RST_VV0:
            next_state = S41;
            break;
        case CLOSE:
            next_state = S2;
            break;
        case SEND:
            next_state = S34;
            break;
        case ACK_VV0:
            next_state = S34;
            break;
        default:
            next_state = S34;
            break;
        }
        break;

    case S35:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S10;
            break;
        case ACK_PSH_VV1:
            next_state = S19;
            break;
        case SYN_ACK_VV0:
            next_state = S19;
            break;
        case RST_VV0:
            next_state = S35;
            break;
        case ACCEPT:
            next_state = S35;
            break;
        case FIN_ACK_VV0:
            next_state = S19;
            break;
        case LISTEN:
            next_state = S35;
            break;
        case SYN_VV0:
            next_state = S35;
            break;
        case RCV:
            next_state = S35;
            break;
        case ACK_RST_VV0:
            next_state = S35;
            break;
        case CLOSE:
            next_state = S2;
            break;
        case SEND:
            next_state = S35;
            break;
        case ACK_VV0:
            next_state = S19;
            break;
        default:
            next_state = S35;
            break;
        }
        break;

    case S36:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S6;
            break;
        case ACK_PSH_VV1:
            next_state = S19;
            break;
        case SYN_ACK_VV0:
            next_state = S36;
            break;
        case RST_VV0:
            next_state = S19;
            break;
        case ACCEPT:
            next_state = S36;
            break;
        case FIN_ACK_VV0:
            next_state = S19;
            break;
        case LISTEN:
            next_state = S36;
            break;
        case SYN_VV0:
            next_state = S28;
            break;
        case RCV:
            next_state = S36;
            break;
        case ACK_RST_VV0:
            next_state = S19;
            break;
        case CLOSE:
            next_state = S5;
            break;
        case SEND:
            next_state = S36;
            break;
        case ACK_VV0:
            next_state = S19;
            break;
        default:
            next_state = S36;
            break;
        }
        break;

    case S37:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S37;
            break;
        case ACK_PSH_VV1:
            next_state = S37;
            break;
        case SYN_ACK_VV0:
            next_state = S37;
            break;
        case RST_VV0:
            next_state = S42;
            break;
        case ACCEPT:
            next_state = S37;
            break;
        case FIN_ACK_VV0:
            next_state = S37;
            break;
        case LISTEN:
            next_state = S37;
            break;
        case SYN_VV0:
            next_state = S37;
            break;
        case RCV:
            next_state = S37;
            break;
        case ACK_RST_VV0:
            next_state = S42;
            break;
        case CLOSE:
            next_state = S37;
            break;
        case SEND:
            next_state = S37;
            break;
        case ACK_VV0:
            next_state = S37;
            break;
        default:
            next_state = S37;
            break;
        }
        break;

    case S38:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S31;
            break;
        case ACK_PSH_VV1:
            next_state = S38;
            break;
        case SYN_ACK_VV0:
            next_state = S38;
            break;
        case RST_VV0:
            next_state = S43;
            break;
        case ACCEPT:
            next_state = S38;
            break;
        case FIN_ACK_VV0:
            next_state = S38;
            break;
        case LISTEN:
            next_state = S38;
            break;
        case SYN_VV0:
            next_state = S38;
            break;
        case RCV:
            next_state = S38;
            break;
        case ACK_RST_VV0:
            next_state = S43;
            break;
        case CLOSE:
            next_state = S37;
            break;
        case SEND:
            next_state = S38;
            break;
        case ACK_VV0:
            next_state = S38;
            break;
        default:
            next_state = S38;
            break;
        }
        break;

    case S39:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S39;
            break;
        case ACK_PSH_VV1:
            next_state = S39;
            break;
        case SYN_ACK_VV0:
            next_state = S39;
            break;
        case RST_VV0:
            next_state = S39;
            break;
        case ACCEPT:
            next_state = S43;
            break;
        case FIN_ACK_VV0:
            next_state = S39;
            break;
        case LISTEN:
            next_state = S39;
            break;
        case SYN_VV0:
            next_state = S3;
            break;
        case RCV:
            next_state = S39;
            break;
        case ACK_RST_VV0:
            next_state = S39;
            break;
        case CLOSE:
            next_state = S42;
            break;
        case SEND:
            next_state = S39;
            break;
        case ACK_VV0:
            next_state = S39;
            break;
        default:
            next_state = S39;
            break;
        }
        break;

    case S40:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S40;
            break;
        case ACK_PSH_VV1:
            next_state = S32;
            break;
        case SYN_ACK_VV0:
            next_state = S44;
            break;
        case RST_VV0:
            next_state = S32;
            break;
        case ACCEPT:
            next_state = S16;
            break;
        case FIN_ACK_VV0:
            next_state = S32;
            break;
        case LISTEN:
            next_state = S40;
            break;
        case SYN_VV0:
            next_state = S40;
            break;
        case RCV:
            next_state = S40;
            break;
        case ACK_RST_VV0:
            next_state = S45;
            break;
        case CLOSE:
            next_state = S5;
            break;
        case SEND:
            next_state = S40;
            break;
        case ACK_VV0:
            next_state = S32;
            break;
        default:
            next_state = S40;
            break;
        }
        break;

    case S41:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S12;
            break;
        case ACK_PSH_VV1:
            next_state = S41;
            break;
        case SYN_ACK_VV0:
            next_state = S41;
            break;
        case RST_VV0:
            next_state = S41;
            break;
        case ACCEPT:
            next_state = S41;
            break;
        case FIN_ACK_VV0:
            next_state = S41;
            break;
        case LISTEN:
            next_state = S41;
            break;
        case SYN_VV0:
            next_state = S46;
            break;
        case RCV:
            next_state = S41;
            break;
        case ACK_RST_VV0:
            next_state = S41;
            break;
        case CLOSE:
            next_state = S2;
            break;
        case SEND:
            next_state = S41;
            break;
        case ACK_VV0:
            next_state = S41;
            break;
        default:
            next_state = S41;
            break;
        }
        break;

    case S42:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S42;
            break;
        case ACK_PSH_VV1:
            next_state = S42;
            break;
        case SYN_ACK_VV0:
            next_state = S42;
            break;
        case RST_VV0:
            next_state = S42;
            break;
        case ACCEPT:
            next_state = S42;
            break;
        case FIN_ACK_VV0:
            next_state = S42;
            break;
        case LISTEN:
            next_state = S42;
            break;
        case SYN_VV0:
            next_state = S2;
            break;
        case RCV:
            next_state = S42;
            break;
        case ACK_RST_VV0:
            next_state = S42;
            break;
        case CLOSE:
            next_state = S42;
            break;
        case SEND:
            next_state = S42;
            break;
        case ACK_VV0:
            next_state = S42;
            break;
        default:
            next_state = S42;
            break;
        }
        break;

    case S43:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S39;
            break;
        case ACK_PSH_VV1:
            next_state = S43;
            break;
        case SYN_ACK_VV0:
            next_state = S43;
            break;
        case RST_VV0:
            next_state = S43;
            break;
        case ACCEPT:
            next_state = S43;
            break;
        case FIN_ACK_VV0:
            next_state = S43;
            break;
        case LISTEN:
            next_state = S43;
            break;
        case SYN_VV0:
            next_state = S9;
            break;
        case RCV:
            next_state = S43;
            break;
        case ACK_RST_VV0:
            next_state = S43;
            break;
        case CLOSE:
            next_state = S42;
            break;
        case SEND:
            next_state = S43;
            break;
        case ACK_VV0:
            next_state = S43;
            break;
        default:
            next_state = S43;
            break;
        }
        break;

    case S44:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S44;
            break;
        case ACK_PSH_VV1:
            next_state = S32;
            break;
        case SYN_ACK_VV0:
            next_state = S44;
            break;
        case RST_VV0:
            next_state = S32;
            break;
        case ACCEPT:
            next_state = S22;
            break;
        case FIN_ACK_VV0:
            next_state = S32;
            break;
        case LISTEN:
            next_state = S44;
            break;
        case SYN_VV0:
            next_state = S40;
            break;
        case RCV:
            next_state = S44;
            break;
        case ACK_RST_VV0:
            next_state = S32;
            break;
        case CLOSE:
            next_state = S5;
            break;
        case SEND:
            next_state = S44;
            break;
        case ACK_VV0:
            next_state = S32;
            break;
        default:
            next_state = S44;
            break;
        }
        break;

    case S45:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S45;
            break;
        case ACK_PSH_VV1:
            next_state = S32;
            break;
        case SYN_ACK_VV0:
            next_state = S32;
            break;
        case RST_VV0:
            next_state = S45;
            break;
        case ACCEPT:
            next_state = S25;
            break;
        case FIN_ACK_VV0:
            next_state = S32;
            break;
        case LISTEN:
            next_state = S45;
            break;
        case SYN_VV0:
            next_state = S45;
            break;
        case RCV:
            next_state = S45;
            break;
        case ACK_RST_VV0:
            next_state = S45;
            break;
        case CLOSE:
            next_state = S2;
            break;
        case SEND:
            next_state = S45;
            break;
        case ACK_VV0:
            next_state = S32;
            break;
        default:
            next_state = S45;
            break;
        }
        break;

    case S46:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S16;
            break;
        case ACK_PSH_VV1:
            next_state = S48;
            break;
        case SYN_ACK_VV0:
            next_state = S49;
            break;
        case RST_VV0:
            next_state = S41;
            break;
        case ACCEPT:
            next_state = S46;
            break;
        case FIN_ACK_VV0:
            next_state = S50;
            break;
        case LISTEN:
            next_state = S46;
            break;
        case SYN_VV0:
            next_state = S46;
            break;
        case RCV:
            next_state = S46;
            break;
        case ACK_RST_VV0:
            next_state = S47;
            break;
        case CLOSE:
            next_state = S5;
            break;
        case SEND:
            next_state = S46;
            break;
        case ACK_VV0:
            next_state = S48;
            break;
        default:
            next_state = S46;
            break;
        }
        break;

    case S47:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S25;
            break;
        case ACK_PSH_VV1:
            next_state = S41;
            break;
        case SYN_ACK_VV0:
            next_state = S41;
            break;
        case RST_VV0:
            next_state = S47;
            break;
        case ACCEPT:
            next_state = S47;
            break;
        case FIN_ACK_VV0:
            next_state = S41;
            break;
        case LISTEN:
            next_state = S47;
            break;
        case SYN_VV0:
            next_state = S47;
            break;
        case RCV:
            next_state = S47;
            break;
        case ACK_RST_VV0:
            next_state = S47;
            break;
        case CLOSE:
            next_state = S2;
            break;
        case SEND:
            next_state = S47;
            break;
        case ACK_VV0:
            next_state = S41;
            break;
        default:
            next_state = S47;
            break;
        }
        break;

    case S48:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S23;
            break;
        case ACK_PSH_VV1:
            next_state = S48;
            break;
        case SYN_ACK_VV0:
            next_state = S51;
            break;
        case RST_VV0:
            next_state = S51;
            break;
        case ACCEPT:
            next_state = S48;
            break;
        case FIN_ACK_VV0:
            next_state = S50;
            break;
        case LISTEN:
            next_state = S48;
            break;
        case SYN_VV0:
            next_state = S51;
            break;
        case RCV:
            next_state = S48;
            break;
        case ACK_RST_VV0:
            next_state = S51;
            break;
        case CLOSE:
            next_state = S2;
            break;
        case SEND:
            next_state = S48;
            break;
        case ACK_VV0:
            next_state = S48;
            break;
        default:
            next_state = S48;
            break;
        }
        break;

    case S49:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S22;
            break;
        case ACK_PSH_VV1:
            next_state = S41;
            break;
        case SYN_ACK_VV0:
            next_state = S49;
            break;
        case RST_VV0:
            next_state = S41;
            break;
        case ACCEPT:
            next_state = S49;
            break;
        case FIN_ACK_VV0:
            next_state = S41;
            break;
        case LISTEN:
            next_state = S49;
            break;
        case SYN_VV0:
            next_state = S46;
            break;
        case RCV:
            next_state = S49;
            break;
        case ACK_RST_VV0:
            next_state = S41;
            break;
        case CLOSE:
            next_state = S5;
            break;
        case SEND:
            next_state = S49;
            break;
        case ACK_VV0:
            next_state = S41;
            break;
        default:
            next_state = S49;
            break;
        }
        break;

    case S50:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S24;
            break;
        case ACK_PSH_VV1:
            next_state = S50;
            break;
        case SYN_ACK_VV0:
            next_state = S51;
            break;
        case RST_VV0:
            next_state = S51;
            break;
        case ACCEPT:
            next_state = S50;
            break;
        case FIN_ACK_VV0:
            next_state = S50;
            break;
        case LISTEN:
            next_state = S50;
            break;
        case SYN_VV0:
            next_state = S51;
            break;
        case RCV:
            next_state = S50;
            break;
        case ACK_RST_VV0:
            next_state = S51;
            break;
        case CLOSE:
            next_state = S2;
            break;
        case SEND:
            next_state = S50;
            break;
        case ACK_VV0:
            next_state = S50;
            break;
        default:
            next_state = S50;
            break;
        }
        break;

    case S51:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S32;
            break;
        case ACK_PSH_VV1:
            next_state = S51;
            break;
        case SYN_ACK_VV0:
            next_state = S51;
            break;
        case RST_VV0:
            next_state = S51;
            break;
        case ACCEPT:
            next_state = S51;
            break;
        case FIN_ACK_VV0:
            next_state = S51;
            break;
        case LISTEN:
            next_state = S51;
            break;
        case SYN_VV0:
            next_state = S52;
            break;
        case RCV:
            next_state = S51;
            break;
        case ACK_RST_VV0:
            next_state = S51;
            break;
        case CLOSE:
            next_state = S2;
            break;
        case SEND:
            next_state = S51;
            break;
        case ACK_VV0:
            next_state = S51;
            break;
        default:
            next_state = S51;
            break;
        }
        break;

    case S52:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S40;
            break;
        case ACK_PSH_VV1:
            next_state = S51;
            break;
        case SYN_ACK_VV0:
            next_state = S53;
            break;
        case RST_VV0:
            next_state = S51;
            break;
        case ACCEPT:
            next_state = S52;
            break;
        case FIN_ACK_VV0:
            next_state = S51;
            break;
        case LISTEN:
            next_state = S52;
            break;
        case SYN_VV0:
            next_state = S52;
            break;
        case RCV:
            next_state = S52;
            break;
        case ACK_RST_VV0:
            next_state = S54;
            break;
        case CLOSE:
            next_state = S5;
            break;
        case SEND:
            next_state = S52;
            break;
        case ACK_VV0:
            next_state = S51;
            break;
        default:
            next_state = S52;
            break;
        }
        break;

    case S53:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S44;
            break;
        case ACK_PSH_VV1:
            next_state = S51;
            break;
        case SYN_ACK_VV0:
            next_state = S53;
            break;
        case RST_VV0:
            next_state = S51;
            break;
        case ACCEPT:
            next_state = S53;
            break;
        case FIN_ACK_VV0:
            next_state = S51;
            break;
        case LISTEN:
            next_state = S53;
            break;
        case SYN_VV0:
            next_state = S52;
            break;
        case RCV:
            next_state = S53;
            break;
        case ACK_RST_VV0:
            next_state = S51;
            break;
        case CLOSE:
            next_state = S5;
            break;
        case SEND:
            next_state = S53;
            break;
        case ACK_VV0:
            next_state = S51;
            break;
        default:
            next_state = S53;
            break;
        }
        break;

    case S54:
        switch (event)
        {
        case CLOSECONNECTION:
            next_state = S45;
            break;
        case ACK_PSH_VV1:
            next_state = S51;
            break;
        case SYN_ACK_VV0:
            next_state = S51;
            break;
        case RST_VV0:
            next_state = S54;
            break;
        case ACCEPT:
            next_state = S54;
            break;
        case FIN_ACK_VV0:
            next_state = S51;
            break;
        case LISTEN:
            next_state = S54;
            break;
        case SYN_VV0:
            next_state = S54;
            break;
        case RCV:
            next_state = S54;
            break;
        case ACK_RST_VV0:
            next_state = S54;
            break;
        case CLOSE:
            next_state = S2;
            break;
        case SEND:
            next_state = S54;
            break;
        case ACK_VV0:
            next_state = S51;
            break;
        default:
            next_state = S54;
            break;
        }
        break;
    }

    return next_state;
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

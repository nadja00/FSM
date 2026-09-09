#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
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
#define UBRR_VAL ((F_CPU / (16UL * BAUD)) - 1) // ovde ispadne oko 9615 BR

static void uart_init(void)
{
    UBRR0H = (uint8_t)(UBRR_VAL >> 8);
    UBRR0L = (uint8_t)UBRR_VAL;
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8N1
}

static void uart_putc(char c)
{
    while (!(UCSR0A & (1 << UDRE0)))
        ;
    UDR0 = c;
}

static void uart_puts(const char *s)
{
    while (*s)
        uart_putc(*s++);
}

static void uart_put_uint(uint16_t v)
{
    char buf[6];
    uint8_t i = 0;
    if (v == 0)
    {
        uart_putc('0');
        return;
    }

    while (v > 0)
    {
        buf[i++] = '0' + (v % 10);
        v /= 10;
    }

    while (i > 0)
        uart_putc(buf[--i]);
}

static uint8_t uart_available(void)
{
    return (UCSR0A & (1 << RXC0)) != 0;
}

static char uart_getc(void)
{
    while (!(UCSR0A & (1 << RXC0)))
        ;
    return UDR0;
}

static inline void cycles_start(void)
{
    TCCR1B = 0;           // stop timer
    TCNT1 = 0;            // reset counter
    TCCR1B = (1 << CS10); // start, no prescaler (1 cycle per tick)
}

static inline uint16_t cycles_stop(void)
{
    TCCR1B = 0;   // stop timer
    return TCNT1; // read elapsed cycles
}

static uint8_t fsm_transition(uint8_t state, uint8_t event)
{
    switch (state)
    {
    case S0:
        switch (event)
        {
        case CLOSECONNECTION:
            return S0;
        case ACK_PSH_VV1:
            return S0;
        case SYN_ACK_VV0:
            return S0;
        case RST_VV0:
            return S0;
        case ACCEPT:
            return S0;
        case FIN_ACK_VV0:
            return S0;
        case LISTEN:
            return S1;
        case SYN_VV0:
            return S0;
        case RCV:
            return S0;
        case ACK_RST_VV0:
            return S0;
        case CLOSE:
            return S2;
        case SEND:
            return S0;
        case ACK_VV0:
            return S0;
        default:
            return S0;
        }
    
    case S1:
        switch (event)
        {
        case CLOSECONNECTION:
            return S1;
        case ACK_PSH_VV1:
            return S1;
        case SYN_ACK_VV0:
            return S1;
        case RST_VV0:
            return S1;
        case ACCEPT:
            return S4;
        case FIN_ACK_VV0:
            return S1;
        case LISTEN:
            return S1;
        case SYN_VV0:
            return S3;
        case RCV:
            return S1;
        case ACK_RST_VV0:
            return S1;
        case CLOSE:
            return S2;
        case SEND:
            return S1;
        case ACK_VV0:
            return S1;
        default:
            return S1;
        }
    
    case S2:
        switch (event)
        {
        case CLOSECONNECTION:
            return S2;
        case ACK_PSH_VV1:
            return S2;
        case SYN_ACK_VV0:
            return S2;
        case RST_VV0:
            return S2;
        case ACCEPT:
            return S2;
        case FIN_ACK_VV0:
            return S2;
        case LISTEN:
            return S2;
        case SYN_VV0:
            return S2;
        case RCV:
            return S2;
        case ACK_RST_VV0:
            return S2;
        case CLOSE:
            return S2;
        case SEND:
            return S2;
        case ACK_VV0:
            return S2;
        default:
            return S2;
        }
    
    case S3:
        switch (event)
        {
        case CLOSECONNECTION:
            return S3;
        case ACK_PSH_VV1:
            return S8;
        case SYN_ACK_VV0:
            return S6;
        case RST_VV0:
            return S1;
        case ACCEPT:
            return S9;
        case FIN_ACK_VV0:
            return S7;
        case LISTEN:
            return S3;
        case SYN_VV0:
            return S3;
        case RCV:
            return S3;
        case ACK_RST_VV0:
            return S10;
        case CLOSE:
            return S5;
        case SEND:
            return S3;
        case ACK_VV0:
            return S8;
        default:
            return S3;
        }
    
    case S4:
        switch (event)
        {
        case CLOSECONNECTION:
            return S1;
        case ACK_PSH_VV1:
            return S4;
        case SYN_ACK_VV0:
            return S4;
        case RST_VV0:
            return S4;
        case ACCEPT:
            return S4;
        case FIN_ACK_VV0:
            return S4;
        case LISTEN:
            return S4;
        case SYN_VV0:
            return S9;
        case RCV:
            return S4;
        case ACK_RST_VV0:
            return S4;
        case CLOSE:
            return S2;
        case SEND:
            return S4;
        case ACK_VV0:
            return S4;
        default:
            return S4;
        }
    
    case S5:
        switch (event)
        {
        case CLOSECONNECTION:
            return S5;
        case ACK_PSH_VV1:
            return S2;
        case SYN_ACK_VV0:
            return S5;
        case RST_VV0:
            return S2;
        case ACCEPT:
            return S5;
        case FIN_ACK_VV0:
            return S2;
        case LISTEN:
            return S5;
        case SYN_VV0:
            return S2;
        case RCV:
            return S5;
        case ACK_RST_VV0:
            return S2;
        case CLOSE:
            return S5;
        case SEND:
            return S5;
        case ACK_VV0:
            return S2;
        default:
            return S5;
        }
    
    case S6:
        switch (event)
        {
        case CLOSECONNECTION:
            return S6;
        case ACK_PSH_VV1:
            return S1;
        case SYN_ACK_VV0:
            return S6;
        case RST_VV0:
            return S1;
        case ACCEPT:
            return S11;
        case FIN_ACK_VV0:
            return S1;
        case LISTEN:
            return S6;
        case SYN_VV0:
            return S3;
        case RCV:
            return S6;
        case ACK_RST_VV0:
            return S1;
        case CLOSE:
            return S5;
        case SEND:
            return S6;
        case ACK_VV0:
            return S1;
        default:
            return S6;
        }
    
    case S7:
        switch (event)
        {
        case CLOSECONNECTION:
            return S7;
        case ACK_PSH_VV1:
            return S7;
        case SYN_ACK_VV0:
            return S12;
        case RST_VV0:
            return S12;
        case ACCEPT:
            return S13;
        case FIN_ACK_VV0:
            return S7;
        case LISTEN:
            return S7;
        case SYN_VV0:
            return S12;
        case RCV:
            return S7;
        case ACK_RST_VV0:
            return S12;
        case CLOSE:
            return S2;
        case SEND:
            return S7;
        case ACK_VV0:
            return S7;
        default:
            return S7;
        }
    
    case S8:
        switch (event)
        {
        case CLOSECONNECTION:
            return S8;
        case ACK_PSH_VV1:
            return S8;
        case SYN_ACK_VV0:
            return S12;
        case RST_VV0:
            return S12;
        case ACCEPT:
            return S14;
        case FIN_ACK_VV0:
            return S7;
        case LISTEN:
            return S8;
        case SYN_VV0:
            return S12;
        case RCV:
            return S8;
        case ACK_RST_VV0:
            return S12;
        case CLOSE:
            return S2;
        case SEND:
            return S8;
        case ACK_VV0:
            return S8;
        default:
            return S8;
        }
    
    case S9:
        switch (event)
        {
        case CLOSECONNECTION:
            return S3;
        case ACK_PSH_VV1:
            return S14;
        case SYN_ACK_VV0:
            return S11;
        case RST_VV0:
            return S4;
        case ACCEPT:
            return S9;
        case FIN_ACK_VV0:
            return S13;
        case LISTEN:
            return S9;
        case SYN_VV0:
            return S9;
        case RCV:
            return S9;
        case ACK_RST_VV0:
            return S15;
        case CLOSE:
            return S5;
        case SEND:
            return S9;
        case ACK_VV0:
            return S14;
        default:
            return S9;
        }
    
    case S10:
        switch (event)
        {
        case CLOSECONNECTION:
            return S10;
        case ACK_PSH_VV1:
            return S1;
        case SYN_ACK_VV0:
            return S1;
        case RST_VV0:
            return S10;
        case ACCEPT:
            return S15;
        case FIN_ACK_VV0:
            return S1;
        case LISTEN:
            return S10;
        case SYN_VV0:
            return S10;
        case RCV:
            return S10;
        case ACK_RST_VV0:
            return S10;
        case CLOSE:
            return S2;
        case SEND:
            return S10;
        case ACK_VV0:
            return S1;
        default:
            return S10;
        }
    
    case S11:
        switch (event)
        {
        case CLOSECONNECTION:
            return S6;
        case ACK_PSH_VV1:
            return S4;
        case SYN_ACK_VV0:
            return S11;
        case RST_VV0:
            return S4;
        case ACCEPT:
            return S11;
        case FIN_ACK_VV0:
            return S4;
        case LISTEN:
            return S11;
        case SYN_VV0:
            return S9;
        case RCV:
            return S11;
        case ACK_RST_VV0:
            return S4;
        case CLOSE:
            return S5;
        case SEND:
            return S11;
        case ACK_VV0:
            return S4;
        default:
            return S11;
        }
    
    case S12:
        switch (event)
        {
        case CLOSECONNECTION:
            return S12;
        case ACK_PSH_VV1:
            return S12;
        case SYN_ACK_VV0:
            return S12;
        case RST_VV0:
            return S12;
        case ACCEPT:
            return S1;
        case FIN_ACK_VV0:
            return S12;
        case LISTEN:
            return S12;
        case SYN_VV0:
            return S16;
        case RCV:
            return S12;
        case ACK_RST_VV0:
            return S12;
        case CLOSE:
            return S2;
        case SEND:
            return S12;
        case ACK_VV0:
            return S12;
        default:
            return S12;
        }
    
    case S13:
        switch (event)
        {
        case CLOSECONNECTION:
            return S18;
        case ACK_PSH_VV1:
            return S13;
        case SYN_ACK_VV0:
            return S19;
        case RST_VV0:
            return S19;
        case ACCEPT:
            return S13;
        case FIN_ACK_VV0:
            return S13;
        case LISTEN:
            return S13;
        case SYN_VV0:
            return S19;
        case RCV:
            return S13;
        case ACK_RST_VV0:
            return S19;
        case CLOSE:
            return S17;
        case SEND:
            return S13;
        case ACK_VV0:
            return S13;
        default:
            return S13;
        }
    
    case S14:
        switch (event)
        {
        case CLOSECONNECTION:
            return S21;
        case ACK_PSH_VV1:
            return S14;
        case SYN_ACK_VV0:
            return S19;
        case RST_VV0:
            return S19;
        case ACCEPT:
            return S14;
        case FIN_ACK_VV0:
            return S13;
        case LISTEN:
            return S14;
        case SYN_VV0:
            return S19;
        case RCV:
            return S14;
        case ACK_RST_VV0:
            return S19;
        case CLOSE:
            return S20;
        case SEND:
            return S14;
        case ACK_VV0:
            return S14;
        default:
            return S14;
        }
    
    case S15:
        switch (event)
        {
        case CLOSECONNECTION:
            return S10;
        case ACK_PSH_VV1:
            return S4;
        case SYN_ACK_VV0:
            return S4;
        case RST_VV0:
            return S15;
        case ACCEPT:
            return S15;
        case FIN_ACK_VV0:
            return S4;
        case LISTEN:
            return S15;
        case SYN_VV0:
            return S15;
        case RCV:
            return S15;
        case ACK_RST_VV0:
            return S15;
        case CLOSE:
            return S2;
        case SEND:
            return S15;
        case ACK_VV0:
            return S4;
        default:
            return S15;
        }
    
    case S16:
        switch (event)
        {
        case CLOSECONNECTION:
            return S16;
        case ACK_PSH_VV1:
            return S23;
        case SYN_ACK_VV0:
            return S22;
        case RST_VV0:
            return S12;
        case ACCEPT:
            return S3;
        case FIN_ACK_VV0:
            return S24;
        case LISTEN:
            return S16;
        case SYN_VV0:
            return S16;
        case RCV:
            return S16;
        case ACK_RST_VV0:
            return S25;
        case CLOSE:
            return S5;
        case SEND:
            return S16;
        case ACK_VV0:
            return S23;
        default:
            return S16;
        }
    
    case S17:
        switch (event)
        {
        case CLOSECONNECTION:
            return S26;
        case ACK_PSH_VV1:
            return S17;
        case SYN_ACK_VV0:
            return S2;
        case RST_VV0:
            return S2;
        case ACCEPT:
            return S17;
        case FIN_ACK_VV0:
            return S17;
        case LISTEN:
            return S17;
        case SYN_VV0:
            return S2;
        case RCV:
            return S17;
        case ACK_RST_VV0:
            return S2;
        case CLOSE:
            return S17;
        case SEND:
            return S17;
        case ACK_VV0:
            return S17;
        default:
            return S17;
        }
    
    case S18:
        switch (event)
        {
        case CLOSECONNECTION:
            return S18;
        case ACK_PSH_VV1:
            return S1;
        case SYN_ACK_VV0:
            return S1;
        case RST_VV0:
            return S1;
        case ACCEPT:
            return S27;
        case FIN_ACK_VV0:
            return S6;
        case LISTEN:
            return S18;
        case SYN_VV0:
            return S1;
        case RCV:
            return S18;
        case ACK_RST_VV0:
            return S1;
        case CLOSE:
            return S26;
        case SEND:
            return S18;
        case ACK_VV0:
            return S6;
        default:
            return S18;
        }
    
    case S19:
        switch (event)
        {
        case CLOSECONNECTION:
            return S1;
        case ACK_PSH_VV1:
            return S19;
        case SYN_ACK_VV0:
            return S19;
        case RST_VV0:
            return S19;
        case ACCEPT:
            return S19;
        case FIN_ACK_VV0:
            return S19;
        case LISTEN:
            return S19;
        case SYN_VV0:
            return S28;
        case RCV:
            return S19;
        case ACK_RST_VV0:
            return S19;
        case CLOSE:
            return S2;
        case SEND:
            return S19;
        case ACK_VV0:
            return S19;
        default:
            return S19;
        }
    
    case S20:
        switch (event)
        {
        case CLOSECONNECTION:
            return S29;
        case ACK_PSH_VV1:
            return S20;
        case SYN_ACK_VV0:
            return S2;
        case RST_VV0:
            return S2;
        case ACCEPT:
            return S20;
        case FIN_ACK_VV0:
            return S17;
        case LISTEN:
            return S20;
        case SYN_VV0:
            return S2;
        case RCV:
            return S20;
        case ACK_RST_VV0:
            return S2;
        case CLOSE:
            return S20;
        case SEND:
            return S20;
        case ACK_VV0:
            return S20;
        default:
            return S20;
        }
    
    case S21:
        switch (event)
        {
        case CLOSECONNECTION:
            return S21;
        case ACK_PSH_VV1:
            return S1;
        case SYN_ACK_VV0:
            return S1;
        case RST_VV0:
            return S1;
        case ACCEPT:
            return S30;
        case FIN_ACK_VV0:
            return S31;
        case LISTEN:
            return S21;
        case SYN_VV0:
            return S1;
        case RCV:
            return S21;
        case ACK_RST_VV0:
            return S1;
        case CLOSE:
            return S29;
        case SEND:
            return S21;
        case ACK_VV0:
            return S21;
        default:
            return S21;
        }
    
    case S22:
        switch (event)
        {
        case CLOSECONNECTION:
            return S22;
        case ACK_PSH_VV1:
            return S12;
        case SYN_ACK_VV0:
            return S22;
        case RST_VV0:
            return S12;
        case ACCEPT:
            return S6;
        case FIN_ACK_VV0:
            return S12;
        case LISTEN:
            return S22;
        case SYN_VV0:
            return S16;
        case RCV:
            return S22;
        case ACK_RST_VV0:
            return S12;
        case CLOSE:
            return S5;
        case SEND:
            return S22;
        case ACK_VV0:
            return S12;
        default:
            return S22;
        }
    
    case S23:
        switch (event)
        {
        case CLOSECONNECTION:
            return S23;
        case ACK_PSH_VV1:
            return S23;
        case SYN_ACK_VV0:
            return S32;
        case RST_VV0:
            return S32;
        case ACCEPT:
            return S8;
        case FIN_ACK_VV0:
            return S24;
        case LISTEN:
            return S23;
        case SYN_VV0:
            return S32;
        case RCV:
            return S23;
        case ACK_RST_VV0:
            return S32;
        case CLOSE:
            return S2;
        case SEND:
            return S23;
        case ACK_VV0:
            return S23;
        default:
            return S23;
        }
    
    case S24:
        switch (event)
        {
        case CLOSECONNECTION:
            return S24;
        case ACK_PSH_VV1:
            return S24;
        case SYN_ACK_VV0:
            return S32;
        case RST_VV0:
            return S32;
        case ACCEPT:
            return S7;
        case FIN_ACK_VV0:
            return S24;
        case LISTEN:
            return S24;
        case SYN_VV0:
            return S32;
        case RCV:
            return S24;
        case ACK_RST_VV0:
            return S32;
        case CLOSE:
            return S2;
        case SEND:
            return S24;
        case ACK_VV0:
            return S24;
        default:
            return S24;
        }
    
    case S25:
        switch (event)
        {
        case CLOSECONNECTION:
            return S25;
        case ACK_PSH_VV1:
            return S12;
        case SYN_ACK_VV0:
            return S12;
        case RST_VV0:
            return S25;
        case ACCEPT:
            return S10;
        case FIN_ACK_VV0:
            return S12;
        case LISTEN:
            return S25;
        case SYN_VV0:
            return S25;
        case RCV:
            return S25;
        case ACK_RST_VV0:
            return S25;
        case CLOSE:
            return S2;
        case SEND:
            return S25;
        case ACK_VV0:
            return S12;
        default:
            return S25;
        }
    
    case S26:
        switch (event)
        {
        case CLOSECONNECTION:
            return S26;
        case ACK_PSH_VV1:
            return S2;
        case SYN_ACK_VV0:
            return S2;
        case RST_VV0:
            return S2;
        case ACCEPT:
            return S26;
        case FIN_ACK_VV0:
            return S5;
        case LISTEN:
            return S26;
        case SYN_VV0:
            return S2;
        case RCV:
            return S26;
        case ACK_RST_VV0:
            return S2;
        case CLOSE:
            return S26;
        case SEND:
            return S26;
        case ACK_VV0:
            return S5;
        default:
            return S26;
        }
    
    case S27:
        switch (event)
        {
        case CLOSECONNECTION:
            return S18;
        case ACK_PSH_VV1:
            return S4;
        case SYN_ACK_VV0:
            return S4;
        case RST_VV0:
            return S4;
        case ACCEPT:
            return S27;
        case FIN_ACK_VV0:
            return S11;
        case LISTEN:
            return S27;
        case SYN_VV0:
            return S4;
        case RCV:
            return S27;
        case ACK_RST_VV0:
            return S4;
        case CLOSE:
            return S26;
        case SEND:
            return S27;
        case ACK_VV0:
            return S11;
        default:
            return S27;
        }
    
    case S28:
        switch (event)
        {
        case CLOSECONNECTION:
            return S3;
        case ACK_PSH_VV1:
            return S34;
        case SYN_ACK_VV0:
            return S36;
        case RST_VV0:
            return S19;
        case ACCEPT:
            return S28;
        case FIN_ACK_VV0:
            return S33;
        case LISTEN:
            return S28;
        case SYN_VV0:
            return S28;
        case RCV:
            return S28;
        case ACK_RST_VV0:
            return S35;
        case CLOSE:
            return S5;
        case SEND:
            return S28;
        case ACK_VV0:
            return S34;
        default:
            return S28;
        }
    
    case S29:
        switch (event)
        {
        case CLOSECONNECTION:
            return S29;
        case ACK_PSH_VV1:
            return S2;
        case SYN_ACK_VV0:
            return S2;
        case RST_VV0:
            return S2;
        case ACCEPT:
            return S29;
        case FIN_ACK_VV0:
            return S37;
        case LISTEN:
            return S29;
        case SYN_VV0:
            return S2;
        case RCV:
            return S29;
        case ACK_RST_VV0:
            return S2;
        case CLOSE:
            return S29;
        case SEND:
            return S29;
        case ACK_VV0:
            return S29;
        default:
            return S29;
        }
    
    case S30:
        switch (event)
        {
        case CLOSECONNECTION:
            return S21;
        case ACK_PSH_VV1:
            return S4;
        case SYN_ACK_VV0:
            return S4;
        case RST_VV0:
            return S4;
        case ACCEPT:
            return S30;
        case FIN_ACK_VV0:
            return S38;
        case LISTEN:
            return S30;
        case SYN_VV0:
            return S4;
        case RCV:
            return S30;
        case ACK_RST_VV0:
            return S4;
        case CLOSE:
            return S29;
        case SEND:
            return S30;
        case ACK_VV0:
            return S30;
        default:
            return S30;
        }
    
    case S31:
        switch (event)
        {
        case CLOSECONNECTION:
            return S31;
        case ACK_PSH_VV1:
            return S31;
        case SYN_ACK_VV0:
            return S31;
        case RST_VV0:
            return S39;
        case ACCEPT:
            return S38;
        case FIN_ACK_VV0:
            return S31;
        case LISTEN:
            return S31;
        case SYN_VV0:
            return S31;
        case RCV:
            return S31;
        case ACK_RST_VV0:
            return S39;
        case CLOSE:
            return S37;
        case SEND:
            return S31;
        case ACK_VV0:
            return S31;
        default:
            return S31;
        }
    
    case S32:
        switch (event)
        {
        case CLOSECONNECTION:
            return S32;
        case ACK_PSH_VV1:
            return S32;
        case SYN_ACK_VV0:
            return S32;
        case RST_VV0:
            return S32;
        case ACCEPT:
            return S12;
        case FIN_ACK_VV0:
            return S32;
        case LISTEN:
            return S32;
        case SYN_VV0:
            return S40;
        case RCV:
            return S32;
        case ACK_RST_VV0:
            return S32;
        case CLOSE:
            return S2;
        case SEND:
            return S32;
        case ACK_VV0:
            return S32;
        default:
            return S32;
        }
    
    case S33:
        switch (event)
        {
        case CLOSECONNECTION:
            return S7;
        case ACK_PSH_VV1:
            return S33;
        case SYN_ACK_VV0:
            return S41;
        case RST_VV0:
            return S41;
        case ACCEPT:
            return S33;
        case FIN_ACK_VV0:
            return S33;
        case LISTEN:
            return S33;
        case SYN_VV0:
            return S41;
        case RCV:
            return S33;
        case ACK_RST_VV0:
            return S41;
        case CLOSE:
            return S2;
        case SEND:
            return S33;
        case ACK_VV0:
            return S33;
        default:
            return S33;
        }
    
    case S34:
        switch (event)
        {
        case CLOSECONNECTION:
            return S8;
        case ACK_PSH_VV1:
            return S34;
        case SYN_ACK_VV0:
            return S41;
        case RST_VV0:
            return S41;
        case ACCEPT:
            return S34;
        case FIN_ACK_VV0:
            return S33;
        case LISTEN:
            return S34;
        case SYN_VV0:
            return S41;
        case RCV:
            return S34;
        case ACK_RST_VV0:
            return S41;
        case CLOSE:
            return S2;
        case SEND:
            return S34;
        case ACK_VV0:
            return S34;
        default:
            return S34;
        }
    
    case S35:
        switch (event)
        {
        case CLOSECONNECTION:
            return S10;
        case ACK_PSH_VV1:
            return S19;
        case SYN_ACK_VV0:
            return S19;
        case RST_VV0:
            return S35;
        case ACCEPT:
            return S35;
        case FIN_ACK_VV0:
            return S19;
        case LISTEN:
            return S35;
        case SYN_VV0:
            return S35;
        case RCV:
            return S35;
        case ACK_RST_VV0:
            return S35;
        case CLOSE:
            return S2;
        case SEND:
            return S35;
        case ACK_VV0:
            return S19;
        default:
            return S35;
        }
    
    case S36:
        switch (event)
        {
        case CLOSECONNECTION:
            return S6;
        case ACK_PSH_VV1:
            return S19;
        case SYN_ACK_VV0:
            return S36;
        case RST_VV0:
            return S19;
        case ACCEPT:
            return S36;
        case FIN_ACK_VV0:
            return S19;
        case LISTEN:
            return S36;
        case SYN_VV0:
            return S28;
        case RCV:
            return S36;
        case ACK_RST_VV0:
            return S19;
        case CLOSE:
            return S5;
        case SEND:
            return S36;
        case ACK_VV0:
            return S19;
        default:
            return S36;
        }
    
    case S37:
        switch (event)
        {
        case CLOSECONNECTION:
            return S37;
        case ACK_PSH_VV1:
            return S37;
        case SYN_ACK_VV0:
            return S37;
        case RST_VV0:
            return S42;
        case ACCEPT:
            return S37;
        case FIN_ACK_VV0:
            return S37;
        case LISTEN:
            return S37;
        case SYN_VV0:
            return S37;
        case RCV:
            return S37;
        case ACK_RST_VV0:
            return S42;
        case CLOSE:
            return S37;
        case SEND:
            return S37;
        case ACK_VV0:
            return S37;
        default:
            return S37;
        }
    
    case S38:
        switch (event)
        {
        case CLOSECONNECTION:
            return S31;
        case ACK_PSH_VV1:
            return S38;
        case SYN_ACK_VV0:
            return S38;
        case RST_VV0:
            return S43;
        case ACCEPT:
            return S38;
        case FIN_ACK_VV0:
            return S38;
        case LISTEN:
            return S38;
        case SYN_VV0:
            return S38;
        case RCV:
            return S38;
        case ACK_RST_VV0:
            return S43;
        case CLOSE:
            return S37;
        case SEND:
            return S38;
        case ACK_VV0:
            return S38;
        default:
            return S38;
        }
    
    case S39:
        switch (event)
        {
        case CLOSECONNECTION:
            return S39;
        case ACK_PSH_VV1:
            return S39;
        case SYN_ACK_VV0:
            return S39;
        case RST_VV0:
            return S39;
        case ACCEPT:
            return S43;
        case FIN_ACK_VV0:
            return S39;
        case LISTEN:
            return S39;
        case SYN_VV0:
            return S3;
        case RCV:
            return S39;
        case ACK_RST_VV0:
            return S39;
        case CLOSE:
            return S42;
        case SEND:
            return S39;
        case ACK_VV0:
            return S39;
        default:
            return S39;
        }
    
    case S40:
        switch (event)
        {
        case CLOSECONNECTION:
            return S40;
        case ACK_PSH_VV1:
            return S32;
        case SYN_ACK_VV0:
            return S44;
        case RST_VV0:
            return S32;
        case ACCEPT:
            return S16;
        case FIN_ACK_VV0:
            return S32;
        case LISTEN:
            return S40;
        case SYN_VV0:
            return S40;
        case RCV:
            return S40;
        case ACK_RST_VV0:
            return S45;
        case CLOSE:
            return S5;
        case SEND:
            return S40;
        case ACK_VV0:
            return S32;
        default:
            return S40;
        }
    
    case S41:
        switch (event)
        {
        case CLOSECONNECTION:
            return S12;
        case ACK_PSH_VV1:
            return S41;
        case SYN_ACK_VV0:
            return S41;
        case RST_VV0:
            return S41;
        case ACCEPT:
            return S41;
        case FIN_ACK_VV0:
            return S41;
        case LISTEN:
            return S41;
        case SYN_VV0:
            return S46;
        case RCV:
            return S41;
        case ACK_RST_VV0:
            return S41;
        case CLOSE:
            return S2;
        case SEND:
            return S41;
        case ACK_VV0:
            return S41;
        default:
            return S41;
        }
    
    case S42:
        switch (event)
        {
        case CLOSECONNECTION:
            return S42;
        case ACK_PSH_VV1:
            return S42;
        case SYN_ACK_VV0:
            return S42;
        case RST_VV0:
            return S42;
        case ACCEPT:
            return S42;
        case FIN_ACK_VV0:
            return S42;
        case LISTEN:
            return S42;
        case SYN_VV0:
            return S2;
        case RCV:
            return S42;
        case ACK_RST_VV0:
            return S42;
        case CLOSE:
            return S42;
        case SEND:
            return S42;
        case ACK_VV0:
            return S42;
        default:
            return S42;
        }
    
    case S43:
        switch (event)
        {
        case CLOSECONNECTION:
            return S39;
        case ACK_PSH_VV1:
            return S43;
        case SYN_ACK_VV0:
            return S43;
        case RST_VV0:
            return S43;
        case ACCEPT:
            return S43;
        case FIN_ACK_VV0:
            return S43;
        case LISTEN:
            return S43;
        case SYN_VV0:
            return S9;
        case RCV:
            return S43;
        case ACK_RST_VV0:
            return S43;
        case CLOSE:
            return S42;
        case SEND:
            return S43;
        case ACK_VV0:
            return S43;
        default:
            return S43;
        }
    
    case S44:
        switch (event)
        {
        case CLOSECONNECTION:
            return S44;
        case ACK_PSH_VV1:
            return S32;
        case SYN_ACK_VV0:
            return S44;
        case RST_VV0:
            return S32;
        case ACCEPT:
            return S22;
        case FIN_ACK_VV0:
            return S32;
        case LISTEN:
            return S44;
        case SYN_VV0:
            return S40;
        case RCV:
            return S44;
        case ACK_RST_VV0:
            return S32;
        case CLOSE:
            return S5;
        case SEND:
            return S44;
        case ACK_VV0:
            return S32;
        default:
            return S44;
        }
    
    case S45:
        switch (event)
        {
        case CLOSECONNECTION:
            return S45;
        case ACK_PSH_VV1:
            return S32;
        case SYN_ACK_VV0:
            return S32;
        case RST_VV0:
            return S45;
        case ACCEPT:
            return S25;
        case FIN_ACK_VV0:
            return S32;
        case LISTEN:
            return S45;
        case SYN_VV0:
            return S45;
        case RCV:
            return S45;
        case ACK_RST_VV0:
            return S45;
        case CLOSE:
            return S2;
        case SEND:
            return S45;
        case ACK_VV0:
            return S32;
        default:
            return S45;
        }
    
    case S46:
        switch (event)
        {
        case CLOSECONNECTION:
            return S16;
        case ACK_PSH_VV1:
            return S48;
        case SYN_ACK_VV0:
            return S49;
        case RST_VV0:
            return S41;
        case ACCEPT:
            return S46;
        case FIN_ACK_VV0:
            return S50;
        case LISTEN:
            return S46;
        case SYN_VV0:
            return S46;
        case RCV:
            return S46;
        case ACK_RST_VV0:
            return S47;
        case CLOSE:
            return S5;
        case SEND:
            return S46;
        case ACK_VV0:
            return S48;
        default:
            return S46;
        }
    
    case S47:
        switch (event)
        {
        case CLOSECONNECTION:
            return S25;
        case ACK_PSH_VV1:
            return S41;
        case SYN_ACK_VV0:
            return S41;
        case RST_VV0:
            return S47;
        case ACCEPT:
            return S47;
        case FIN_ACK_VV0:
            return S41;
        case LISTEN:
            return S47;
        case SYN_VV0:
            return S47;
        case RCV:
            return S47;
        case ACK_RST_VV0:
            return S47;
        case CLOSE:
            return S2;
        case SEND:
            return S47;
        case ACK_VV0:
            return S41;
        default:
            return S47;
        }
    
    case S48:
        switch (event)
        {
        case CLOSECONNECTION:
            return S23;
        case ACK_PSH_VV1:
            return S48;
        case SYN_ACK_VV0:
            return S51;
        case RST_VV0:
            return S51;
        case ACCEPT:
            return S48;
        case FIN_ACK_VV0:
            return S50;
        case LISTEN:
            return S48;
        case SYN_VV0:
            return S51;
        case RCV:
            return S48;
        case ACK_RST_VV0:
            return S51;
        case CLOSE:
            return S2;
        case SEND:
            return S48;
        case ACK_VV0:
            return S48;
        default:
            return S48;
        }
    
    case S49:
        switch (event)
        {
        case CLOSECONNECTION:
            return S22;
        case ACK_PSH_VV1:
            return S41;
        case SYN_ACK_VV0:
            return S49;
        case RST_VV0:
            return S41;
        case ACCEPT:
            return S49;
        case FIN_ACK_VV0:
            return S41;
        case LISTEN:
            return S49;
        case SYN_VV0:
            return S46;
        case RCV:
            return S49;
        case ACK_RST_VV0:
            return S41;
        case CLOSE:
            return S5;
        case SEND:
            return S49;
        case ACK_VV0:
            return S41;
        default:
            return S49;
        }
    
    case S50:
        switch (event)
        {
        case CLOSECONNECTION:
            return S24;
        case ACK_PSH_VV1:
            return S50;
        case SYN_ACK_VV0:
            return S51;
        case RST_VV0:
            return S51;
        case ACCEPT:
            return S50;
        case FIN_ACK_VV0:
            return S50;
        case LISTEN:
            return S50;
        case SYN_VV0:
            return S51;
        case RCV:
            return S50;
        case ACK_RST_VV0:
            return S51;
        case CLOSE:
            return S2;
        case SEND:
            return S50;
        case ACK_VV0:
            return S50;
        default:
            return S50;
        }
    
    case S51:
        switch (event)
        {
        case CLOSECONNECTION:
            return S32;
        case ACK_PSH_VV1:
            return S51;
        case SYN_ACK_VV0:
            return S51;
        case RST_VV0:
            return S51;
        case ACCEPT:
            return S51;
        case FIN_ACK_VV0:
            return S51;
        case LISTEN:
            return S51;
        case SYN_VV0:
            return S52;
        case RCV:
            return S51;
        case ACK_RST_VV0:
            return S51;
        case CLOSE:
            return S2;
        case SEND:
            return S51;
        case ACK_VV0:
            return S51;
        default:
            return S51;
        }
    
    case S52:
        switch (event)
        {
        case CLOSECONNECTION:
            return S40;
        case ACK_PSH_VV1:
            return S51;
        case SYN_ACK_VV0:
            return S53;
        case RST_VV0:
            return S51;
        case ACCEPT:
            return S52;
        case FIN_ACK_VV0:
            return S51;
        case LISTEN:
            return S52;
        case SYN_VV0:
            return S52;
        case RCV:
            return S52;
        case ACK_RST_VV0:
            return S54;
        case CLOSE:
            return S5;
        case SEND:
            return S52;
        case ACK_VV0:
            return S51;
        default:
            return S52;
        }
    
    case S53:
        switch (event)
        {
        case CLOSECONNECTION:
            return S44;
        case ACK_PSH_VV1:
            return S51;
        case SYN_ACK_VV0:
            return S53;
        case RST_VV0:
            return S51;
        case ACCEPT:
            return S53;
        case FIN_ACK_VV0:
            return S51;
        case LISTEN:
            return S53;
        case SYN_VV0:
            return S52;
        case RCV:
            return S53;
        case ACK_RST_VV0:
            return S51;
        case CLOSE:
            return S5;
        case SEND:
            return S53;
        case ACK_VV0:
            return S51;
        default:
            return S53;
        }
    
    case S54:
        switch (event)
        {
        case CLOSECONNECTION:
            return S45;
        case ACK_PSH_VV1:
            return S51;
        case SYN_ACK_VV0:
            return S51;
        case RST_VV0:
            return S54;
        case ACCEPT:
            return S54;
        case FIN_ACK_VV0:
            return S51;
        case LISTEN:
            return S54;
        case SYN_VV0:
            return S54;
        case RCV:
            return S54;
        case ACK_RST_VV0:
            return S54;
        case CLOSE:
            return S2;
        case SEND:
            return S54;
        case ACK_VV0:
            return S51;
        default:
            return S54;
        }
    default:
        return S0;
    }
}

static uint16_t measured_transition(uint8_t event)
{
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

        if (event < 0 || event >= NUM_EVENTS)  //event je uint pa svakako nikad nece biti <0 pa ovaj uslov sa && nikad nece biti ispunjen
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

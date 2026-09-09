#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>

// ---------- States & Events (identical enums to previous versions) ----------

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

#define EVENT_UNHANDLED 0xFF  // sentinel: "this handler does not consume this event"

static volatile uint8_t current_state = S0;

// ---------- UART (raw register access, no Serial lib) ----------
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

// ---------- Timer1 cycle counter ----------
static inline void cycles_start(void) {
    TCCR1B = 0;
    TCNT1 = 0;
    TCCR1B = (1 << CS10); // no prescaler
}

static inline uint16_t cycles_stop(void) {
    TCCR1B = 0;
    return TCNT1;
}

// ---------- HSM state descriptor ----------
typedef uint8_t (*StateHandler)(uint8_t event);

typedef struct {
    StateHandler handler;
    int8_t parent; // index into state_table[], or -1 if no parent (top state)
} StateNode;

// ---------- Leaf state handlers ----------
// Each handler returns EVENT_UNHANDLED if it does not process the given event,
// which triggers bubbling to the parent state (if any).

static uint8_t state_s0_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S0;
    if (event == ACK_PSH_VV1)
        return S0;
    if (event == SYN_ACK_VV0)
        return S0;
    if (event == RST_VV0)
        return S0;
    if (event == ACCEPT)
        return S0;
    if (event == FIN_ACK_VV0)
        return S0;
    if (event == LISTEN)
        return S1;
    if (event == SYN_VV0)
        return S0;
    if (event == RCV)
        return S0;
    if (event == ACK_RST_VV0)
        return S0;
    if (event == CLOSE)
        return S2;
    if (event == SEND)
        return S0;
    if (event == ACK_VV0)
        return S0;
    return S0; // default
}

static uint8_t state_s1_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S1;
    if (event == ACK_PSH_VV1)
        return S1;
    if (event == SYN_ACK_VV0)
        return S1;
    if (event == RST_VV0)
        return S1;
    if (event == ACCEPT)
        return S4;
    if (event == FIN_ACK_VV0)
        return S1;
    if (event == LISTEN)
        return S1;
    if (event == SYN_VV0)
        return S3;
    if (event == RCV)
        return S1;
    if (event == ACK_RST_VV0)
        return S1;
    if (event == CLOSE)
        return S2;
    if (event == SEND)
        return S1;
    if (event == ACK_VV0)
        return S1;
    return S1; // default
}

static uint8_t state_s2_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S2;
    if (event == ACK_PSH_VV1)
        return S2;
    if (event == SYN_ACK_VV0)
        return S2;
    if (event == RST_VV0)
        return S2;
    if (event == ACCEPT)
        return S2;
    if (event == FIN_ACK_VV0)
        return S2;
    if (event == LISTEN)
        return S2;
    if (event == SYN_VV0)
        return S2;
    if (event == RCV)
        return S2;
    if (event == ACK_RST_VV0)
        return S2;
    if (event == CLOSE)
        return S2;
    if (event == SEND)
        return S2;
    if (event == ACK_VV0)
        return S2;
    return S2; // default
}

static uint8_t state_s3_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S3;
    if (event == ACK_PSH_VV1)
        return S8;
    if (event == SYN_ACK_VV0)
        return S6;
    if (event == RST_VV0)
        return S1;
    if (event == ACCEPT)
        return S9;
    if (event == FIN_ACK_VV0)
        return S7;
    if (event == LISTEN)
        return S3;
    if (event == SYN_VV0)
        return S3;
    if (event == RCV)
        return S3;
    if (event == ACK_RST_VV0)
        return S10;
    if (event == CLOSE)
        return S5;
    if (event == SEND)
        return S3;
    if (event == ACK_VV0)
        return S8;
    return S3; // default
}

static uint8_t state_s4_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S1;
    if (event == ACK_PSH_VV1)
        return S4;
    if (event == SYN_ACK_VV0)
        return S4;
    if (event == RST_VV0)
        return S4;
    if (event == ACCEPT)
        return S4;
    if (event == FIN_ACK_VV0)
        return S4;
    if (event == LISTEN)
        return S4;
    if (event == SYN_VV0)
        return S9;
    if (event == RCV)
        return S4;
    if (event == ACK_RST_VV0)
        return S4;
    if (event == CLOSE)
        return S2;
    if (event == SEND)
        return S4;
    if (event == ACK_VV0)
        return S4;
    return S4; // default
}

static uint8_t state_s5_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S5;
    if (event == ACK_PSH_VV1)
        return S2;
    if (event == SYN_ACK_VV0)
        return S5;
    if (event == RST_VV0)
        return S2;
    if (event == ACCEPT)
        return S5;
    if (event == FIN_ACK_VV0)
        return S2;
    if (event == LISTEN)
        return S5;
    if (event == SYN_VV0)
        return S2;
    if (event == RCV)
        return S5;
    if (event == ACK_RST_VV0)
        return S2;
    if (event == CLOSE)
        return S5;
    if (event == SEND)
        return S5;
    if (event == ACK_VV0)
        return S2;
    return S5; // default
}

static uint8_t state_s6_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S6;
    if (event == ACK_PSH_VV1)
        return S1;
    if (event == SYN_ACK_VV0)
        return S6;
    if (event == RST_VV0)
        return S1;
    if (event == ACCEPT)
        return S11;
    if (event == FIN_ACK_VV0)
        return S1;
    if (event == LISTEN)
        return S6;
    if (event == SYN_VV0)
        return S3;
    if (event == RCV)
        return S6;
    if (event == ACK_RST_VV0)
        return S1;
    if (event == CLOSE)
        return S5;
    if (event == SEND)
        return S6;
    if (event == ACK_VV0)
        return S1;
    return S6; // default
}

static uint8_t state_s7_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S7;
    if (event == ACK_PSH_VV1)
        return S7;
    if (event == SYN_ACK_VV0)
        return S12;
    if (event == RST_VV0)
        return S12;
    if (event == ACCEPT)
        return S13;
    if (event == FIN_ACK_VV0)
        return S7;
    if (event == LISTEN)
        return S7;
    if (event == SYN_VV0)
        return S12;
    if (event == RCV)
        return S7;
    if (event == ACK_RST_VV0)
        return S12;
    if (event == CLOSE)
        return S2;
    if (event == SEND)
        return S7;
    if (event == ACK_VV0)
        return S7;
    return S7; // default
}

static uint8_t state_s8_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S8;
    if (event == ACK_PSH_VV1)
        return S8;
    if (event == SYN_ACK_VV0)
        return S12;
    if (event == RST_VV0)
        return S12;
    if (event == ACCEPT)
        return S14;
    if (event == FIN_ACK_VV0)
        return S7;
    if (event == LISTEN)
        return S8;
    if (event == SYN_VV0)
        return S12;
    if (event == RCV)
        return S8;
    if (event == ACK_RST_VV0)
        return S12;
    if (event == CLOSE)
        return S2;
    if (event == SEND)
        return S8;
    if (event == ACK_VV0)
        return S8;
    return S8; // default
}

static uint8_t state_s9_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S3;
    if (event == ACK_PSH_VV1)
        return S14;
    if (event == SYN_ACK_VV0)
        return S11;
    if (event == RST_VV0)
        return S4;
    if (event == ACCEPT)
        return S9;
    if (event == FIN_ACK_VV0)
        return S13;
    if (event == LISTEN)
        return S9;
    if (event == SYN_VV0)
        return S9;
    if (event == RCV)
        return S9;
    if (event == ACK_RST_VV0)
        return S15;
    if (event == CLOSE)
        return S5;
    if (event == SEND)
        return S9;
    if (event == ACK_VV0)
        return S14;
    return S9; // default
}

static uint8_t state_s10_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S10;
    if (event == ACK_PSH_VV1)
        return S1;
    if (event == SYN_ACK_VV0)
        return S1;
    if (event == RST_VV0)
        return S10;
    if (event == ACCEPT)
        return S15;
    if (event == FIN_ACK_VV0)
        return S1;
    if (event == LISTEN)
        return S10;
    if (event == SYN_VV0)
        return S10;
    if (event == RCV)
        return S10;
    if (event == ACK_RST_VV0)
        return S10;
    if (event == CLOSE)
        return S2;
    if (event == SEND)
        return S10;
    if (event == ACK_VV0)
        return S1;
    return S10; // default
}

static uint8_t state_s11_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S6;
    if (event == ACK_PSH_VV1)
        return S4;
    if (event == SYN_ACK_VV0)
        return S11;
    if (event == RST_VV0)
        return S4;
    if (event == ACCEPT)
        return S11;
    if (event == FIN_ACK_VV0)
        return S4;
    if (event == LISTEN)
        return S11;
    if (event == SYN_VV0)
        return S9;
    if (event == RCV)
        return S11;
    if (event == ACK_RST_VV0)
        return S4;
    if (event == CLOSE)
        return S5;
    if (event == SEND)
        return S11;
    if (event == ACK_VV0)
        return S4;
    return S11; // default
}

static uint8_t state_s12_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S12;
    if (event == ACK_PSH_VV1)
        return S12;
    if (event == SYN_ACK_VV0)
        return S12;
    if (event == RST_VV0)
        return S12;
    if (event == ACCEPT)
        return S1;
    if (event == FIN_ACK_VV0)
        return S12;
    if (event == LISTEN)
        return S12;
    if (event == SYN_VV0)
        return S16;
    if (event == RCV)
        return S12;
    if (event == ACK_RST_VV0)
        return S12;
    if (event == CLOSE)
        return S2;
    if (event == SEND)
        return S12;
    if (event == ACK_VV0)
        return S12;
    return S12; // default
}

static uint8_t state_s13_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S18;
    if (event == ACK_PSH_VV1)
        return S13;
    if (event == SYN_ACK_VV0)
        return S19;
    if (event == RST_VV0)
        return S19;
    if (event == ACCEPT)
        return S13;
    if (event == FIN_ACK_VV0)
        return S13;
    if (event == LISTEN)
        return S13;
    if (event == SYN_VV0)
        return S19;
    if (event == RCV)
        return S13;
    if (event == ACK_RST_VV0)
        return S19;
    if (event == CLOSE)
        return S17;
    if (event == SEND)
        return S13;
    if (event == ACK_VV0)
        return S13;
    return S13; // default
}

static uint8_t state_s14_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S21;
    if (event == ACK_PSH_VV1)
        return S14;
    if (event == SYN_ACK_VV0)
        return S19;
    if (event == RST_VV0)
        return S19;
    if (event == ACCEPT)
        return S14;
    if (event == FIN_ACK_VV0)
        return S13;
    if (event == LISTEN)
        return S14;
    if (event == SYN_VV0)
        return S19;
    if (event == RCV)
        return S14;
    if (event == ACK_RST_VV0)
        return S19;
    if (event == CLOSE)
        return S20;
    if (event == SEND)
        return S14;
    if (event == ACK_VV0)
        return S14;
    return S14; // default
}

static uint8_t state_s15_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S10;
    if (event == ACK_PSH_VV1)
        return S4;
    if (event == SYN_ACK_VV0)
        return S4;
    if (event == RST_VV0)
        return S15;
    if (event == ACCEPT)
        return S15;
    if (event == FIN_ACK_VV0)
        return S4;
    if (event == LISTEN)
        return S15;
    if (event == SYN_VV0)
        return S15;
    if (event == RCV)
        return S15;
    if (event == ACK_RST_VV0)
        return S15;
    if (event == CLOSE)
        return S2;
    if (event == SEND)
        return S15;
    if (event == ACK_VV0)
        return S4;
    return S15; // default
}

static uint8_t state_s16_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S16;
    if (event == ACK_PSH_VV1)
        return S23;
    if (event == SYN_ACK_VV0)
        return S22;
    if (event == RST_VV0)
        return S12;
    if (event == ACCEPT)
        return S3;
    if (event == FIN_ACK_VV0)
        return S24;
    if (event == LISTEN)
        return S16;
    if (event == SYN_VV0)
        return S16;
    if (event == RCV)
        return S16;
    if (event == ACK_RST_VV0)
        return S25;
    if (event == CLOSE)
        return S5;
    if (event == SEND)
        return S16;
    if (event == ACK_VV0)
        return S23;
    return S16; // default
}

static uint8_t state_s17_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S26;
    if (event == ACK_PSH_VV1)
        return S17;
    if (event == SYN_ACK_VV0)
        return S2;
    if (event == RST_VV0)
        return S2;
    if (event == ACCEPT)
        return S17;
    if (event == FIN_ACK_VV0)
        return S17;
    if (event == LISTEN)
        return S17;
    if (event == SYN_VV0)
        return S2;
    if (event == RCV)
        return S17;
    if (event == ACK_RST_VV0)
        return S2;
    if (event == CLOSE)
        return S17;
    if (event == SEND)
        return S17;
    if (event == ACK_VV0)
        return S17;
    return S17; // default
}

static uint8_t state_s18_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S18;
    if (event == ACK_PSH_VV1)
        return S1;
    if (event == SYN_ACK_VV0)
        return S1;
    if (event == RST_VV0)
        return S1;
    if (event == ACCEPT)
        return S27;
    if (event == FIN_ACK_VV0)
        return S6;
    if (event == LISTEN)
        return S18;
    if (event == SYN_VV0)
        return S1;
    if (event == RCV)
        return S18;
    if (event == ACK_RST_VV0)
        return S1;
    if (event == CLOSE)
        return S26;
    if (event == SEND)
        return S18;
    if (event == ACK_VV0)
        return S6;
    return S18; // default
}

static uint8_t state_s19_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S1;
    if (event == ACK_PSH_VV1)
        return S19;
    if (event == SYN_ACK_VV0)
        return S19;
    if (event == RST_VV0)
        return S19;
    if (event == ACCEPT)
        return S19;
    if (event == FIN_ACK_VV0)
        return S19;
    if (event == LISTEN)
        return S19;
    if (event == SYN_VV0)
        return S28;
    if (event == RCV)
        return S19;
    if (event == ACK_RST_VV0)
        return S19;
    if (event == CLOSE)
        return S2;
    if (event == SEND)
        return S19;
    if (event == ACK_VV0)
        return S19;
    return S19; // default
}

static uint8_t state_s20_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S29;
    if (event == ACK_PSH_VV1)
        return S20;
    if (event == SYN_ACK_VV0)
        return S2;
    if (event == RST_VV0)
        return S2;
    if (event == ACCEPT)
        return S20;
    if (event == FIN_ACK_VV0)
        return S17;
    if (event == LISTEN)
        return S20;
    if (event == SYN_VV0)
        return S2;
    if (event == RCV)
        return S20;
    if (event == ACK_RST_VV0)
        return S2;
    if (event == CLOSE)
        return S20;
    if (event == SEND)
        return S20;
    if (event == ACK_VV0)
        return S20;
    return S20; // default
}

static uint8_t state_s21_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S21;
    if (event == ACK_PSH_VV1)
        return S1;
    if (event == SYN_ACK_VV0)
        return S1;
    if (event == RST_VV0)
        return S1;
    if (event == ACCEPT)
        return S30;
    if (event == FIN_ACK_VV0)
        return S31;
    if (event == LISTEN)
        return S21;
    if (event == SYN_VV0)
        return S1;
    if (event == RCV)
        return S21;
    if (event == ACK_RST_VV0)
        return S1;
    if (event == CLOSE)
        return S29;
    if (event == SEND)
        return S21;
    if (event == ACK_VV0)
        return S21;
    return S21; // default
}

static uint8_t state_s22_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S22;
    if (event == ACK_PSH_VV1)
        return S12;
    if (event == SYN_ACK_VV0)
        return S22;
    if (event == RST_VV0)
        return S12;
    if (event == ACCEPT)
        return S6;
    if (event == FIN_ACK_VV0)
        return S12;
    if (event == LISTEN)
        return S22;
    if (event == SYN_VV0)
        return S16;
    if (event == RCV)
        return S22;
    if (event == ACK_RST_VV0)
        return S12;
    if (event == CLOSE)
        return S5;
    if (event == SEND)
        return S22;
    if (event == ACK_VV0)
        return S12;
    return S22; // default
}

static uint8_t state_s23_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S23;
    if (event == ACK_PSH_VV1)
        return S23;
    if (event == SYN_ACK_VV0)
        return S32;
    if (event == RST_VV0)
        return S32;
    if (event == ACCEPT)
        return S8;
    if (event == FIN_ACK_VV0)
        return S24;
    if (event == LISTEN)
        return S23;
    if (event == SYN_VV0)
        return S32;
    if (event == RCV)
        return S23;
    if (event == ACK_RST_VV0)
        return S32;
    if (event == CLOSE)
        return S2;
    if (event == SEND)
        return S23;
    if (event == ACK_VV0)
        return S23;
    return S23; // default
}

static uint8_t state_s24_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S24;
    if (event == ACK_PSH_VV1)
        return S24;
    if (event == SYN_ACK_VV0)
        return S32;
    if (event == RST_VV0)
        return S32;
    if (event == ACCEPT)
        return S7;
    if (event == FIN_ACK_VV0)
        return S24;
    if (event == LISTEN)
        return S24;
    if (event == SYN_VV0)
        return S32;
    if (event == RCV)
        return S24;
    if (event == ACK_RST_VV0)
        return S32;
    if (event == CLOSE)
        return S2;
    if (event == SEND)
        return S24;
    if (event == ACK_VV0)
        return S24;
    return S24; // default
}

static uint8_t state_s25_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S25;
    if (event == ACK_PSH_VV1)
        return S12;
    if (event == SYN_ACK_VV0)
        return S12;
    if (event == RST_VV0)
        return S25;
    if (event == ACCEPT)
        return S10;
    if (event == FIN_ACK_VV0)
        return S12;
    if (event == LISTEN)
        return S25;
    if (event == SYN_VV0)
        return S25;
    if (event == RCV)
        return S25;
    if (event == ACK_RST_VV0)
        return S25;
    if (event == CLOSE)
        return S2;
    if (event == SEND)
        return S25;
    if (event == ACK_VV0)
        return S12;
    return S25; // default
}

static uint8_t state_s26_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S26;
    if (event == ACK_PSH_VV1)
        return S2;
    if (event == SYN_ACK_VV0)
        return S2;
    if (event == RST_VV0)
        return S2;
    if (event == ACCEPT)
        return S26;
    if (event == FIN_ACK_VV0)
        return S5;
    if (event == LISTEN)
        return S26;
    if (event == SYN_VV0)
        return S2;
    if (event == RCV)
        return S26;
    if (event == ACK_RST_VV0)
        return S2;
    if (event == CLOSE)
        return S26;
    if (event == SEND)
        return S26;
    if (event == ACK_VV0)
        return S5;
    return S26; // default
}

static uint8_t state_s27_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S18;
    if (event == ACK_PSH_VV1)
        return S4;
    if (event == SYN_ACK_VV0)
        return S4;
    if (event == RST_VV0)
        return S4;
    if (event == ACCEPT)
        return S27;
    if (event == FIN_ACK_VV0)
        return S11;
    if (event == LISTEN)
        return S27;
    if (event == SYN_VV0)
        return S4;
    if (event == RCV)
        return S27;
    if (event == ACK_RST_VV0)
        return S4;
    if (event == CLOSE)
        return S26;
    if (event == SEND)
        return S27;
    if (event == ACK_VV0)
        return S11;
    return S27; // default
}

static uint8_t state_s28_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S3;
    if (event == ACK_PSH_VV1)
        return S34;
    if (event == SYN_ACK_VV0)
        return S36;
    if (event == RST_VV0)
        return S19;
    if (event == ACCEPT)
        return S28;
    if (event == FIN_ACK_VV0)
        return S33;
    if (event == LISTEN)
        return S28;
    if (event == SYN_VV0)
        return S28;
    if (event == RCV)
        return S28;
    if (event == ACK_RST_VV0)
        return S35;
    if (event == CLOSE)
        return S5;
    if (event == SEND)
        return S28;
    if (event == ACK_VV0)
        return S34;
    return S28; // default
}

static uint8_t state_s29_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S29;
    if (event == ACK_PSH_VV1)
        return S2;
    if (event == SYN_ACK_VV0)
        return S2;
    if (event == RST_VV0)
        return S2;
    if (event == ACCEPT)
        return S29;
    if (event == FIN_ACK_VV0)
        return S37;
    if (event == LISTEN)
        return S29;
    if (event == SYN_VV0)
        return S2;
    if (event == RCV)
        return S29;
    if (event == ACK_RST_VV0)
        return S2;
    if (event == CLOSE)
        return S29;
    if (event == SEND)
        return S29;
    if (event == ACK_VV0)
        return S29;
    return S29; // default
}

static uint8_t state_s30_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S21;
    if (event == ACK_PSH_VV1)
        return S4;
    if (event == SYN_ACK_VV0)
        return S4;
    if (event == RST_VV0)
        return S4;
    if (event == ACCEPT)
        return S30;
    if (event == FIN_ACK_VV0)
        return S38;
    if (event == LISTEN)
        return S30;
    if (event == SYN_VV0)
        return S4;
    if (event == RCV)
        return S30;
    if (event == ACK_RST_VV0)
        return S4;
    if (event == CLOSE)
        return S29;
    if (event == SEND)
        return S30;
    if (event == ACK_VV0)
        return S30;
    return S30; // default
}

static uint8_t state_s31_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S31;
    if (event == ACK_PSH_VV1)
        return S31;
    if (event == SYN_ACK_VV0)
        return S31;
    if (event == RST_VV0)
        return S39;
    if (event == ACCEPT)
        return S38;
    if (event == FIN_ACK_VV0)
        return S31;
    if (event == LISTEN)
        return S31;
    if (event == SYN_VV0)
        return S31;
    if (event == RCV)
        return S31;
    if (event == ACK_RST_VV0)
        return S39;
    if (event == CLOSE)
        return S37;
    if (event == SEND)
        return S31;
    if (event == ACK_VV0)
        return S31;
    return S31; // default
}

static uint8_t state_s32_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S32;
    if (event == ACK_PSH_VV1)
        return S32;
    if (event == SYN_ACK_VV0)
        return S32;
    if (event == RST_VV0)
        return S32;
    if (event == ACCEPT)
        return S12;
    if (event == FIN_ACK_VV0)
        return S32;
    if (event == LISTEN)
        return S32;
    if (event == SYN_VV0)
        return S40;
    if (event == RCV)
        return S32;
    if (event == ACK_RST_VV0)
        return S32;
    if (event == CLOSE)
        return S2;
    if (event == SEND)
        return S32;
    if (event == ACK_VV0)
        return S32;
    return S32; // default
}

static uint8_t state_s33_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S7;
    if (event == ACK_PSH_VV1)
        return S33;
    if (event == SYN_ACK_VV0)
        return S41;
    if (event == RST_VV0)
        return S41;
    if (event == ACCEPT)
        return S33;
    if (event == FIN_ACK_VV0)
        return S33;
    if (event == LISTEN)
        return S33;
    if (event == SYN_VV0)
        return S41;
    if (event == RCV)
        return S33;
    if (event == ACK_RST_VV0)
        return S41;
    if (event == CLOSE)
        return S2;
    if (event == SEND)
        return S33;
    if (event == ACK_VV0)
        return S33;
    return S33; // default
}

static uint8_t state_s34_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S8;
    if (event == ACK_PSH_VV1)
        return S34;
    if (event == SYN_ACK_VV0)
        return S41;
    if (event == RST_VV0)
        return S41;
    if (event == ACCEPT)
        return S34;
    if (event == FIN_ACK_VV0)
        return S33;
    if (event == LISTEN)
        return S34;
    if (event == SYN_VV0)
        return S41;
    if (event == RCV)
        return S34;
    if (event == ACK_RST_VV0)
        return S41;
    if (event == CLOSE)
        return S2;
    if (event == SEND)
        return S34;
    if (event == ACK_VV0)
        return S34;
    return S34; // default
}

static uint8_t state_s35_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S10;
    if (event == ACK_PSH_VV1)
        return S19;
    if (event == SYN_ACK_VV0)
        return S19;
    if (event == RST_VV0)
        return S35;
    if (event == ACCEPT)
        return S35;
    if (event == FIN_ACK_VV0)
        return S19;
    if (event == LISTEN)
        return S35;
    if (event == SYN_VV0)
        return S35;
    if (event == RCV)
        return S35;
    if (event == ACK_RST_VV0)
        return S35;
    if (event == CLOSE)
        return S2;
    if (event == SEND)
        return S35;
    if (event == ACK_VV0)
        return S19;
    return S35; // default
}

static uint8_t state_s36_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S6;
    if (event == ACK_PSH_VV1)
        return S19;
    if (event == SYN_ACK_VV0)
        return S36;
    if (event == RST_VV0)
        return S19;
    if (event == ACCEPT)
        return S36;
    if (event == FIN_ACK_VV0)
        return S19;
    if (event == LISTEN)
        return S36;
    if (event == SYN_VV0)
        return S28;
    if (event == RCV)
        return S36;
    if (event == ACK_RST_VV0)
        return S19;
    if (event == CLOSE)
        return S5;
    if (event == SEND)
        return S36;
    if (event == ACK_VV0)
        return S19;
    return S36; // default
}

static uint8_t state_s37_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S37;
    if (event == ACK_PSH_VV1)
        return S37;
    if (event == SYN_ACK_VV0)
        return S37;
    if (event == RST_VV0)
        return S42;
    if (event == ACCEPT)
        return S37;
    if (event == FIN_ACK_VV0)
        return S37;
    if (event == LISTEN)
        return S37;
    if (event == SYN_VV0)
        return S37;
    if (event == RCV)
        return S37;
    if (event == ACK_RST_VV0)
        return S42;
    if (event == CLOSE)
        return S37;
    if (event == SEND)
        return S37;
    if (event == ACK_VV0)
        return S37;
    return S37; // default
}

static uint8_t state_s38_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S31;
    if (event == ACK_PSH_VV1)
        return S38;
    if (event == SYN_ACK_VV0)
        return S38;
    if (event == RST_VV0)
        return S43;
    if (event == ACCEPT)
        return S38;
    if (event == FIN_ACK_VV0)
        return S38;
    if (event == LISTEN)
        return S38;
    if (event == SYN_VV0)
        return S38;
    if (event == RCV)
        return S38;
    if (event == ACK_RST_VV0)
        return S43;
    if (event == CLOSE)
        return S37;
    if (event == SEND)
        return S38;
    if (event == ACK_VV0)
        return S38;
    return S38; // default
}

static uint8_t state_s39_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S39;
    if (event == ACK_PSH_VV1)
        return S39;
    if (event == SYN_ACK_VV0)
        return S39;
    if (event == RST_VV0)
        return S39;
    if (event == ACCEPT)
        return S43;
    if (event == FIN_ACK_VV0)
        return S39;
    if (event == LISTEN)
        return S39;
    if (event == SYN_VV0)
        return S3;
    if (event == RCV)
        return S39;
    if (event == ACK_RST_VV0)
        return S39;
    if (event == CLOSE)
        return S42;
    if (event == SEND)
        return S39;
    if (event == ACK_VV0)
        return S39;
    return S39; // default
}

static uint8_t state_s40_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S40;
    if (event == ACK_PSH_VV1)
        return S32;
    if (event == SYN_ACK_VV0)
        return S44;
    if (event == RST_VV0)
        return S32;
    if (event == ACCEPT)
        return S16;
    if (event == FIN_ACK_VV0)
        return S32;
    if (event == LISTEN)
        return S40;
    if (event == SYN_VV0)
        return S40;
    if (event == RCV)
        return S40;
    if (event == ACK_RST_VV0)
        return S45;
    if (event == CLOSE)
        return S5;
    if (event == SEND)
        return S40;
    if (event == ACK_VV0)
        return S32;
    return S40; // default
}

static uint8_t state_s41_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S12;
    if (event == ACK_PSH_VV1)
        return S41;
    if (event == SYN_ACK_VV0)
        return S41;
    if (event == RST_VV0)
        return S41;
    if (event == ACCEPT)
        return S41;
    if (event == FIN_ACK_VV0)
        return S41;
    if (event == LISTEN)
        return S41;
    if (event == SYN_VV0)
        return S46;
    if (event == RCV)
        return S41;
    if (event == ACK_RST_VV0)
        return S41;
    if (event == CLOSE)
        return S2;
    if (event == SEND)
        return S41;
    if (event == ACK_VV0)
        return S41;
    return S41; // default
}

static uint8_t state_s42_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S42;
    if (event == ACK_PSH_VV1)
        return S42;
    if (event == SYN_ACK_VV0)
        return S42;
    if (event == RST_VV0)
        return S42;
    if (event == ACCEPT)
        return S42;
    if (event == FIN_ACK_VV0)
        return S42;
    if (event == LISTEN)
        return S42;
    if (event == SYN_VV0)
        return S2;
    if (event == RCV)
        return S42;
    if (event == ACK_RST_VV0)
        return S42;
    if (event == CLOSE)
        return S42;
    if (event == SEND)
        return S42;
    if (event == ACK_VV0)
        return S42;
    return S42; // default
}

static uint8_t state_s43_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S39;
    if (event == ACK_PSH_VV1)
        return S43;
    if (event == SYN_ACK_VV0)
        return S43;
    if (event == RST_VV0)
        return S43;
    if (event == ACCEPT)
        return S43;
    if (event == FIN_ACK_VV0)
        return S43;
    if (event == LISTEN)
        return S43;
    if (event == SYN_VV0)
        return S9;
    if (event == RCV)
        return S43;
    if (event == ACK_RST_VV0)
        return S43;
    if (event == CLOSE)
        return S42;
    if (event == SEND)
        return S43;
    if (event == ACK_VV0)
        return S43;
    return S43; // default
}

static uint8_t state_s44_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S44;
    if (event == ACK_PSH_VV1)
        return S32;
    if (event == SYN_ACK_VV0)
        return S44;
    if (event == RST_VV0)
        return S32;
    if (event == ACCEPT)
        return S22;
    if (event == FIN_ACK_VV0)
        return S32;
    if (event == LISTEN)
        return S44;
    if (event == SYN_VV0)
        return S40;
    if (event == RCV)
        return S44;
    if (event == ACK_RST_VV0)
        return S32;
    if (event == CLOSE)
        return S5;
    if (event == SEND)
        return S44;
    if (event == ACK_VV0)
        return S32;
    return S44; // default
}

static uint8_t state_s45_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S45;
    if (event == ACK_PSH_VV1)
        return S32;
    if (event == SYN_ACK_VV0)
        return S32;
    if (event == RST_VV0)
        return S45;
    if (event == ACCEPT)
        return S25;
    if (event == FIN_ACK_VV0)
        return S32;
    if (event == LISTEN)
        return S45;
    if (event == SYN_VV0)
        return S45;
    if (event == RCV)
        return S45;
    if (event == ACK_RST_VV0)
        return S45;
    if (event == CLOSE)
        return S2;
    if (event == SEND)
        return S45;
    if (event == ACK_VV0)
        return S32;
    return S45; // default
}

static uint8_t state_s46_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S16;
    if (event == ACK_PSH_VV1)
        return S48;
    if (event == SYN_ACK_VV0)
        return S49;
    if (event == RST_VV0)
        return S41;
    if (event == ACCEPT)
        return S46;
    if (event == FIN_ACK_VV0)
        return S50;
    if (event == LISTEN)
        return S46;
    if (event == SYN_VV0)
        return S46;
    if (event == RCV)
        return S46;
    if (event == ACK_RST_VV0)
        return S47;
    if (event == CLOSE)
        return S5;
    if (event == SEND)
        return S46;
    if (event == ACK_VV0)
        return S48;
    return S46; // default
}

static uint8_t state_s47_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S25;
    if (event == ACK_PSH_VV1)
        return S41;
    if (event == SYN_ACK_VV0)
        return S41;
    if (event == RST_VV0)
        return S47;
    if (event == ACCEPT)
        return S47;
    if (event == FIN_ACK_VV0)
        return S41;
    if (event == LISTEN)
        return S47;
    if (event == SYN_VV0)
        return S47;
    if (event == RCV)
        return S47;
    if (event == ACK_RST_VV0)
        return S47;
    if (event == CLOSE)
        return S2;
    if (event == SEND)
        return S47;
    if (event == ACK_VV0)
        return S41;
    return S47; // default
}

static uint8_t state_s48_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S23;
    if (event == ACK_PSH_VV1)
        return S48;
    if (event == SYN_ACK_VV0)
        return S51;
    if (event == RST_VV0)
        return S51;
    if (event == ACCEPT)
        return S48;
    if (event == FIN_ACK_VV0)
        return S50;
    if (event == LISTEN)
        return S48;
    if (event == SYN_VV0)
        return S51;
    if (event == RCV)
        return S48;
    if (event == ACK_RST_VV0)
        return S51;
    if (event == CLOSE)
        return S2;
    if (event == SEND)
        return S48;
    if (event == ACK_VV0)
        return S48;
    return S48; // default
}

static uint8_t state_s49_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S22;
    if (event == ACK_PSH_VV1)
        return S41;
    if (event == SYN_ACK_VV0)
        return S49;
    if (event == RST_VV0)
        return S41;
    if (event == ACCEPT)
        return S49;
    if (event == FIN_ACK_VV0)
        return S41;
    if (event == LISTEN)
        return S49;
    if (event == SYN_VV0)
        return S46;
    if (event == RCV)
        return S49;
    if (event == ACK_RST_VV0)
        return S41;
    if (event == CLOSE)
        return S5;
    if (event == SEND)
        return S49;
    if (event == ACK_VV0)
        return S41;
    return S49; // default
}

static uint8_t state_s50_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S24;
    if (event == ACK_PSH_VV1)
        return S50;
    if (event == SYN_ACK_VV0)
        return S51;
    if (event == RST_VV0)
        return S51;
    if (event == ACCEPT)
        return S50;
    if (event == FIN_ACK_VV0)
        return S50;
    if (event == LISTEN)
        return S50;
    if (event == SYN_VV0)
        return S51;
    if (event == RCV)
        return S50;
    if (event == ACK_RST_VV0)
        return S51;
    if (event == CLOSE)
        return S2;
    if (event == SEND)
        return S50;
    if (event == ACK_VV0)
        return S50;
    return S50; // default
}

static uint8_t state_s51_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S32;
    if (event == ACK_PSH_VV1)
        return S51;
    if (event == SYN_ACK_VV0)
        return S51;
    if (event == RST_VV0)
        return S51;
    if (event == ACCEPT)
        return S51;
    if (event == FIN_ACK_VV0)
        return S51;
    if (event == LISTEN)
        return S51;
    if (event == SYN_VV0)
        return S52;
    if (event == RCV)
        return S51;
    if (event == ACK_RST_VV0)
        return S51;
    if (event == CLOSE)
        return S2;
    if (event == SEND)
        return S51;
    if (event == ACK_VV0)
        return S51;
    return S51; // default
}

static uint8_t state_s52_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S40;
    if (event == ACK_PSH_VV1)
        return S51;
    if (event == SYN_ACK_VV0)
        return S53;
    if (event == RST_VV0)
        return S51;
    if (event == ACCEPT)
        return S52;
    if (event == FIN_ACK_VV0)
        return S51;
    if (event == LISTEN)
        return S52;
    if (event == SYN_VV0)
        return S52;
    if (event == RCV)
        return S52;
    if (event == ACK_RST_VV0)
        return S54;
    if (event == CLOSE)
        return S5;
    if (event == SEND)
        return S52;
    if (event == ACK_VV0)
        return S51;
    return S52; // default
}

static uint8_t state_s53_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S44;
    if (event == ACK_PSH_VV1)
        return S51;
    if (event == SYN_ACK_VV0)
        return S53;
    if (event == RST_VV0)
        return S51;
    if (event == ACCEPT)
        return S53;
    if (event == FIN_ACK_VV0)
        return S51;
    if (event == LISTEN)
        return S53;
    if (event == SYN_VV0)
        return S52;
    if (event == RCV)
        return S53;
    if (event == ACK_RST_VV0)
        return S51;
    if (event == CLOSE)
        return S5;
    if (event == SEND)
        return S53;
    if (event == ACK_VV0)
        return S51;
    return S53; // default
}

static uint8_t state_s54_handle(uint8_t event)
{
    if (event == CLOSECONNECTION)
        return S45;
    if (event == ACK_PSH_VV1)
        return S51;
    if (event == SYN_ACK_VV0)
        return S51;
    if (event == RST_VV0)
        return S54;
    if (event == ACCEPT)
        return S54;
    if (event == FIN_ACK_VV0)
        return S51;
    if (event == LISTEN)
        return S54;
    if (event == SYN_VV0)
        return S54;
    if (event == RCV)
        return S54;
    if (event == ACK_RST_VV0)
        return S54;
    if (event == CLOSE)
        return S2;
    if (event == SEND)
        return S54;
    if (event == ACK_VV0)
        return S51;
    return S54; // default
}

// ---------- State table: every parent is -1 (NO real hierarchy in this test) ----------
static const StateNode state_table[NUM_STATES] = {
    { state_s0_handle, -1 },
    { state_s1_handle, -1 },
    { state_s2_handle, -1 },
    { state_s3_handle, -1 },
    { state_s4_handle, -1 },
    { state_s5_handle, -1 },
    { state_s6_handle, -1 },
    { state_s7_handle, -1 },
    { state_s8_handle, -1 },
    { state_s9_handle, -1 },
    { state_s10_handle, -1 },
    { state_s11_handle, -1 },
    { state_s12_handle, -1 },
    { state_s13_handle, -1 },
    { state_s14_handle, -1 },
    { state_s15_handle, -1 },
    { state_s16_handle, -1 },
    { state_s17_handle, -1 },
    { state_s18_handle, -1 },
    { state_s19_handle, -1 },
    { state_s20_handle, -1 },
    { state_s21_handle, -1 },
    { state_s22_handle, -1 },
    { state_s23_handle, -1 },
    { state_s24_handle, -1 },
    { state_s25_handle, -1 },
    { state_s26_handle, -1 },
    { state_s27_handle, -1 },
    { state_s28_handle, -1 },
    { state_s29_handle, -1 },
    { state_s30_handle, -1 },
    { state_s31_handle, -1 },
    { state_s32_handle, -1 },
    { state_s33_handle, -1 },
    { state_s34_handle, -1 },
    { state_s35_handle, -1 },
    { state_s36_handle, -1 },
    { state_s37_handle, -1 },
    { state_s38_handle, -1 },
    { state_s39_handle, -1 },
    { state_s40_handle, -1 },
    { state_s41_handle, -1 },
    { state_s42_handle, -1 },
    { state_s43_handle, -1 },
    { state_s44_handle, -1 },
    { state_s45_handle, -1 },
    { state_s46_handle, -1 },
    { state_s47_handle, -1 },
    { state_s48_handle, -1 },
    { state_s49_handle, -1 },
    { state_s50_handle, -1 },
    { state_s51_handle, -1 },
    { state_s52_handle, -1 },
    { state_s53_handle, -1 },
    { state_s54_handle, -1 }
};

// ---------- HSM dispatch: try current state, bubble up to parent if unhandled ----------
static uint8_t fsm_transition(uint8_t state, uint8_t event) {
    int8_t node = state;
    uint8_t result;

    while (node != -1) {
        result = state_table[node].handler(event);
        if (result != EVENT_UNHANDLED) {
            return result; // consumed by this state (or an ancestor)
        }
        node = state_table[node].parent; // bubble up
    }
    return state; // nobody in the chain handled it -> stay in current state
}

// ---------- Single measured transition ----------
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
    
    uart_puts("Min cycles: "); uart_put_uint(min_c); uart_puts("\r\n");
    uart_puts("Max cycles: "); uart_put_uint(max_c); uart_puts("\r\n");
    uart_puts("Avg cycles: "); uart_put_uint((uint16_t)(sum_c / N)); uart_puts("\r\n");
}

// ---------- Setup / Loop ----------
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

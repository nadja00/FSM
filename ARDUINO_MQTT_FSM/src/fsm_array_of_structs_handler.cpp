/*
 * Rad: Carlgren, J., Oskarsson, P. W. (2023). "State Machine Model-To-Code
 * Transformation In C." UPTEC F 23044, Uppsala University - sekcija 3.7
 * "Array of Structs" (Figure 9).
 *
 * Access Control FSM - Array of Structs Pattern, VERNIJA varijanta prema radu.
 * FSM: 4 states - IDLE, CHECKING, GRANTED, DENIED (identicno ostalim testovima)
 * Events: EV_VALID, EV_INVALID, EV_TIMEOUT
 *
 * Razlika od fsm_array_of_structs.cpp: svaki unos u nizu struct-ova ovde ne
 * cuva next_state direktno vec POKAZIVAC NA eventHandler FUNKCIJU (tacno kao
 * "stateMachineEventHandler" polje u radu), koja se poziva i vraca next_state.
 * Ovo dodaje jedan indirektni poziv funkcije po tranziciji koji jednostavnija
 * verzija (fsm_array_of_structs.cpp, po uzoru na A. Kumar) nema.
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

// eventHandler funkcije - svaka vraca sledece stanje (kao u Fig. 6/9 rada)
typedef uint8_t (*EventHandler)(void);

static uint8_t handler_s0_connectc2(void)          { return S3; }
static uint8_t handler_s0_connectc1withwill(void)  { return S1; }
static uint8_t handler_s0_publishqos0c2(void)      { return S0; }
static uint8_t handler_s0_publishqos1c1(void)      { return S0; }
static uint8_t handler_s0_subscribec1(void)        { return S0; }
static uint8_t handler_s0_unsubscribec1(void)      { return S0; }
static uint8_t handler_s0_subscribec2(void)        { return S0; }
static uint8_t handler_s0_unsubscribec2(void)      { return S0; }
static uint8_t handler_s0_disconnecttcpc1(void)    { return S0; }

static uint8_t handler_s1_connectc2(void)          { return S2; }
static uint8_t handler_s1_connectc1withwill(void)  { return S4; }
static uint8_t handler_s1_publishqos0c2(void)      { return S1; }
static uint8_t handler_s1_publishqos1c1(void)      { return S1; }
static uint8_t handler_s1_subscribec1(void)        { return S14; }
static uint8_t handler_s1_unsubscribec1(void)      { return S1; }
static uint8_t handler_s1_subscribec2(void)        { return S1; }
static uint8_t handler_s1_unsubscribec2(void)      { return S1; }
static uint8_t handler_s1_disconnecttcpc1(void)    { return S0; }

static uint8_t handler_s2_connectc2(void)          { return S8; }
static uint8_t handler_s2_connectc1withwill(void)  { return S5; }
static uint8_t handler_s2_publishqos0c2(void)      { return S2; }
static uint8_t handler_s2_publishqos1c1(void)      { return S2; }
static uint8_t handler_s2_subscribec1(void)        { return S11; }
static uint8_t handler_s2_unsubscribec1(void)      { return S2; }
static uint8_t handler_s2_subscribec2(void)        { return S6; }
static uint8_t handler_s2_unsubscribec2(void)      { return S2; }
static uint8_t handler_s2_disconnecttcpc1(void)    { return S3; }

static uint8_t handler_s3_connectc2(void)          { return S9; }
static uint8_t handler_s3_connectc1withwill(void)  { return S2; }
static uint8_t handler_s3_publishqos0c2(void)      { return S3; }
static uint8_t handler_s3_publishqos1c1(void)      { return S3; }
static uint8_t handler_s3_subscribec1(void)        { return S3; }
static uint8_t handler_s3_unsubscribec1(void)      { return S3; }
static uint8_t handler_s3_subscribec2(void)        { return S13; }
static uint8_t handler_s3_unsubscribec2(void)      { return S3; }
static uint8_t handler_s3_disconnecttcpc1(void)    { return S3; }

static uint8_t handler_s4_connectc2(void)          { return S5; }
static uint8_t handler_s4_connectc1withwill(void)  { return S1; }
static uint8_t handler_s4_publishqos0c2(void)      { return S4; }
static uint8_t handler_s4_publishqos1c1(void)      { return S4; }
static uint8_t handler_s4_subscribec1(void)        { return S4; }
static uint8_t handler_s4_unsubscribec1(void)      { return S4; }
static uint8_t handler_s4_subscribec2(void)        { return S4; }
static uint8_t handler_s4_unsubscribec2(void)      { return S4; }
static uint8_t handler_s4_disconnecttcpc1(void)    { return S0; }

static uint8_t handler_s5_connectc2(void)          { return S12; }
static uint8_t handler_s5_connectc1withwill(void)  { return S2; }
static uint8_t handler_s5_publishqos0c2(void)      { return S5; }
static uint8_t handler_s5_publishqos1c1(void)      { return S5; }
static uint8_t handler_s5_subscribec1(void)        { return S5; }
static uint8_t handler_s5_unsubscribec1(void)      { return S5; }
static uint8_t handler_s5_subscribec2(void)        { return S7; }
static uint8_t handler_s5_unsubscribec2(void)      { return S5; }
static uint8_t handler_s5_disconnecttcpc1(void)    { return S3; }

static uint8_t handler_s6_connectc2(void)          { return S8; }
static uint8_t handler_s6_connectc1withwill(void)  { return S7; }
static uint8_t handler_s6_publishqos0c2(void)      { return S6; }
static uint8_t handler_s6_publishqos1c1(void)      { return S6; }
static uint8_t handler_s6_subscribec1(void)        { return S10; }
static uint8_t handler_s6_unsubscribec1(void)      { return S6; }
static uint8_t handler_s6_subscribec2(void)        { return S6; }
static uint8_t handler_s6_unsubscribec2(void)      { return S2; }
static uint8_t handler_s6_disconnecttcpc1(void)    { return S13; }

static uint8_t handler_s7_connectc2(void)          { return S12; }
static uint8_t handler_s7_connectc1withwill(void)  { return S6; }
static uint8_t handler_s7_publishqos0c2(void)      { return S7; }
static uint8_t handler_s7_publishqos1c1(void)      { return S7; }
static uint8_t handler_s7_subscribec1(void)        { return S7; }
static uint8_t handler_s7_unsubscribec1(void)      { return S7; }
static uint8_t handler_s7_subscribec2(void)        { return S7; }
static uint8_t handler_s7_unsubscribec2(void)      { return S5; }
static uint8_t handler_s7_disconnecttcpc1(void)    { return S13; }

static uint8_t handler_s8_connectc2(void)          { return S2; }
static uint8_t handler_s8_connectc1withwill(void)  { return S12; }
static uint8_t handler_s8_publishqos0c2(void)      { return S8; }
static uint8_t handler_s8_publishqos1c1(void)      { return S8; }
static uint8_t handler_s8_subscribec1(void)        { return S15; }
static uint8_t handler_s8_unsubscribec1(void)      { return S8; }
static uint8_t handler_s8_subscribec2(void)        { return S8; }
static uint8_t handler_s8_unsubscribec2(void)      { return S8; }
static uint8_t handler_s8_disconnecttcpc1(void)    { return S9; }

static uint8_t handler_s9_connectc2(void)          { return S3; }
static uint8_t handler_s9_connectc1withwill(void)  { return S8; }
static uint8_t handler_s9_publishqos0c2(void)      { return S9; }
static uint8_t handler_s9_publishqos1c1(void)      { return S9; }
static uint8_t handler_s9_subscribec1(void)        { return S9; }
static uint8_t handler_s9_unsubscribec1(void)      { return S9; }
static uint8_t handler_s9_subscribec2(void)        { return S9; }
static uint8_t handler_s9_unsubscribec2(void)      { return S9; }
static uint8_t handler_s9_disconnecttcpc1(void)    { return S9; }

static uint8_t handler_s10_connectc2(void)         { return S15; }
static uint8_t handler_s10_connectc1withwill(void) { return S7; }
static uint8_t handler_s10_publishqos0c2(void)     { return S10; }
static uint8_t handler_s10_publishqos1c1(void)     { return S10; }
static uint8_t handler_s10_subscribec1(void)       { return S10; }
static uint8_t handler_s10_unsubscribec1(void)     { return S6; }
static uint8_t handler_s10_subscribec2(void)       { return S10; }
static uint8_t handler_s10_unsubscribec2(void)     { return S11; }
static uint8_t handler_s10_disconnecttcpc1(void)   { return S13; }

static uint8_t handler_s11_connectc2(void)         { return S15; }
static uint8_t handler_s11_connectc1withwill(void) { return S5; }
static uint8_t handler_s11_publishqos0c2(void)     { return S11; }
static uint8_t handler_s11_publishqos1c1(void)     { return S11; }
static uint8_t handler_s11_subscribec1(void)       { return S11; }
static uint8_t handler_s11_unsubscribec1(void)     { return S2; }
static uint8_t handler_s11_subscribec2(void)       { return S10; }
static uint8_t handler_s11_unsubscribec2(void)     { return S11; }
static uint8_t handler_s11_disconnecttcpc1(void)   { return S3; }

static uint8_t handler_s12_connectc2(void)         { return S5; }
static uint8_t handler_s12_connectc1withwill(void) { return S8; }
static uint8_t handler_s12_publishqos0c2(void)     { return S12; }
static uint8_t handler_s12_publishqos1c1(void)     { return S12; }
static uint8_t handler_s12_subscribec1(void)       { return S12; }
static uint8_t handler_s12_unsubscribec1(void)     { return S12; }
static uint8_t handler_s12_subscribec2(void)       { return S12; }
static uint8_t handler_s12_unsubscribec2(void)     { return S12; }
static uint8_t handler_s12_disconnecttcpc1(void)   { return S9; }

static uint8_t handler_s13_connectc2(void)         { return S9; }
static uint8_t handler_s13_connectc1withwill(void) { return S6; }
static uint8_t handler_s13_publishqos0c2(void)     { return S13; }
static uint8_t handler_s13_publishqos1c1(void)     { return S13; }
static uint8_t handler_s13_subscribec1(void)       { return S13; }
static uint8_t handler_s13_unsubscribec1(void)     { return S13; }
static uint8_t handler_s13_subscribec2(void)       { return S13; }
static uint8_t handler_s13_unsubscribec2(void)     { return S3; }
static uint8_t handler_s13_disconnecttcpc1(void)   { return S13; }

static uint8_t handler_s14_connectc2(void)         { return S11; }
static uint8_t handler_s14_connectc1withwill(void) { return S4; }
static uint8_t handler_s14_publishqos0c2(void)     { return S14; }
static uint8_t handler_s14_publishqos1c1(void)     { return S14; }
static uint8_t handler_s14_subscribec1(void)       { return S14; }
static uint8_t handler_s14_unsubscribec1(void)     { return S1; }
static uint8_t handler_s14_subscribec2(void)       { return S14; }
static uint8_t handler_s14_unsubscribec2(void)     { return S14; }
static uint8_t handler_s14_disconnecttcpc1(void)   { return S0; }

static uint8_t handler_s15_connectc2(void)         { return S11; }
static uint8_t handler_s15_connectc1withwill(void) { return S12; }
static uint8_t handler_s15_publishqos0c2(void)     { return S15; }
static uint8_t handler_s15_publishqos1c1(void)     { return S15; }
static uint8_t handler_s15_subscribec1(void)       { return S15; }
static uint8_t handler_s15_unsubscribec1(void)     { return S8; }
static uint8_t handler_s15_subscribec2(void)       { return S15; }
static uint8_t handler_s15_unsubscribec2(void)     { return S15; }
static uint8_t handler_s15_disconnecttcpc1(void)   { return S9; }

typedef struct {
    uint8_t state;
    uint8_t event;
    EventHandler handler; // pokazivac na funkciju, ne next_state konstanta
} Transition;

static const Transition transition_table[] = {
    { S0, ConnectC2, handler_s0_connectc2 },
    { S0, ConnectC1WithWill, handler_s0_connectc1withwill },
    { S0, PublishQoS0C2, handler_s0_publishqos0c2 },
    { S0, PublishQoS1C1, handler_s0_publishqos1c1 },
    { S0, SubscribeC1, handler_s0_subscribec1 },
    { S0, UnSubScribeC1, handler_s0_unsubscribec1 },
    { S0, SubscribeC2, handler_s0_subscribec2 },
    { S0, UnSubScribeC2, handler_s0_unsubscribec2 },
    { S0, DisconnectTCPC1, handler_s0_disconnecttcpc1 },

    { S1, ConnectC2, handler_s1_connectc2 },
    { S1, ConnectC1WithWill, handler_s1_connectc1withwill },
    { S1, PublishQoS0C2, handler_s1_publishqos0c2 },
    { S1, PublishQoS1C1, handler_s1_publishqos1c1 },
    { S1, SubscribeC1, handler_s1_subscribec1 },
    { S1, UnSubScribeC1, handler_s1_unsubscribec1 },
    { S1, SubscribeC2, handler_s1_subscribec2 },
    { S1, UnSubScribeC2, handler_s1_unsubscribec2 },
    { S1, DisconnectTCPC1, handler_s1_disconnecttcpc1 },

    { S2, ConnectC2, handler_s2_connectc2 },
    { S2, ConnectC1WithWill, handler_s2_connectc1withwill },
    { S2, PublishQoS0C2, handler_s2_publishqos0c2 },
    { S2, PublishQoS1C1, handler_s2_publishqos1c1 },
    { S2, SubscribeC1, handler_s2_subscribec1 },
    { S2, UnSubScribeC1, handler_s2_unsubscribec1 },
    { S2, SubscribeC2, handler_s2_subscribec2 },
    { S2, UnSubScribeC2, handler_s2_unsubscribec2 },
    { S2, DisconnectTCPC1, handler_s2_disconnecttcpc1 },

    { S3, ConnectC2, handler_s3_connectc2 },
    { S3, ConnectC1WithWill, handler_s3_connectc1withwill },
    { S3, PublishQoS0C2, handler_s3_publishqos0c2 },
    { S3, PublishQoS1C1, handler_s3_publishqos1c1 },
    { S3, SubscribeC1, handler_s3_subscribec1 },
    { S3, UnSubScribeC1, handler_s3_unsubscribec1 },
    { S3, SubscribeC2, handler_s3_subscribec2 },
    { S3, UnSubScribeC2, handler_s3_unsubscribec2 },
    { S3, DisconnectTCPC1, handler_s3_disconnecttcpc1 },

    { S4, ConnectC2, handler_s4_connectc2 },
    { S4, ConnectC1WithWill, handler_s4_connectc1withwill },
    { S4, PublishQoS0C2, handler_s4_publishqos0c2 },
    { S4, PublishQoS1C1, handler_s4_publishqos1c1 },
    { S4, SubscribeC1, handler_s4_subscribec1 },
    { S4, UnSubScribeC1, handler_s4_unsubscribec1 },
    { S4, SubscribeC2, handler_s4_subscribec2 },
    { S4, UnSubScribeC2, handler_s4_unsubscribec2 },
    { S4, DisconnectTCPC1, handler_s4_disconnecttcpc1 },

    { S5, ConnectC2, handler_s5_connectc2 },
    { S5, ConnectC1WithWill, handler_s5_connectc1withwill },
    { S5, PublishQoS0C2, handler_s5_publishqos0c2 },
    { S5, PublishQoS1C1, handler_s5_publishqos1c1 },
    { S5, SubscribeC1, handler_s5_subscribec1 },
    { S5, UnSubScribeC1, handler_s5_unsubscribec1 },
    { S5, SubscribeC2, handler_s5_subscribec2 },
    { S5, UnSubScribeC2, handler_s5_unsubscribec2 },
    { S5, DisconnectTCPC1, handler_s5_disconnecttcpc1 },

    { S6, ConnectC2, handler_s6_connectc2 },
    { S6, ConnectC1WithWill, handler_s6_connectc1withwill },
    { S6, PublishQoS0C2, handler_s6_publishqos0c2 },
    { S6, PublishQoS1C1, handler_s6_publishqos1c1 },
    { S6, SubscribeC1, handler_s6_subscribec1 },
    { S6, UnSubScribeC1, handler_s6_unsubscribec1 },
    { S6, SubscribeC2, handler_s6_subscribec2 },
    { S6, UnSubScribeC2, handler_s6_unsubscribec2 },
    { S6, DisconnectTCPC1, handler_s6_disconnecttcpc1 },

    { S7, ConnectC2, handler_s7_connectc2 },
    { S7, ConnectC1WithWill, handler_s7_connectc1withwill },
    { S7, PublishQoS0C2, handler_s7_publishqos0c2 },
    { S7, PublishQoS1C1, handler_s7_publishqos1c1 },
    { S7, SubscribeC1, handler_s7_subscribec1 },
    { S7, UnSubScribeC1, handler_s7_unsubscribec1 },
    { S7, SubscribeC2, handler_s7_subscribec2 },
    { S7, UnSubScribeC2, handler_s7_unsubscribec2 },
    { S7, DisconnectTCPC1, handler_s7_disconnecttcpc1 },

    { S8, ConnectC2, handler_s8_connectc2 },
    { S8, ConnectC1WithWill, handler_s8_connectc1withwill },
    { S8, PublishQoS0C2, handler_s8_publishqos0c2 },
    { S8, PublishQoS1C1, handler_s8_publishqos1c1 },
    { S8, SubscribeC1, handler_s8_subscribec1 },
    { S8, UnSubScribeC1, handler_s8_unsubscribec1 },
    { S8, SubscribeC2, handler_s8_subscribec2 },
    { S8, UnSubScribeC2, handler_s8_unsubscribec2 },
    { S8, DisconnectTCPC1, handler_s8_disconnecttcpc1 },

    { S9, ConnectC2, handler_s9_connectc2 },
    { S9, ConnectC1WithWill, handler_s9_connectc1withwill },
    { S9, PublishQoS0C2, handler_s9_publishqos0c2 },
    { S9, PublishQoS1C1, handler_s9_publishqos1c1 },
    { S9, SubscribeC1, handler_s9_subscribec1 },
    { S9, UnSubScribeC1, handler_s9_unsubscribec1 },
    { S9, SubscribeC2, handler_s9_subscribec2 },
    { S9, UnSubScribeC2, handler_s9_unsubscribec2 },
    { S9, DisconnectTCPC1, handler_s9_disconnecttcpc1 },

    { S10, ConnectC2, handler_s10_connectc2 },
    { S10, ConnectC1WithWill, handler_s10_connectc1withwill },
    { S10, PublishQoS0C2, handler_s10_publishqos0c2 },
    { S10, PublishQoS1C1, handler_s10_publishqos1c1 },
    { S10, SubscribeC1, handler_s10_subscribec1 },
    { S10, UnSubScribeC1, handler_s10_unsubscribec1 },
    { S10, SubscribeC2, handler_s10_subscribec2 },
    { S10, UnSubScribeC2, handler_s10_unsubscribec2 },
    { S10, DisconnectTCPC1, handler_s10_disconnecttcpc1 },

    { S11, ConnectC2, handler_s11_connectc2 },
    { S11, ConnectC1WithWill, handler_s11_connectc1withwill },
    { S11, PublishQoS0C2, handler_s11_publishqos0c2 },
    { S11, PublishQoS1C1, handler_s11_publishqos1c1 },
    { S11, SubscribeC1, handler_s11_subscribec1 },
    { S11, UnSubScribeC1, handler_s11_unsubscribec1 },
    { S11, SubscribeC2, handler_s11_subscribec2 },
    { S11, UnSubScribeC2, handler_s11_unsubscribec2 },
    { S11, DisconnectTCPC1, handler_s11_disconnecttcpc1 },

    { S12, ConnectC2, handler_s12_connectc2 },
    { S12, ConnectC1WithWill, handler_s12_connectc1withwill },
    { S12, PublishQoS0C2, handler_s12_publishqos0c2 },
    { S12, PublishQoS1C1, handler_s12_publishqos1c1 },
    { S12, SubscribeC1, handler_s12_subscribec1 },
    { S12, UnSubScribeC1, handler_s12_unsubscribec1 },
    { S12, SubscribeC2, handler_s12_subscribec2 },
    { S12, UnSubScribeC2, handler_s12_unsubscribec2 },
    { S12, DisconnectTCPC1, handler_s12_disconnecttcpc1 },

    { S13, ConnectC2, handler_s13_connectc2 },
    { S13, ConnectC1WithWill, handler_s13_connectc1withwill },
    { S13, PublishQoS0C2, handler_s13_publishqos0c2 },
    { S13, PublishQoS1C1, handler_s13_publishqos1c1 },
    { S13, SubscribeC1, handler_s13_subscribec1 },
    { S13, UnSubScribeC1, handler_s13_unsubscribec1 },
    { S13, SubscribeC2, handler_s13_subscribec2 },
    { S13, UnSubScribeC2, handler_s13_unsubscribec2 },
    { S13, DisconnectTCPC1, handler_s13_disconnecttcpc1 },

    { S14, ConnectC2, handler_s14_connectc2 },
    { S14, ConnectC1WithWill, handler_s14_connectc1withwill },
    { S14, PublishQoS0C2, handler_s14_publishqos0c2 },
    { S14, PublishQoS1C1, handler_s14_publishqos1c1 },
    { S14, SubscribeC1, handler_s14_subscribec1 },
    { S14, UnSubScribeC1, handler_s14_unsubscribec1 },
    { S14, SubscribeC2, handler_s14_subscribec2 },
    { S14, UnSubScribeC2, handler_s14_unsubscribec2 },
    { S14, DisconnectTCPC1, handler_s14_disconnecttcpc1 },

    { S15, ConnectC2, handler_s15_connectc2 },
    { S15, ConnectC1WithWill, handler_s15_connectc1withwill },
    { S15, PublishQoS0C2, handler_s15_publishqos0c2 },
    { S15, PublishQoS1C1, handler_s15_publishqos1c1 },
    { S15, SubscribeC1, handler_s15_subscribec1 },
    { S15, UnSubScribeC1, handler_s15_unsubscribec1 },
    { S15, SubscribeC2, handler_s15_subscribec2 },
    { S15, UnSubScribeC2, handler_s15_unsubscribec2 },
    { S15, DisconnectTCPC1, handler_s15_disconnecttcpc1 }
};

#define TABLE_SIZE (sizeof(transition_table) / sizeof(Transition))

static uint8_t fsm_transition(uint8_t state, uint8_t event) {
    for (uint8_t i = 0; i < TABLE_SIZE; i++) {
        if (transition_table[i].state == state && transition_table[i].event == event) {
            return transition_table[i].handler(); // indirektni poziv - dodatni trosak
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

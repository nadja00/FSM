/*
 * Rad: Carlgren, J., Oskarsson, P. W. (2023). "State Machine Model-To-Code
 * Transformation In C." UPTEC F 23044, Uppsala University - sekcija 3.6
 * "Function Pointers" (Figure 8).
 *
 * Access Control FSM - Function Pointers Pattern (kako ga rad doslovno naziva).
 * FSM: 4 states - IDLE, CHECKING, GRANTED, DENIED
 * Events: EV_VALID, EV_INVALID, EV_TIMEOUT
 *
 * Razlika od fsm_indexed_table.cpp: tamo 2D niz cuva next_state VREDNOST
 * direktno (jedno citanje iz memorije). Ovde 2D niz cuva POKAZIVACE NA
 * FUNKCIJE (StateMachine[state][event] iz Fig. 8 rada) - svaka celija se
 * poziva, ukljucujuci i "no-op" celije za nevalidne kombinacije. Ovo dodaje
 * indirektni poziv funkcije koji indexed_table nema.
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
#define NUM_STATES 16

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

// 2D niz POKAZIVACA NA FUNKCIJE - svaka (state,event) kombinacija je popunjena
static const EventHandler transition_table[NUM_STATES][NUM_EVENTS] = {
    /*             ConnectC2        ConnectC1WithWill  PublishQoS0C2      PublishQoS1C1      SubscribeC1       UnSubScribeC1     SubscribeC2       UnSubScribeC2     DisconnectTCPC1  */
    /* S0    */ { handler_s0_connectc2, handler_s0_connectc1withwill, handler_s0_publishqos0c2, handler_s0_publishqos1c1, handler_s0_subscribec1, handler_s0_unsubscribec1, handler_s0_subscribec2, handler_s0_unsubscribec2, handler_s0_disconnecttcpc1 },
    /* S1    */ { handler_s1_connectc2, handler_s1_connectc1withwill, handler_s1_publishqos0c2, handler_s1_publishqos1c1, handler_s1_subscribec1, handler_s1_unsubscribec1, handler_s1_subscribec2, handler_s1_unsubscribec2, handler_s1_disconnecttcpc1 },
    /* S2    */ { handler_s2_connectc2, handler_s2_connectc1withwill, handler_s2_publishqos0c2, handler_s2_publishqos1c1, handler_s2_subscribec1, handler_s2_unsubscribec1, handler_s2_subscribec2, handler_s2_unsubscribec2, handler_s2_disconnecttcpc1 },
    /* S3    */ { handler_s3_connectc2, handler_s3_connectc1withwill, handler_s3_publishqos0c2, handler_s3_publishqos1c1, handler_s3_subscribec1, handler_s3_unsubscribec1, handler_s3_subscribec2, handler_s3_unsubscribec2, handler_s3_disconnecttcpc1 },
    /* S4    */ { handler_s4_connectc2, handler_s4_connectc1withwill, handler_s4_publishqos0c2, handler_s4_publishqos1c1, handler_s4_subscribec1, handler_s4_unsubscribec1, handler_s4_subscribec2, handler_s4_unsubscribec2, handler_s4_disconnecttcpc1 },
    /* S5    */ { handler_s5_connectc2, handler_s5_connectc1withwill, handler_s5_publishqos0c2, handler_s5_publishqos1c1, handler_s5_subscribec1, handler_s5_unsubscribec1, handler_s5_subscribec2, handler_s5_unsubscribec2, handler_s5_disconnecttcpc1 },
    /* S6    */ { handler_s6_connectc2, handler_s6_connectc1withwill, handler_s6_publishqos0c2, handler_s6_publishqos1c1, handler_s6_subscribec1, handler_s6_unsubscribec1, handler_s6_subscribec2, handler_s6_unsubscribec2, handler_s6_disconnecttcpc1 },
    /* S7    */ { handler_s7_connectc2, handler_s7_connectc1withwill, handler_s7_publishqos0c2, handler_s7_publishqos1c1, handler_s7_subscribec1, handler_s7_unsubscribec1, handler_s7_subscribec2, handler_s7_unsubscribec2, handler_s7_disconnecttcpc1 },
    /* S8    */ { handler_s8_connectc2, handler_s8_connectc1withwill, handler_s8_publishqos0c2, handler_s8_publishqos1c1, handler_s8_subscribec1, handler_s8_unsubscribec1, handler_s8_subscribec2, handler_s8_unsubscribec2, handler_s8_disconnecttcpc1 },
    /* S9    */ { handler_s9_connectc2, handler_s9_connectc1withwill, handler_s9_publishqos0c2, handler_s9_publishqos1c1, handler_s9_subscribec1, handler_s9_unsubscribec1, handler_s9_subscribec2, handler_s9_unsubscribec2, handler_s9_disconnecttcpc1 },
    /* S10   */ { handler_s10_connectc2, handler_s10_connectc1withwill, handler_s10_publishqos0c2, handler_s10_publishqos1c1, handler_s10_subscribec1, handler_s10_unsubscribec1, handler_s10_subscribec2, handler_s10_unsubscribec2, handler_s10_disconnecttcpc1 },
    /* S11   */ { handler_s11_connectc2, handler_s11_connectc1withwill, handler_s11_publishqos0c2, handler_s11_publishqos1c1, handler_s11_subscribec1, handler_s11_unsubscribec1, handler_s11_subscribec2, handler_s11_unsubscribec2, handler_s11_disconnecttcpc1 },
    /* S12   */ { handler_s12_connectc2, handler_s12_connectc1withwill, handler_s12_publishqos0c2, handler_s12_publishqos1c1, handler_s12_subscribec1, handler_s12_unsubscribec1, handler_s12_subscribec2, handler_s12_unsubscribec2, handler_s12_disconnecttcpc1 },
    /* S13   */ { handler_s13_connectc2, handler_s13_connectc1withwill, handler_s13_publishqos0c2, handler_s13_publishqos1c1, handler_s13_subscribec1, handler_s13_unsubscribec1, handler_s13_subscribec2, handler_s13_unsubscribec2, handler_s13_disconnecttcpc1 },
    /* S14   */ { handler_s14_connectc2, handler_s14_connectc1withwill, handler_s14_publishqos0c2, handler_s14_publishqos1c1, handler_s14_subscribec1, handler_s14_unsubscribec1, handler_s14_subscribec2, handler_s14_unsubscribec2, handler_s14_disconnecttcpc1 },
    /* S15   */ { handler_s15_connectc2, handler_s15_connectc1withwill, handler_s15_publishqos0c2, handler_s15_publishqos1c1, handler_s15_subscribec1, handler_s15_unsubscribec1, handler_s15_subscribec2, handler_s15_unsubscribec2, handler_s15_disconnecttcpc1 },
};

static inline uint8_t fsm_transition(uint8_t state, uint8_t event) {
    return transition_table[state][event](); // indeksiranje + indirektni poziv
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

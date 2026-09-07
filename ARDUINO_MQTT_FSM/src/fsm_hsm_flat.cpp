#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

// ---------- States & Events (identical enums to previous versions) ----------
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

#define NUM_STATES 16
#define NUM_EVENTS 9

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
    if (event == ConnectC2)
        return S3;
    if (event == ConnectC1WithWill)
        return S1;
    return S0; // default
}

static uint8_t state_s1_handle(uint8_t event)
{
    if (event == ConnectC2)
        return S2;
    if (event == ConnectC1WithWill)
        return S4;
    if (event == SubscribeC1)
        return S14;
    if (event == DisconnectTCPC1)
        return S0;
    return S1; // default
}

static uint8_t state_s2_handle(uint8_t event)
{
    if (event == ConnectC2)
        return S8;
    if (event == ConnectC1WithWill)
        return S5;
    if (event == SubscribeC1)
        return S11;
    if (event == SubscribeC2)
        return S6;
    if (event == DisconnectTCPC1)
        return S3;
    return S2; // default
}

static uint8_t state_s3_handle(uint8_t event)
{
    if (event == ConnectC2)
        return S9;
    if (event == ConnectC1WithWill)
        return S2;
    if (event == SubscribeC2)
        return S13;
    return S3; // default
}

static uint8_t state_s4_handle(uint8_t event)
{
    if (event == ConnectC2)
        return S5;
    if (event == ConnectC1WithWill)
        return S1;
    if (event == DisconnectTCPC1)
        return S0;
    return S4; // default
}

static uint8_t state_s5_handle(uint8_t event)
{
    if (event == ConnectC2)
        return S12;
    if (event == ConnectC1WithWill)
        return S2;
    if (event == SubscribeC2)
        return S7;
    if (event == DisconnectTCPC1)
        return S3;
    return S5; // default
}

static uint8_t state_s6_handle(uint8_t event)
{
    if (event == ConnectC2)
        return S8;
    if (event == ConnectC1WithWill)
        return S7;
    if (event == SubscribeC1)
        return S10;
    if (event == UnSubScribeC2)
        return S2;
    if (event == DisconnectTCPC1)
        return S13;
    return S6; // default
}

static uint8_t state_s7_handle(uint8_t event)
{
    if (event == ConnectC2)
        return S12;
    if (event == ConnectC1WithWill)
        return S6;
    if (event == UnSubScribeC2)
        return S5;
    if (event == DisconnectTCPC1)
        return S13;
    return S7; // default
}

static uint8_t state_s8_handle(uint8_t event)
{
    if (event == ConnectC2)
        return S2;
    if (event == ConnectC1WithWill)
        return S12;
    if (event == SubscribeC1)
        return S15;
    if (event == DisconnectTCPC1)
        return S9;
    return S8; // default
}

static uint8_t state_s9_handle(uint8_t event)
{
    if (event == ConnectC2)
        return S3;
    if (event == ConnectC1WithWill)
        return S8;
    return S9; // default
}

static uint8_t state_s10_handle(uint8_t event)
{
    if (event == ConnectC2)
        return S15;
    if (event == ConnectC1WithWill)
        return S7;
    if (event == UnSubScribeC1)
        return S6;
    if (event == UnSubScribeC2)
        return S11;
    if (event == DisconnectTCPC1)
        return S13;
    return S10; // default
}

static uint8_t state_s11_handle(uint8_t event)
{
    if (event == ConnectC2)
        return S15;
    if (event == ConnectC1WithWill)
        return S5;
    if (event == UnSubScribeC1)
        return S2;
    if (event == SubscribeC2)
        return S10;
    if (event == DisconnectTCPC1)
        return S3;
    return S11; // default
}

static uint8_t state_s12_handle(uint8_t event)
{
    if (event == ConnectC2)
        return S5;
    if (event == ConnectC1WithWill)
        return S8;
    if (event == DisconnectTCPC1)
        return S9;
    return S12; // default
}

static uint8_t state_s13_handle(uint8_t event)
{
    if (event == ConnectC2)
        return S9;
    if (event == ConnectC1WithWill)
        return S6;
    if (event == UnSubScribeC2)
        return S3;
    return S13; // default
}

static uint8_t state_s14_handle(uint8_t event)
{
    if (event == ConnectC2)
        return S11;
    if (event == ConnectC1WithWill)
        return S4;
    if (event == UnSubScribeC1)
        return S1;
    if (event == DisconnectTCPC1)
        return S0;
    return S14; // default
}

static uint8_t state_s15_handle(uint8_t event)
{
    if (event == ConnectC2)
        return S11;
    if (event == ConnectC1WithWill)
        return S12;
    if (event == UnSubScribeC1)
        return S8;
    if (event == DisconnectTCPC1)
        return S9;
    return S15; // default
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
    { state_s15_handle, -1 }
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

// ---------- Automatic benchmark ----------
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

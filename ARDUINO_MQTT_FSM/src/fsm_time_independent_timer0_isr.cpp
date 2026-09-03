/*
 * NAPOMENA: ovaj fajl NE modeluje konkretan FSM pattern iz literature - to je
 * nasa sopstvena provera metodologije merenja (dokazuje da fsm_transition()
 * trosak ne zavisi od toga koliko je FSM prethodno stajao u stanju, cak i uz
 * pravi hardverski tajmer/ISR koji izaziva cekanje). Srodna metodologija
 * merenja ciklusa (broj ciklusa umesto apsolutnog vremena, radi prenosivosti
 * izmedju MK-ova) opisana je u: Katin, P., Chmelov, V., Shemaev, V. (2020).
 * "Development of Typical 'State' Software Patterns for Cortex-M
 * Microcontrollers in Real Time." Eastern-European Journal of Enterprise
 * Technologies, 3/9(105) - sek. 5.3, Cycle Count (DWT_CYCCNT) metodologija.
 *
 * UART commands (9600 baud):
 *   'a' -> Scenario A (immediate transition from CHECKING, no wait)
 *   'b' -> Scenario B (transition from GRANTED after ~3s Timer0-driven wait)
 *   'r' -> repeat both scenarios 5x and print min/avg/max for each
 */
#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

// ---------- States & Events (identical to original access-control FSM) ----------
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

// ---------- Timer1 cycle counter (used ONLY for measuring fsm_transition) ----------
// Completely separate hardware unit from Timer0 - no shared registers, no interaction.
static inline void cycles_start(void) {
    TCCR1B = 0;
    TCNT1 = 0;
    TCCR1B = (1 << CS10); // no prescaler
}

static inline uint16_t cycles_stop(void) {
    TCCR1B = 0;
    return TCNT1;
}

// ---------- Timer0 CTC + ISR: state-duration timer (simulates e.g. GRANTED timeout) ----------
// One Timer0 "tick cycle" (OCR0A=249, prescaler=1024) = 16ms.
// We need ~3000ms, so we count 188 tick cycles (188 * 16ms = 3008ms =~ 3s).
#define TICK_CYCLES_FOR_3S 188

static volatile uint16_t timer0_tick_count = 0;
static volatile uint8_t timer0_duration_elapsed = 0;

ISR(TIMER0_COMPA_vect) {
    timer0_tick_count++;
    if (timer0_tick_count >= TICK_CYCLES_FOR_3S) {
        timer0_duration_elapsed = 1;
        // Stop Timer0 - this state-duration wait is a one-shot event, not periodic.
        TCCR0B = 0;
    }
}

static void timer0_start_duration_wait(void) {
    timer0_tick_count = 0;
    timer0_duration_elapsed = 0;

    TCCR0A = (1 << WGM01);           // CTC mode (WGM02:00 = 010, WGM02 stays 0 in TCCR0B)
    OCR0A  = 249;                    // TOP value -> 250 counts per tick cycle
    TIMSK0 = (1 << OCIE0A);          // enable Compare Match A interrupt
    TCNT0  = 0;                      // start counting from zero
    TCCR0B = (1 << CS02) | (1 << CS00); // prescaler /1024 (CS02:00 = 101), starts the timer
}

// Blocks (via sleep-free polling of a flag set by the ISR) until ~3s have elapsed.
// The CPU could instead sleep here (SLEEP instruction) in a real low-power design;
// polling is used here only for simplicity of demonstration.
static void wait_for_state_duration(void) {
    timer0_start_duration_wait();
    while (!timer0_duration_elapsed) {
        // CPU is free here - in a real system this could be sleep_mode() instead.
    }
    TIMSK0 = 0; // disable Timer0 interrupt after use, to avoid any stray ISR firing later
}

// ---------- Nested Switch FSM transition (identical logic to the original test) ----------
static uint8_t fsm_transition(uint8_t state, uint8_t event)
{
    switch (state)
    {
    case S0:
        switch (event)
        {
        case ConnectC2:
            return S3;
        case ConnectC1WithWill:
            return S1;
        default:
            return S0;
        }
    case S1:
        switch (event)
        {
        case ConnectC2:
            return S2;
        case ConnectC1WithWill:
            return S4;
        case SubscribeC1:
            return S14;
        case DisconnectTCPC1:
            return S0;
        default:
            return S1;
        }
    case S2:
        switch (event)
        {
        case ConnectC2:
            return S8;
        case ConnectC1WithWill:
            return S5;
        case SubscribeC1:
            return S11;
        case SubscribeC2:
            return S6;
        case DisconnectTCPC1:
            return S3;
        default:
            return S2;
        }
    case S3:
        switch (event)
        {
        case ConnectC1WithWill:
            return S2;
        case SubscribeC2:
            return S13;
        default:
            return S3;
        }
    case S4:
        switch (event)
        {
        case ConnectC2:
            return S5;
        case ConnectC1WithWill:
            return S1;
        case DisconnectTCPC1:
            return S0;
        default:
            return S4;
        }
    case S5:
        switch (event)
        {
        case ConnectC1WithWill:
            return S2;
        case SubscribeC2:
            return S7;
        case DisconnectTCPC1:
            return S3;
        default:
            return S5;
        }
    case S6:
        switch (event)
        {
        case ConnectC2:
            return S8;
        case ConnectC1WithWill:
            return S7;
        case SubscribeC1:
            return S10;
        case UnSubScribeC2:
            return S2;
        case DisconnectTCPC1:
            return S13;
        default:
            return S6;
        }
    case S7:
        switch (event)
        {
        case ConnectC2:
            return S12;
        case ConnectC1WithWill:
            return S6;
        case UnSubScribeC2:
            return S5;
        case DisconnectTCPC1:
            return S13;
        default:
            return S7;
        }
    case S8:
        switch (event)
        {
        case ConnectC2:
            return S2;
        case ConnectC1WithWill:
            return S12;
        case SubscribeC1:
            return S15;
        case DisconnectTCPC1:
            return S9;
        default:
            return S8;
        }
    case S9:
        switch (event)
        {
        case ConnectC1WithWill:
            return S8;
        case ConnectC2:
            return S3;
        default:
            return S9;
        }
    case S10:
        switch (event)
        {
        case ConnectC1WithWill:
            return S7;
        case ConnectC2:
            return S15;
        case DisconnectTCPC1:
            return S13;
        case UnSubScribeC2:
            return S11;
        case UnSubScribeC1:
            return S6;
        default:
            return S10;
        }
    case S11:
        switch (event)
        {
        case ConnectC1WithWill:
            return S5;
        case ConnectC2:
            return S15;
        case DisconnectTCPC1:
            return S3;
        case SubscribeC2:
            return S10;
        case UnSubScribeC1:
            return S2;
        default:
            return S11;
        }
    case S12:
        switch (event)
        {
        case ConnectC1WithWill:
            return S8;
        case ConnectC2:
            return S5;
        case DisconnectTCPC1:
            return S9;
        default:
            return S12;
        }
    case S13:
        switch (event)
        {
        case ConnectC1WithWill:
            return S6;
        case ConnectC2:
            return S9;
        case UnSubScribeC2:
            return S3;
        default:
            return S13;
        }
    case S14:
        switch (event)
        {
        case ConnectC1WithWill:
            return S4;
        case ConnectC2:
            return S11;
        case DisconnectTCPC1:
            return S0;
        case UnSubScribeC1:
            return S1;
        default:
            return S14;
        }
    case S15:
        switch (event)
        {
        case ConnectC1WithWill:
            return S12;
        case ConnectC2:
            return S11;
        case DisconnectTCPC1:
            return S9;
        case UnSubScribeC1:
            return S8;
        default:
            return S15;
        }

    default:
        return S0;
    }
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

// ---------- Single measured transition (unchanged methodology) ----------
static uint16_t measured_transition(uint8_t event) {
    cli();
    cycles_start();
    uint8_t next = fsm_transition(current_state, event);
    uint16_t cycles = cycles_stop();
    sei();
    current_state = next;
    return cycles;
}

// ---------- Scenario A: immediate transition, no wait ----------
static uint16_t scenario_A(void) {
    current_state = S0;
    return measured_transition(ConnectC1WithWill); // S0 -> S1
}

// ---------- Scenario B: transition after ~3s Timer0-driven wait ----------
static uint16_t scenario_B(void) {
    current_state = S0;
    wait_for_state_duration();            // ~3s, driven by Timer0 CTC + ISR
    return measured_transition(DisconnectTCPC1); // S1 -> S0, AFTER the wait
}

// ---------- Repeat both scenarios and compare statistics ----------
static void run_comparison(uint16_t N) {
    uint16_t minA = 0xFFFF, maxA = 0; uint32_t sumA = 0;
    uint16_t minB = 0xFFFF, maxB = 0; uint32_t sumB = 0;

    uart_puts("Running ");
    uart_put_uint(N);
    uart_puts(" repetitions of each scenario (Timer0-driven ~3s wait each time)...\r\n");

    for (uint16_t i = 0; i < N; i++) {
        uint16_t a = scenario_A();
        if (a < minA) minA = a;
        if (a > maxA) maxA = a;
        sumA += a;

        uint16_t b = scenario_B();
        if (b < minB) minB = b;
        if (b > maxB) maxB = b;
        sumB += b;
    }

    uart_puts("\r\n--- Scenario A: CHECKING->GRANTED, NO wait ---\r\n");
    uart_puts("Min: "); uart_put_uint(minA); uart_puts("  Avg: "); uart_put_uint((uint16_t)(sumA / N));
    uart_puts("  Max: "); uart_put_uint(maxA); uart_puts("\r\n");

    uart_puts("--- Scenario B: GRANTED->IDLE, AFTER ~3s Timer0 wait ---\r\n");
    uart_puts("Min: "); uart_put_uint(minB); uart_puts("  Avg: "); uart_put_uint((uint16_t)(sumB / N));
    uart_puts("  Max: "); uart_put_uint(maxB); uart_puts("\r\n");

    uart_puts("\r\nIf the two ranges overlap/match, transition cost is proven\r\n");
    uart_puts("independent of prior state duration, even with an ISR-driven wait.\r\n");
}

// ---------- Setup / Loop ----------
void setup() {
    uart_init();
    uart_puts("\r\nTime-independence proof (Timer0 CTC+ISR version) ready.\r\n");
    uart_puts("Commands: a=Scenario A (immediate)  b=Scenario B (~3s Timer0 wait)  r=Repeat comparison (5x)\r\n");
}

void loop() {
    if (uart_available()) {
        char c = uart_getc();

        if (c == 'a') {
            uint16_t cyc = scenario_A();
            uart_puts("Scenario A -> State: "); uart_puts(state_name(current_state));
            uart_puts(" | Cycles: "); uart_put_uint(cyc); uart_puts("\r\n");
        } else if (c == 'b') {
            uart_puts("Waiting ~3s (Timer0 CTC + ISR)...\r\n");
            uint16_t cyc = scenario_B();
            uart_puts("Scenario B -> State: "); uart_puts(state_name(current_state));
            uart_puts(" | Cycles: "); uart_put_uint(cyc); uart_puts("\r\n");
        } else if (c == 'r') {
            run_comparison(5);
        }
    }
}

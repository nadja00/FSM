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
    return measured_transition(LISTEN); // S0 -> S1
}

// ---------- Scenario B: transition after ~3s Timer0-driven wait ----------
static uint16_t scenario_B(void) {
    current_state = S0;
    wait_for_state_duration();            // ~3s, driven by Timer0 CTC + ISR
    return measured_transition(ACCEPT); // S1 -> S4, AFTER the wait
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
            uart_puts("Scenario A -> State: ");
            uart_puts(" | Cycles: "); uart_put_uint(cyc); uart_puts("\r\n");
        } else if (c == 'b') {
            uart_puts("Waiting ~3s (Timer0 CTC + ISR)...\r\n");
            uint16_t cyc = scenario_B();
            uart_puts("Scenario B -> State: ");
            uart_puts(" | Cycles: "); uart_put_uint(cyc); uart_puts("\r\n");
        } else if (c == 'r') {
            run_comparison(5);
        }
    }
}

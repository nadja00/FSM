/*
 * Rad: Carlgren, J., Oskarsson, P. W. (2023). "State Machine Model-To-Code
 * Transformation In C." UPTEC F 23044, Uppsala University - sekcija 2.8.1
 * "Nested Switch/If Statements" i sekcija 3.5 "Nested Switch".
 * Pominje se i u: Adamczyk, P. "The Anthology of the Finite State Machine
 * Design Patterns" (Introduction, "nested switch statements [vGB99]"); Kadam,
 * Jogalekar, Hembade (2023) "Model a Finite State Machine as a Construct in
 * Computer Programming" - FSM2Construct algoritam (Listing 2) koristi isti
 * switch(CurrentState) dispatch.
 *
 * SWITCH-CASE BREAK
 * FSM: 4 states - IDLE, CHECKING, GRANTED, DENIED
 * Events: EV_VALID, EV_INVALID, EV_TIMEOUT
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
#include <stdio.h>
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
        case ConnectC2:
            next_state = S3;
            break;
        case ConnectC1WithWill:
            next_state = S1;
            break;
        default:
            next_state = S0;
            break;
        }
        break;
    case S1:
        switch (event)
        {
        case ConnectC2:
            next_state =  S2;
            break;
        case ConnectC1WithWill:
            next_state =  S4;
            break;
        case SubscribeC1:
            next_state =  S14;
            break;
        case DisconnectTCPC1:
            next_state =  S0;
            break;
        default:
            next_state =  S1;
            break;
        }
        break;
    case S2:
        switch (event)
        {
        case ConnectC2:
            next_state =  S8;
            break;
        case ConnectC1WithWill:
            next_state =  S5;
            break;
        case SubscribeC1:
            next_state =  S11;
            break;
        case SubscribeC2:
            next_state =  S6;
            break;
        case DisconnectTCPC1:
            next_state =  S3;
            break;
        default:
            next_state =  S2;
            break;
        }
        break;
    case S3:
        switch (event)
        {
        case ConnectC2:
            next_state =  S9;
            break;
        case ConnectC1WithWill:
            next_state =  S2;
            break;
        case SubscribeC2:
            next_state =  S13;
            break;
        default:
            next_state =  S3;
            break;
        }
        break;
    case S4:
        switch (event)
        {
        case ConnectC2:
            next_state =  S5;
            break;
        case ConnectC1WithWill:
            next_state =  S1;
            break;
        case DisconnectTCPC1:
            next_state =  S0;
            break;
        default:
            next_state =  S4;
            break;
        }
        break;
    case S5:
        switch (event)
        {
        case ConnectC2:
            next_state =  S12;
            break;  
        case ConnectC1WithWill:
            next_state =  S2;
            break;
        case SubscribeC2:
            next_state =  S7;
            break;
        case DisconnectTCPC1:
            next_state =  S3;
            break;
        default:
            next_state =  S5;
            break;
        }
        break;
    case S6:
        switch (event)
        {
        case ConnectC2:
            next_state =  S8;
            break;
        case ConnectC1WithWill:
            next_state =  S7;
            break;
        case SubscribeC1:
            next_state =  S10;
            break;
        case UnSubScribeC2:
            next_state =  S2;
            break;
        case DisconnectTCPC1:
            next_state =  S13;
            break;
        default:
            next_state =  S6;
            break;
        }
        break;  
    case S7:
        switch (event)
        {
        case ConnectC2:
            next_state =  S12;
            break;
        case ConnectC1WithWill:
            next_state =  S6;
            break;
        case UnSubScribeC2:
            next_state =  S5;
            break;
        case DisconnectTCPC1:
            next_state =  S13;
            break;
        default:
            next_state =  S7;
            break;
        }
        break;
    case S8:
        switch (event)
        {
        case ConnectC2:
            next_state =  S2;
            break;
        case ConnectC1WithWill:
            next_state =  S12;
            break;
        case SubscribeC1:
            next_state =  S15;
            break;
        case DisconnectTCPC1:
            next_state =  S9;
            break;
        default:
            next_state =  S8;
            break;
        }
        break;
    case S9:
        switch (event)
        {
        case ConnectC1WithWill:
            next_state =  S8;
            break;
        case ConnectC2:
            next_state =  S3;
            break;
        default:
            next_state =  S9;
            break;
        }
        break;
    case S10:
        switch (event)
        {
        case ConnectC1WithWill:
            next_state =  S7;
            break;
        case ConnectC2:
            next_state =  S15;
            break;
        case DisconnectTCPC1:
            next_state =  S13;
            break;
        case UnSubScribeC2:
            next_state =  S11;
            break;
        case UnSubScribeC1:
            next_state =  S6;
            break;
        default:
            next_state =  S10;
            break;
        }
        break;      
    case S11:
        switch (event)
        {
        case ConnectC1WithWill:
            next_state =  S5;
            break;
        case ConnectC2:
            next_state =  S15;
            break;
        case DisconnectTCPC1:
            next_state =  S3;
            break;
        case SubscribeC2:
            next_state =  S10;
            break;
        case UnSubScribeC1:
            next_state =  S2;
            break;
        default:
            next_state =  S11;
            break;
        }
        break;
    case S12:
        switch (event)
        {
        case ConnectC1WithWill:
            next_state =  S8;
            break;
        case ConnectC2:
            next_state =  S5;
            break;
        case DisconnectTCPC1:
            next_state =  S9;
            break;
        default:
            next_state =  S12;
            break;
        }
        break;
    case S13:
        switch (event)
        {
        case ConnectC1WithWill:
            next_state =  S6;
            break;
        case ConnectC2:
            next_state =  S9;
            break;
        case UnSubScribeC2:
            next_state =  S3;
            break;
        default:
            next_state =  S13;
            break;
        }
        break;
    case S14:
        switch (event)
        {
        case ConnectC1WithWill:
            next_state =  S4;
            break;
        case ConnectC2:
            next_state =  S11;
            break;
        case DisconnectTCPC1:
            next_state =  S0;
            break;
        case UnSubScribeC1:
            next_state =  S1;
            break;
        default:
            next_state =  S14;
            break;
        }
        break;  
    case S15:
        switch (event)
        {
        case ConnectC1WithWill:
            next_state =  S12;
            break;
        case ConnectC2:
            next_state =  S11;
            break;
        case DisconnectTCPC1:
            next_state =  S9;
            break;
        case UnSubScribeC1:
            next_state =  S8;
            break;
        default:
            next_state =  S15;
            break;
        }
        break;
    default:
        next_state = S0;
        break;
    }

    return next_state;
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
        uart_puts(state_name(current_state));
        uart_puts(" | Cycles: ");
        uart_put_uint(cycles);
        uart_puts("\r\n");
    }
}

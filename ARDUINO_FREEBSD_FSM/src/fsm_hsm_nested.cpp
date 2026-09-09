/*
*
 * MQTT (mosquitto, dva klijenta) FSM - Manual HSM Pattern, Test 4b: NESTED
 * (real hierarchy), 16 stanja, 9 dogadjaja.
 *
 * DisconnectTCPC1 dogadjaj ponasa identicno za grupe od po 3 stanja - svaka grupa se vraca
 * u isto, vec postojece "root" stanje te grupe (koje je za DisconnectTCPC1
 * samo-petlja). Umesto da se ta ista tranzicija ponavlja u sva 3 handlera
 * svake grupe, definisana je JEDNOM na roditeljskom stanju, a deca je
 * nasledjuju kroz bubbling:
 *
 *   S0  (roditelj) <- S1, S4, S14   (DisconnectTCPC1 -> S0, definisano samo na S0)
 *   S3  (roditelj) <- S2, S5, S11   (DisconnectTCPC1 -> S3, definisano samo na S3)
 *   S13 (roditelj) <- S6, S7, S10   (DisconnectTCPC1 -> S13, definisano samo na S13)
 *   S9  (roditelj) <- S8, S12, S15  (DisconnectTCPC1 -> S9, definisano samo na S9)
 *
 * Nijedno novo, vestacko stanje nije uvedeno - S0/S3/S9/S13 su vec postojeca, roditeljska stanja svojih grupa.
 *
 * VAZNA NAPOMENA O DEFAULT PONASANJU: svako od 12 dece MORA eksplicitno da
 * obradi svih SVOJIH preostalih 8 dogadjaja (ukljucujuci sopstvene
 * samo-petlje), i vraca EVENT_UNHANDLED SAMO za DisconnectTCPC1. Da default
 * grana kod dece vraca EVENT_UNHANDLED za bilo koji drugi neobradjen
 * dogadjaj, ti dogadjaji bi se pogresno probublali ka roditelju cak i kada
 * roditeljevo ponasanje za njih NIJE isto kao kod deteta (npr. S1 na
 * PublishQoS0C2 ostaje S1, dok S0 na isti dogadjaj ostaje S0 - bubbling
 * bi pogresno prebacilo S1 -> S0).
 *
 *
 * UART commands (9600 baud):
 *   num(dec) -> event index (0-8, redosled kao u enum Event)
 *   'b' -> run automatic benchmark (1000 forced-bubble transitions), prints min/avg/max cycles
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
    S0, S1, S2, S3, S4, S5, S6, S7,
    S8, S9, S10, S11, S12, S13, S14, S15
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

#define EVENT_UNHANDLED 0xFF 

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
    while (i > 0) {
        uart_putc(buf[--i]);
    }
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

// ---------- HSM state descriptor ----------
typedef uint8_t (*StateHandler)(uint8_t event);

typedef struct {
    StateHandler handler;
    int8_t parent; // index roditeljskog stanja, ili -1 ako je top-level
} StateNode;

// ---------- Handleri (generisano i verifikovano protiv .dot specifikacije) ----------

static uint8_t state_s0_handle(uint8_t event)
{
    if (event == ConnectC2)         return S3;
    if (event == ConnectC1WithWill) return S1;
    if (event == PublishQoS0C2)     return S0;
    if (event == PublishQoS1C1)     return S0;
    if (event == SubscribeC1)       return S0;
    if (event == UnSubScribeC1)     return S0;
    if (event == SubscribeC2)       return S0;
    if (event == UnSubScribeC2)     return S0;
    if (event == DisconnectTCPC1)   return S0; // root za grupu {S1,S4,S14}
    return EVENT_UNHANDLED;
}

static uint8_t state_s1_handle(uint8_t event)
{
    if (event == ConnectC2)        
         return S2;
    if (event == ConnectC1WithWill) 
        return S4;
    if (event == PublishQoS0C2)     
        return S1;
    if (event == PublishQoS1C1)     
        return S1;
    if (event == SubscribeC1)       
        return S14;
    if (event == UnSubScribeC1)     
        return S1;
    if (event == SubscribeC2)       
        return S1;
    if (event == UnSubScribeC2)     
        return S1;
    return EVENT_UNHANDLED; // DisconnectTCPC1 -> bubble ka S0
}

static uint8_t state_s2_handle(uint8_t event)
{
    if (event == ConnectC2)         
         return S8;
    if (event == ConnectC1WithWill) 
        return S5;
    if (event == PublishQoS0C2)     
        return S2;
    if (event == PublishQoS1C1)    
         return S2;
    if (event == SubscribeC1)       
        return S11;
    if (event == UnSubScribeC1)     
        return S2;
    if (event == SubscribeC2)       
        return S6;
    if (event == UnSubScribeC2)     
        return S2;
    return EVENT_UNHANDLED; // DisconnectTCPC1 -> bubbling ka S3
}

static uint8_t state_s3_handle(uint8_t event)
{
    if (event == ConnectC2)         
        return S9;
    if (event == ConnectC1WithWill) 
        return S2;
    if (event == PublishQoS0C2)     
        return S3;
    if (event == PublishQoS1C1)     
        return S3;
    if (event == SubscribeC1)       
        return S3;
    if (event == UnSubScribeC1)     
        return S3;
    if (event == SubscribeC2)       
        return S13;
    if (event == UnSubScribeC2)     
        return S3;
    if (event == DisconnectTCPC1)   
        return S3; // root za grupu {S2,S5,S11}
    return EVENT_UNHANDLED;
}

static uint8_t state_s4_handle(uint8_t event)
{
    if (event == ConnectC2)         return S5;
    if (event == ConnectC1WithWill) return S1;
    if (event == PublishQoS0C2)     return S4;
    if (event == PublishQoS1C1)     return S4;
    if (event == SubscribeC1)       return S4;
    if (event == UnSubScribeC1)     return S4;
    if (event == SubscribeC2)       return S4;
    if (event == UnSubScribeC2)     return S4;
    return EVENT_UNHANDLED; // DisconnectTCPC1 -> bubbluje ka S0
}

static uint8_t state_s5_handle(uint8_t event)
{
    if (event == ConnectC2)         return S12;
    if (event == ConnectC1WithWill) return S2;
    if (event == PublishQoS0C2)     return S5;
    if (event == PublishQoS1C1)     return S5;
    if (event == SubscribeC1)       return S5;
    if (event == UnSubScribeC1)     return S5;
    if (event == SubscribeC2)       return S7;
    if (event == UnSubScribeC2)     return S5;
    return EVENT_UNHANDLED; // DisconnectTCPC1 -> bubbluje ka S3
}

static uint8_t state_s6_handle(uint8_t event)
{
    if (event == ConnectC2)         return S8;
    if (event == ConnectC1WithWill) return S7;
    if (event == PublishQoS0C2)     return S6;
    if (event == PublishQoS1C1)     return S6;
    if (event == SubscribeC1)       return S10;
    if (event == UnSubScribeC1)     return S6;
    if (event == SubscribeC2)       return S6;
    if (event == UnSubScribeC2)     return S2;
    return EVENT_UNHANDLED; // DisconnectTCPC1 -> bubbluje ka S13
}

static uint8_t state_s7_handle(uint8_t event)
{
    if (event == ConnectC2)         return S12;
    if (event == ConnectC1WithWill) return S6;
    if (event == PublishQoS0C2)     return S7;
    if (event == PublishQoS1C1)     return S7;
    if (event == SubscribeC1)       return S7;
    if (event == UnSubScribeC1)     return S7;
    if (event == SubscribeC2)       return S7;
    if (event == UnSubScribeC2)     return S5;
    return EVENT_UNHANDLED; // DisconnectTCPC1 -> bubbluje ka S13
}

static uint8_t state_s8_handle(uint8_t event)
{
    if (event == ConnectC2)         return S2;
    if (event == ConnectC1WithWill) return S12;
    if (event == PublishQoS0C2)     return S8;
    if (event == PublishQoS1C1)     return S8;
    if (event == SubscribeC1)       return S15;
    if (event == UnSubScribeC1)     return S8;
    if (event == SubscribeC2)       return S8;
    if (event == UnSubScribeC2)     return S8;
    return EVENT_UNHANDLED; // DisconnectTCPC1 -> bubbluje ka S9
}

static uint8_t state_s9_handle(uint8_t event)
{
    if (event == ConnectC2)         return S3;
    if (event == ConnectC1WithWill) return S8;
    if (event == PublishQoS0C2)     return S9;
    if (event == PublishQoS1C1)     return S9;
    if (event == SubscribeC1)       return S9;
    if (event == UnSubScribeC1)     return S9;
    if (event == SubscribeC2)       return S9;
    if (event == UnSubScribeC2)     return S9;
    if (event == DisconnectTCPC1)   return S9; // root za grupu {S8,S12,S15}
    return EVENT_UNHANDLED;
}

static uint8_t state_s10_handle(uint8_t event)
{
    if (event == ConnectC2)         return S15;
    if (event == ConnectC1WithWill) return S7;
    if (event == PublishQoS0C2)     return S10;
    if (event == PublishQoS1C1)     return S10;
    if (event == SubscribeC1)       return S10;
    if (event == UnSubScribeC1)     return S6;
    if (event == SubscribeC2)       return S10;
    if (event == UnSubScribeC2)     return S11;
    return EVENT_UNHANDLED; // DisconnectTCPC1 -> bubbluje ka S13
}

static uint8_t state_s11_handle(uint8_t event)
{
    if (event == ConnectC2)         return S15;
    if (event == ConnectC1WithWill) return S5;
    if (event == PublishQoS0C2)     return S11;
    if (event == PublishQoS1C1)     return S11;
    if (event == SubscribeC1)       return S11;
    if (event == UnSubScribeC1)     return S2;
    if (event == SubscribeC2)       return S10;
    if (event == UnSubScribeC2)     return S11;
    return EVENT_UNHANDLED; // DisconnectTCPC1 -> bubbluje ka S3
}

static uint8_t state_s12_handle(uint8_t event)
{
    if (event == ConnectC2)         return S5;
    if (event == ConnectC1WithWill) return S8;
    if (event == PublishQoS0C2)     return S12;
    if (event == PublishQoS1C1)     return S12;
    if (event == SubscribeC1)       return S12;
    if (event == UnSubScribeC1)     return S12;
    if (event == SubscribeC2)       return S12;
    if (event == UnSubScribeC2)     return S12;
    return EVENT_UNHANDLED; // DisconnectTCPC1 -> bubbluje ka S9
}

static uint8_t state_s13_handle(uint8_t event)
{
    if (event == ConnectC2)         return S9;
    if (event == ConnectC1WithWill) return S6;
    if (event == PublishQoS0C2)     return S13;
    if (event == PublishQoS1C1)     return S13;
    if (event == SubscribeC1)       return S13;
    if (event == UnSubScribeC1)     return S13;
    if (event == SubscribeC2)       return S13;
    if (event == UnSubScribeC2)     return S3;
    if (event == DisconnectTCPC1)   return S13; // root za grupu {S6,S7,S10}
    return EVENT_UNHANDLED;
}

static uint8_t state_s14_handle(uint8_t event)
{
    if (event == ConnectC2)         return S11;
    if (event == ConnectC1WithWill) return S4;
    if (event == PublishQoS0C2)     return S14;
    if (event == PublishQoS1C1)     return S14;
    if (event == SubscribeC1)       return S14;
    if (event == UnSubScribeC1)     return S1;
    if (event == SubscribeC2)       return S14;
    if (event == UnSubScribeC2)     return S14;
    return EVENT_UNHANDLED; // DisconnectTCPC1 -> bubbluje ka S0
}

static uint8_t state_s15_handle(uint8_t event)
{
    if (event == ConnectC2)         return S11;
    if (event == ConnectC1WithWill) return S12;
    if (event == PublishQoS0C2)     return S15;
    if (event == PublishQoS1C1)     return S15;
    if (event == SubscribeC1)       return S15;
    if (event == UnSubScribeC1)     return S8;
    if (event == SubscribeC2)       return S15;
    if (event == UnSubScribeC2)     return S15;
    return EVENT_UNHANDLED; // DisconnectTCPC1 -> bubbluje ka S9
}

// State table sa stvarnim parent/child odnosima 

static const StateNode state_table[NUM_STATES] = {
    { state_s0_handle,  -1 }, // S0  (root grupe A: S1,S4,S14)
    { state_s1_handle,   0 }, // S1  (dete od S0)
    { state_s2_handle,   3 }, // S2  (dete od S3)
    { state_s3_handle,  -1 }, // S3  (root grupe B: S2,S5,S11)
    { state_s4_handle,   0 }, // S4  (dete od S0)
    { state_s5_handle,   3 }, // S5  (dete od S3)
    { state_s6_handle,  13 }, // S6  (dete od S13)
    { state_s7_handle,  13 }, // S7  (dete od S13)
    { state_s8_handle,   9 }, // S8  (dete od S9)
    { state_s9_handle,  -1 }, // S9  (root grupe D: S8,S12,S15)
    { state_s10_handle, 13 }, // S10 (dete od S13)
    { state_s11_handle,  3 }, // S11 (dete od S3)
    { state_s12_handle,  9 }, // S12 (dete od S9)
    { state_s13_handle, -1 }, // S13 (root grupe C: S6,S7,S10)
    { state_s14_handle,  0 }, // S14 (dete od S0)
    { state_s15_handle,  9 }, // S15 (dete od S9)
};


static uint8_t fsm_transition(uint8_t state, uint8_t event) {
    int8_t node = state;
    uint8_t result;

    while (node != -1) {
        result = state_table[node].handler(event);
        if (result != EVENT_UNHANDLED) {
            return result;
        }
        node = state_table[node].parent;
    }
    return state; // niko u lancu nije obradio -> ostani u trenutnom stanju
}

static const char *state_name(uint8_t s)
{
    switch (s) {
        case S0:  
            return "S0";  
        case S1:  
            return "S1"; 
         case S2:  
            return "S2";  
        case S3: 
             return "S3";
        case S4: 
            return "S4";  
        case S5:  
            return "S5";  
        case S6:  
            return "S6";  
        case S7:  
            return "S7";
        case S8: 
         return "S8";  
         case S9:  
            return "S9";  
         case S10: 
            return "S10"; 
         case S11: 
            return "S11";
        case S12: 
            return "S12";  
        case S13: 
            return "S13";  
        case S14: 
            return "S14";  
        case S15: 
            return "S15";
        default:  
            return "UNKNOWN";
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

// Meri iyolovano probubljavanje: pre svake merene tranzicije stanje se
// prisilno postavi na jedno od 12 dece (rotacija kroz sva 4 childa iz sve 4
// grupe, da bi test pokrio sve grane hijerarhije podjednako), a primenjeni
// dogadjaj je uvek DisconnectTCPC1 - dogadjaj koji se kod svakog deteta
// obavezno probublava tacno jedan nivo do roditelja.
static void run_benchmark(void) {
    const uint16_t N = 1000;
    uint16_t min_c = 0xFFFF, max_c = 0;
    uint32_t sum_c = 0;
    static const uint8_t children[12] = { S1, S4, S14, S2, S5, S11, S6, S7, S10, S8, S12, S15 };

    uart_puts("Running benchmark (");
    uart_put_uint(N);
    uart_puts(" forced child+DisconnectTCPC1 bubble transitions)...\r\n");

    for (uint16_t i = 0; i < N; i++) {
        current_state = children[i % 12]; // rotiraj kroz svu decu, sve 4 grupe podjednako
        uint16_t c = measured_transition(DisconnectTCPC1);
        if (c < min_c) min_c = c;
        if (c > max_c) max_c = c;
        sum_c += c;
    }

    uart_puts("Min cycles: ");  uart_put_uint(min_c); uart_puts("\r\n");
    uart_puts("Max cycles: ");  uart_put_uint(max_c); uart_puts("\r\n");
    uart_puts("Avg cycles: ");  uart_put_uint((uint16_t)(sum_c / N)); uart_puts("\r\n");
    current_state = S0; // reset posle benchmarka
}

// ---------- Setup / Loop ----------
void setup()
{
    uart_init();
    uart_puts("\r\nMQTT FSM - Manual HSM (NESTED, Test 4b) ready.\r\n");
    uart_puts("Commands: num(dec 0-8)=EVENT b=BENCHMARK\r\n");
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
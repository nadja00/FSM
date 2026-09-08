/*
 *
 * MQTT (mosquitto, dva klijenta) FSM - Manual HSM Pattern, Test 4b: NESTED
 * (stvarna hijerarhija), 16 stanja, 9 dogadjaja - ESP32.
 *
 * DisconnectTCPC1 dogadjaj ponasa se identicno za grupe od po 3 stanja -
 * svaka grupa se vraca u isto, vec postojece "root" stanje te grupe (koje je
 * za DisconnectTCPC1 samo-petlja). Umesto da se ta ista tranzicija ponavlja
 * u sva 3 handlera svake grupe, definisana je JEDNOM na roditeljskom stanju,
 * a deca je nasledjuju kroz bubbling:
 *
 *   S0  (roditelj) <- S1, S4, S14   (DisconnectTCPC1 -> S0)
 *   S3  (roditelj) <- S2, S5, S11   (DisconnectTCPC1 -> S3)
 *   S13 (roditelj) <- S6, S7, S10   (DisconnectTCPC1 -> S13)
 *   S9  (roditelj) <- S8, S12, S15  (DisconnectTCPC1 -> S9)
 *
 * Nijedno novo, vestacko stanje nije uvedeno - S0/S3/S9/S13 su vec postojeca,
 * roditeljska stanja svojih grupa. Poredi se direktno sa fsm_hsm_flat_esp32.cpp.
 *
 * NAPOMENA O BENCHMARKU: za razliku od ostalih implementacija, ovaj benchmark
 * ne koristi rand()%NUM_EVENTS - dogadjaj je uvek fiksan (DisconnectTCPC1), a
 * stanje se pre svake merene tranzicije prisilno rotira kroz 12 dece (sva 4
 * roditelja podjednako), sto izoluje TACNO trosak jednog nivoa bubbling-a.
 * Ovo znaci da -flto artefakt (relevantan samo za AVR, videti napomenu u
 * ostalim fajlovima) ovde nikad nije bio pitanje, jer nema deljenja u
 * merenom putu.
 *
 * Serial commands (115200 baud):
 *   num(dec 0-8) -> event index (redosled kao u enum Event)
 *   'b' -> run automatic benchmark (1000 forced-bubble transitions), prints min/avg/max cycles + resource usage
 *   'r' -> print current resource usage (heap/stack/flash) on demand
 */

#include <Arduino.h>
#include <stdint.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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

static void print_resource_usage(void) {
    Serial.println();
    Serial.println(F("--- ESP32 resource usage ---"));
    Serial.printf("Free heap:      %u / %u bytes\r\n", ESP.getFreeHeap(), ESP.getHeapSize());
    Serial.printf("Min free heap:  %u bytes (worst case since boot)\r\n", ESP.getMinFreeHeap());
    Serial.printf("Max alloc heap: %u bytes (largest contiguous block)\r\n", ESP.getMaxAllocHeap());
    Serial.printf("Stack HWM:      %u bytes free (loop task)\r\n", (unsigned)(uxTaskGetStackHighWaterMark(NULL) * 4));
    Serial.printf("Sketch used:    %u bytes, free flash: %u bytes\r\n", ESP.getSketchSize(), ESP.getFreeSketchSpace());
    Serial.printf("CPU freq:       %u MHz\r\n", ESP.getCpuFreqMHz());
    Serial.println();
}

static inline uint32_t cycles_now(void) {
    return ESP.getCycleCount();
}

typedef uint8_t (*StateHandler)(uint8_t event);

typedef struct {
    StateHandler handler;
    int8_t parent;
} StateNode;

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
    if (event == ConnectC2)         return S2;
    if (event == ConnectC1WithWill) return S4;
    if (event == PublishQoS0C2)     return S1;
    if (event == PublishQoS1C1)     return S1;
    if (event == SubscribeC1)       return S14;
    if (event == UnSubScribeC1)     return S1;
    if (event == SubscribeC2)       return S1;
    if (event == UnSubScribeC2)     return S1;
    return EVENT_UNHANDLED; // DisconnectTCPC1 -> bubble ka S0
}

static uint8_t state_s2_handle(uint8_t event)
{
    if (event == ConnectC2)         return S8;
    if (event == ConnectC1WithWill) return S5;
    if (event == PublishQoS0C2)     return S2;
    if (event == PublishQoS1C1)     return S2;
    if (event == SubscribeC1)       return S11;
    if (event == UnSubScribeC1)     return S2;
    if (event == SubscribeC2)       return S6;
    if (event == UnSubScribeC2)     return S2;
    return EVENT_UNHANDLED; // DisconnectTCPC1 -> bubble ka S3
}

static uint8_t state_s3_handle(uint8_t event)
{
    if (event == ConnectC2)         return S9;
    if (event == ConnectC1WithWill) return S2;
    if (event == PublishQoS0C2)     return S3;
    if (event == PublishQoS1C1)     return S3;
    if (event == SubscribeC1)       return S3;
    if (event == UnSubScribeC1)     return S3;
    if (event == SubscribeC2)       return S13;
    if (event == UnSubScribeC2)     return S3;
    if (event == DisconnectTCPC1)   return S3; // root za grupu {S2,S5,S11}
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
    return EVENT_UNHANDLED; // DisconnectTCPC1 -> bubble ka S0
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
    return EVENT_UNHANDLED; // DisconnectTCPC1 -> bubble ka S3
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
    return EVENT_UNHANDLED; // DisconnectTCPC1 -> bubble ka S13
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
    return EVENT_UNHANDLED; // DisconnectTCPC1 -> bubble ka S13
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
    return EVENT_UNHANDLED; // DisconnectTCPC1 -> bubble ka S9
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
    return EVENT_UNHANDLED; // DisconnectTCPC1 -> bubble ka S13
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
    return EVENT_UNHANDLED; // DisconnectTCPC1 -> bubble ka S3
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
    return EVENT_UNHANDLED; // DisconnectTCPC1 -> bubble ka S9
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
    return EVENT_UNHANDLED; // DisconnectTCPC1 -> bubble ka S0
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
    return EVENT_UNHANDLED; // DisconnectTCPC1 -> bubble ka S9
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
    return state;
}

static const char *state_name(uint8_t s)
{
    switch(s) {
        case S0:  return "S0";  case S1:  return "S1";  case S2:  return "S2";  case S3:  return "S3";
        case S4:  return "S4";  case S5:  return "S5";  case S6:  return "S6";  case S7:  return "S7";
        case S8:  return "S8";  case S9:  return "S9";  case S10: return "S10"; case S11: return "S11";
        case S12: return "S12"; case S13: return "S13"; case S14: return "S14"; case S15: return "S15";
        default:  return "UNKNOWN";
    }
}

static uint32_t measured_transition(uint8_t event) {
    noInterrupts();
    uint32_t start = cycles_now();
    uint8_t next = fsm_transition(current_state, event);
    uint32_t end = cycles_now();
    interrupts();
    current_state = next;
    return end - start;
}

// Meri izolovano probubljavanje: pre svake merene tranzicije stanje se
// prisilno postavi na jedno od 12 dece (rotacija kroz sva 4 childa iz sve 4
// grupe), a primenjeni dogadjaj je uvek DisconnectTCPC1.
static void run_benchmark(void) {
    const uint16_t N = 1000;
    uint32_t min_c = 0xFFFFFFFF, max_c = 0;
    uint64_t sum_c = 0;
    static const uint8_t children[12] = { S1, S4, S14, S2, S5, S11, S6, S7, S10, S8, S12, S15 };

    Serial.printf("Running benchmark (%u forced child+DisconnectTCPC1 bubble transitions)...\r\n", N);

    for (uint16_t i = 0; i < N; i++) {
        current_state = children[i % 12];
        uint32_t c = measured_transition(DisconnectTCPC1);
        if (c < min_c) min_c = c;
        if (c > max_c) max_c = c;
        sum_c += c;
    }

    double mhz = ESP.getCpuFreqMHz();
    uint32_t avg_c = (uint32_t)(sum_c / N);
    Serial.printf("Min cycles: %u (%.3f us)\r\n", min_c, min_c / mhz);
    Serial.printf("Max cycles: %u (%.3f us)\r\n", max_c, max_c / mhz);
    Serial.printf("Avg cycles: %u (%.3f us)\r\n", avg_c, avg_c / mhz);

    print_resource_usage();
    current_state = S0; // reset posle benchmarka
}

void setup()
{
    Serial.begin(115200);
    delay(200);
    Serial.println();
    Serial.println(F("MQTT FSM - Manual HSM (NESTED, Test 4b) ready (ESP32)."));
    Serial.println(F("Commands: num(dec 0-8)=EVENT b=BENCHMARK r=RESOURCES"));
    Serial.println(F("Current state: S0"));
    print_resource_usage();

    srand(time(0));
}

void loop()
{
    if (Serial.available()) {
        char c = Serial.read();
        uint8_t event;

        if (c == 'b') {
            run_benchmark();
            return;
        } else if (c == 'r') {
            print_resource_usage();
            return;
        }

        event = atoi(&c);

        if (event < 0 || event >= NUM_EVENTS)
        {
            return;
        }

        uint32_t cycles = measured_transition(event);
        double mhz = ESP.getCpuFreqMHz();

        Serial.printf("Event handled -> State: %s | Cycles: %u (%.3f us)\r\n",
                      state_name(current_state), cycles, cycles / mhz);
    }
}

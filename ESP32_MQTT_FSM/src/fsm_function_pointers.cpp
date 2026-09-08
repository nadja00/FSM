/*
 *
 * MQTT (mosquitto, dva klijenta) FSM - Function Pointers Pattern - ESP32
 * 16 stanja, 9 dogadjaja. 2D niz POKAZIVACA NA FUNKCIJE, svaka celija se
 * poziva - jedan indirektni poziv po tranziciji.
 * Serial commands (115200 baud):
 *   num(dec 0-8) -> event index (redosled kao u enum Event)
 *   'b' -> run automatic benchmark (1000 transitions), prints min/avg/max cycles + resource usage
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

#define NUM_EVENTS 9
#define NUM_STATES 16

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

static void run_benchmark(void)
{
    const uint16_t N = 1000;
    uint32_t min_c = 0xFFFFFFFF, max_c = 0;
    uint64_t sum_c = 0;

    Serial.printf("Running benchmark (%u transitions)...\r\n", N);

    for (uint16_t i = 0; i < N; i++)
    {
        uint8_t event = rand() % NUM_EVENTS;
        uint32_t c = measured_transition(event);
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
}

void setup()
{
    Serial.begin(115200);
    delay(200);
    Serial.println();
    Serial.println(F("MQTT FSM - Function Pointers pattern ready (ESP32)."));
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

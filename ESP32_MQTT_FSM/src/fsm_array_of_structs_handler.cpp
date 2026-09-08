/*
 *
 * MQTT (mosquitto, dva klijenta) FSM - Array of Structs Pattern, VERNIJA
 * varijanta prema radu - ESP32. 16 stanja, 9 dogadjaja, 144 tranzicije.
 *
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
    Serial.println(F("MQTT FSM - Array of Structs (handler) pattern ready (ESP32)."));
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

        // ISPRAVKA: originalni AVR fajl je ovde imao "&&" umesto "||" (uslov
        // nikad tacan zbog uint8_t tipa) - ovde je vec ispravno.
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

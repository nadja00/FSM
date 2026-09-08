/*
 * MQTT (mosquitto, dva klijenta) FSM - Manual HSM Pattern, Test 4a: FLAT
 * (bez prave hijerarhije), 16 stanja, 9 dogadjaja - ESP32.
 *
 * Koristi direktan rand()%NUM_EVENTS unutar merene petlje (bez unapred
 * generisanog niza) - kontrolnim merenjem potvrdjeno da Xtensa arhitektura
 * ne pati od -flto artefakta prisutnog na AVR-u.
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

#define NUM_STATES 16
#define NUM_EVENTS 9

#define EVENT_UNHANDLED 0xFF // sentinel: "ovaj handler ne obradjuje ovaj dogadjaj"

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
    int8_t parent; // index into state_table[], or -1 if no parent (top state)
} StateNode;

static uint8_t state_s0_handle(uint8_t event)
{
    if (event == ConnectC2) return S3;
    if (event == ConnectC1WithWill) return S1;
    return S0;
}
static uint8_t state_s1_handle(uint8_t event)
{
    if (event == ConnectC2) return S2;
    if (event == ConnectC1WithWill) return S4;
    if (event == SubscribeC1) return S14;
    if (event == DisconnectTCPC1) return S0;
    return S1;
}
static uint8_t state_s2_handle(uint8_t event)
{
    if (event == ConnectC2) return S8;
    if (event == ConnectC1WithWill) return S5;
    if (event == SubscribeC1) return S11;
    if (event == SubscribeC2) return S6;
    if (event == DisconnectTCPC1) return S3;
    return S2;
}
static uint8_t state_s3_handle(uint8_t event)
{
    if (event == ConnectC2) return S9;
    if (event == ConnectC1WithWill) return S2;
    if (event == SubscribeC2) return S13;
    return S3;
}
static uint8_t state_s4_handle(uint8_t event)
{
    if (event == ConnectC2) return S5;
    if (event == ConnectC1WithWill) return S1;
    if (event == DisconnectTCPC1) return S0;
    return S4;
}
static uint8_t state_s5_handle(uint8_t event)
{
    if (event == ConnectC2) return S12;
    if (event == ConnectC1WithWill) return S2;
    if (event == SubscribeC2) return S7;
    if (event == DisconnectTCPC1) return S3;
    return S5;
}
static uint8_t state_s6_handle(uint8_t event)
{
    if (event == ConnectC2) return S8;
    if (event == ConnectC1WithWill) return S7;
    if (event == SubscribeC1) return S10;
    if (event == UnSubScribeC2) return S2;
    if (event == DisconnectTCPC1) return S13;
    return S6;
}
static uint8_t state_s7_handle(uint8_t event)
{
    if (event == ConnectC2) return S12;
    if (event == ConnectC1WithWill) return S6;
    if (event == UnSubScribeC2) return S5;
    if (event == DisconnectTCPC1) return S13;
    return S7;
}
static uint8_t state_s8_handle(uint8_t event)
{
    if (event == ConnectC2) return S2;
    if (event == ConnectC1WithWill) return S12;
    if (event == SubscribeC1) return S15;
    if (event == DisconnectTCPC1) return S9;
    return S8;
}
static uint8_t state_s9_handle(uint8_t event)
{
    if (event == ConnectC2) return S3;
    if (event == ConnectC1WithWill) return S8;
    return S9;
}
static uint8_t state_s10_handle(uint8_t event)
{
    if (event == ConnectC2) return S15;
    if (event == ConnectC1WithWill) return S7;
    if (event == UnSubScribeC1) return S6;
    if (event == UnSubScribeC2) return S11;
    if (event == DisconnectTCPC1) return S13;
    return S10;
}
static uint8_t state_s11_handle(uint8_t event)
{
    if (event == ConnectC2) return S15;
    if (event == ConnectC1WithWill) return S5;
    if (event == UnSubScribeC1) return S2;
    if (event == SubscribeC2) return S10;
    if (event == DisconnectTCPC1) return S3;
    return S11;
}
static uint8_t state_s12_handle(uint8_t event)
{
    if (event == ConnectC2) return S5;
    if (event == ConnectC1WithWill) return S8;
    if (event == DisconnectTCPC1) return S9;
    return S12;
}
static uint8_t state_s13_handle(uint8_t event)
{
    if (event == ConnectC2) return S9;
    if (event == ConnectC1WithWill) return S6;
    if (event == UnSubScribeC2) return S3;
    return S13;
}
static uint8_t state_s14_handle(uint8_t event)
{
    if (event == ConnectC2) return S11;
    if (event == ConnectC1WithWill) return S4;
    if (event == UnSubScribeC1) return S1;
    if (event == DisconnectTCPC1) return S0;
    return S14;
}
static uint8_t state_s15_handle(uint8_t event)
{
    if (event == ConnectC2) return S11;
    if (event == ConnectC1WithWill) return S12;
    if (event == UnSubScribeC1) return S8;
    if (event == DisconnectTCPC1) return S9;
    return S15;
}

// State table: svi roditelji su -1 (NEMA prave hijerarhije u ovom testu)
static const StateNode state_table[NUM_STATES] = {
    { state_s0_handle, -1 }, { state_s1_handle, -1 }, { state_s2_handle, -1 }, { state_s3_handle, -1 },
    { state_s4_handle, -1 }, { state_s5_handle, -1 }, { state_s6_handle, -1 }, { state_s7_handle, -1 },
    { state_s8_handle, -1 }, { state_s9_handle, -1 }, { state_s10_handle, -1 }, { state_s11_handle, -1 },
    { state_s12_handle, -1 }, { state_s13_handle, -1 }, { state_s14_handle, -1 }, { state_s15_handle, -1 }
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
    Serial.println(F("MQTT FSM - Manual HSM (FLAT, Test 4a) ready (ESP32)."));
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

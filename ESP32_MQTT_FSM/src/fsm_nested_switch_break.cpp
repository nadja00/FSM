/*
 * MQTT (mosquitto, dva klijenta) FSM - Nested Switch (break) Pattern - ESP32
 * 16 stanja, 9 dogadjaja.
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

static uint8_t fsm_transition(uint8_t state, uint8_t event) {
    uint8_t next_state = S0;
    switch (state)
    {
    case S0:
        switch (event)
        {
        case ConnectC2:         next_state = S3; break;
        case ConnectC1WithWill: next_state = S1; break;
        default:                next_state = S0; break;
        }
        break;
    case S1:
        switch (event)
        {
        case ConnectC2:         next_state = S2;  break;
        case ConnectC1WithWill: next_state = S4;  break;
        case SubscribeC1:       next_state = S14; break;
        case DisconnectTCPC1:   next_state = S0;  break;
        default:                next_state = S1;  break;
        }
        break;
    case S2:
        switch (event)
        {
        case ConnectC2:         next_state = S8;  break;
        case ConnectC1WithWill: next_state = S5;  break;
        case SubscribeC1:       next_state = S11; break;
        case SubscribeC2:       next_state = S6;  break;
        case DisconnectTCPC1:   next_state = S3;  break;
        default:                next_state = S2;  break;
        }
        break;
    case S3:
        switch (event)
        {
        case ConnectC2:         next_state = S9;  break;
        case ConnectC1WithWill: next_state = S2;  break;
        case SubscribeC2:       next_state = S13; break;
        default:                next_state = S3;  break;
        }
        break;
    case S4:
        switch (event)
        {
        case ConnectC2:         next_state = S5; break;
        case ConnectC1WithWill: next_state = S1; break;
        case DisconnectTCPC1:   next_state = S0; break;
        default:                next_state = S4; break;
        }
        break;
    case S5:
        switch (event)
        {
        case ConnectC2:         next_state = S12; break;
        case ConnectC1WithWill: next_state = S2;  break;
        case SubscribeC2:       next_state = S7;  break;
        case DisconnectTCPC1:   next_state = S3;  break;
        default:                next_state = S5;  break;
        }
        break;
    case S6:
        switch (event)
        {
        case ConnectC2:         next_state = S8;  break;
        case ConnectC1WithWill: next_state = S7;  break;
        case SubscribeC1:       next_state = S10; break;
        case UnSubScribeC2:     next_state = S2;  break;
        case DisconnectTCPC1:   next_state = S13; break;
        default:                next_state = S6;  break;
        }
        break;
    case S7:
        switch (event)
        {
        case ConnectC2:         next_state = S12; break;
        case ConnectC1WithWill: next_state = S6;  break;
        case UnSubScribeC2:     next_state = S5;  break;
        case DisconnectTCPC1:   next_state = S13; break;
        default:                next_state = S7;  break;
        }
        break;
    case S8:
        switch (event)
        {
        case ConnectC2:         next_state = S2;  break;
        case ConnectC1WithWill: next_state = S12; break;
        case SubscribeC1:       next_state = S15; break;
        case DisconnectTCPC1:   next_state = S9;  break;
        default:                next_state = S8;  break;
        }
        break;
    case S9:
        switch (event)
        {
        case ConnectC1WithWill: next_state = S8; break;
        case ConnectC2:         next_state = S3; break;
        default:                next_state = S9; break;
        }
        break;
    case S10:
        switch (event)
        {
        case ConnectC1WithWill: next_state = S7;  break;
        case ConnectC2:         next_state = S15; break;
        case DisconnectTCPC1:   next_state = S13; break;
        case UnSubScribeC2:     next_state = S11; break;
        case UnSubScribeC1:     next_state = S6;  break;
        default:                next_state = S10; break;
        }
        break;
    case S11:
        switch (event)
        {
        case ConnectC1WithWill: next_state = S5;  break;
        case ConnectC2:         next_state = S15; break;
        case DisconnectTCPC1:   next_state = S3;  break;
        case SubscribeC2:       next_state = S10; break;
        case UnSubScribeC1:     next_state = S2;  break;
        default:                next_state = S11; break;
        }
        break;
    case S12:
        switch (event)
        {
        case ConnectC1WithWill: next_state = S8; break;
        case ConnectC2:         next_state = S5; break;
        case DisconnectTCPC1:   next_state = S9; break;
        default:                next_state = S12; break;
        }
        break;
    case S13:
        switch (event)
        {
        case ConnectC1WithWill: next_state = S6; break;
        case ConnectC2:         next_state = S9; break;
        case UnSubScribeC2:     next_state = S3; break;
        default:                next_state = S13; break;
        }
        break;
    case S14:
        switch (event)
        {
        case ConnectC1WithWill: next_state = S4;  break;
        case ConnectC2:         next_state = S11; break;
        case DisconnectTCPC1:   next_state = S0;  break;
        case UnSubScribeC1:     next_state = S1;  break;
        default:                next_state = S14; break;
        }
        break;
    case S15:
        switch (event)
        {
        case ConnectC1WithWill: next_state = S12; break;
        case ConnectC2:         next_state = S11; break;
        case DisconnectTCPC1:   next_state = S9;  break;
        case UnSubScribeC1:     next_state = S8;  break;
        default:                next_state = S15; break;
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

/*

static void run_benchmark(void) {
    const uint16_t N = 1000;
    uint32_t min_c = 0xFFFFFFFF, max_c = 0;
    uint64_t sum_c = 0;

    static uint8_t precomputed_events[N];
    for (uint16_t i = 0; i < N; i++) {
        precomputed_events[i] = rand() % NUM_EVENTS;
    }

    Serial.printf("Running benchmark (%u transitions)...\r\n", N);

    for (uint16_t i = 0; i < N; i++) {
        uint32_t c = measured_transition(precomputed_events[i]);
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
    */
static void run_benchmark(void) {
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

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println();
    Serial.println(F("MQTT FSM - Nested Switch (break) pattern ready (ESP32)."));
    Serial.println(F("Commands: num(dec 0-8)=EVENT b=BENCHMARK r=RESOURCES"));
    Serial.println(F("Current state: S0"));
    print_resource_usage();

    srand(time(0));
}

void loop() {
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

        if (event < 0 || event >= NUM_EVENTS) {
            return;
        }

        uint32_t cycles = measured_transition(event);
        double mhz = ESP.getCpuFreqMHz();

        Serial.printf("Event handled -> State: %s | Cycles: %u (%.3f us)\r\n",
                      state_name(current_state), cycles, cycles / mhz);
    }
}

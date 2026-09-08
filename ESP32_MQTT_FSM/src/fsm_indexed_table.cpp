/*
 * MQTT (mosquitto, dva klijenta) FSM - Indexed Table Pattern - ESP32
 * 16 stanja, 9 dogadjaja. Direktno 2D indeksiranje, O(1) po tranziciji.
 *
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
    S8, S9, S10, S11, S12, S13, S14, S15,
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

static const uint8_t transition_table[NUM_STATES][NUM_EVENTS] = {
    /*             ConnectC2 ConnectC1WithWill PublishQoS0C2 PublishQoS1C1 SubscribeC1 UnSubScribeC1 SubscribeC2 UnSubScribeC2 DisconnectTCPC1 */
    /* S0    */ { S3, S1, S0, S0, S0, S0, S0, S0, S0 },
    /* S1    */ { S2, S4, S1, S1, S14, S1, S1, S1, S0 },
    /* S2    */ { S8, S5, S2, S2, S11, S2, S6, S2, S3 },
    /* S3    */ { S9, S2, S3, S3, S3, S3, S13, S3, S3 },
    /* S4    */ { S5, S1, S4, S4, S4, S4, S4, S4, S0 },
    /* S5    */ { S12, S2, S5, S5, S5, S5, S7, S5, S3 },
    /* S6    */ { S8, S7, S6, S6, S10, S6, S6, S2, S13 },
    /* S7    */ { S12, S6, S7, S7, S7, S7, S7, S5, S13 },
    /* S8    */ { S2, S12, S8, S8, S15, S8, S8, S8, S9 },
    /* S9    */ { S3, S8, S9, S9, S9, S9, S9, S9, S9 },
    /* S10   */ { S15, S7, S10, S10, S10, S6, S10, S11, S13 },
    /* S11   */ { S15, S5, S11, S11, S11, S2, S10, S11, S3 },
    /* S12   */ { S5, S8, S12, S12, S12, S12, S12, S12, S9 },
    /* S13   */ { S9, S6, S13, S13, S13, S13, S13, S3, S13 },
    /* S14   */ { S11, S4, S14, S14, S14, S1, S14, S14, S0 },
    /* S15   */ { S11, S12, S15, S15, S15, S8, S15, S15, S9 },
};

static inline uint8_t fsm_transition(uint8_t state, uint8_t event) {
    return transition_table[state][event];
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

static void run_benchmark(void) {
    const uint16_t N = 1000;
    uint32_t min_c = 0xFFFFFFFF, max_c = 0;
    uint64_t sum_c = 0;

    Serial.printf("Running benchmark (%u transitions)...\r\n", N);

    for (uint16_t i = 0; i < N; i++) {
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
    Serial.println(F("MQTT FSM - Indexed Table pattern ready (ESP32)."));
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

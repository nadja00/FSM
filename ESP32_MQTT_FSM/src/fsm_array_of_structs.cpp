/*
 * MQTT (mosquitto, dva klijenta) FSM - Array of Structs Pattern - ESP32
 * 16 stanja, 9 dogadjaja, 144 tranzicije. Linearna pretraga kroz niz struktura.
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

typedef struct {
    uint8_t state;
    uint8_t event;
    uint8_t next_state;
} Transition;

static const Transition transition_table[] = {
    { S0, ConnectC2, S3 }, { S0, ConnectC1WithWill, S1 }, { S0, PublishQoS0C2, S0 },
    { S0, PublishQoS1C1, S0 }, { S0, SubscribeC1, S0 }, { S0, UnSubScribeC1, S0 },
    { S0, SubscribeC2, S0 }, { S0, UnSubScribeC2, S0 }, { S0, DisconnectTCPC1, S0 },

    { S1, ConnectC2, S2 }, { S1, ConnectC1WithWill, S4 }, { S1, PublishQoS0C2, S1 },
    { S1, PublishQoS1C1, S1 }, { S1, SubscribeC1, S14 }, { S1, UnSubScribeC1, S1 },
    { S1, SubscribeC2, S1 }, { S1, UnSubScribeC2, S1 }, { S1, DisconnectTCPC1, S0 },

    { S2, ConnectC2, S8 }, { S2, ConnectC1WithWill, S5 }, { S2, PublishQoS0C2, S2 },
    { S2, PublishQoS1C1, S2 }, { S2, SubscribeC1, S11 }, { S2, UnSubScribeC1, S2 },
    { S2, SubscribeC2, S6 }, { S2, UnSubScribeC2, S2 }, { S2, DisconnectTCPC1, S3 },

    { S3, ConnectC2, S9 }, { S3, ConnectC1WithWill, S2 }, { S3, PublishQoS0C2, S3 },
    { S3, PublishQoS1C1, S3 }, { S3, SubscribeC1, S3 }, { S3, UnSubScribeC1, S3 },
    { S3, SubscribeC2, S13 }, { S3, UnSubScribeC2, S3 }, { S3, DisconnectTCPC1, S3 },

    { S4, ConnectC2, S5 }, { S4, ConnectC1WithWill, S1 }, { S4, PublishQoS0C2, S4 },
    { S4, PublishQoS1C1, S4 }, { S4, SubscribeC1, S4 }, { S4, UnSubScribeC1, S4 },
    { S4, SubscribeC2, S4 }, { S4, UnSubScribeC2, S4 }, { S4, DisconnectTCPC1, S0 },

    { S5, ConnectC2, S12 }, { S5, ConnectC1WithWill, S2 }, { S5, PublishQoS0C2, S5 },
    { S5, PublishQoS1C1, S5 }, { S5, SubscribeC1, S5 }, { S5, UnSubScribeC1, S5 },
    { S5, SubscribeC2, S7 }, { S5, UnSubScribeC2, S5 }, { S5, DisconnectTCPC1, S3 },

    { S6, ConnectC2, S8 }, { S6, ConnectC1WithWill, S7 }, { S6, PublishQoS0C2, S6 },
    { S6, PublishQoS1C1, S6 }, { S6, SubscribeC1, S10 }, { S6, UnSubScribeC1, S6 },
    { S6, SubscribeC2, S6 }, { S6, UnSubScribeC2, S2 }, { S6, DisconnectTCPC1, S13 },

    { S7, ConnectC2, S12 }, { S7, ConnectC1WithWill, S6 }, { S7, PublishQoS0C2, S7 },
    { S7, PublishQoS1C1, S7 }, { S7, SubscribeC1, S7 }, { S7, UnSubScribeC1, S7 },
    { S7, SubscribeC2, S7 }, { S7, UnSubScribeC2, S5 }, { S7, DisconnectTCPC1, S13 },

    { S8, ConnectC2, S2 }, { S8, ConnectC1WithWill, S12 }, { S8, PublishQoS0C2, S8 },
    { S8, PublishQoS1C1, S8 }, { S8, SubscribeC1, S15 }, { S8, UnSubScribeC1, S8 },
    { S8, SubscribeC2, S8 }, { S8, UnSubScribeC2, S8 }, { S8, DisconnectTCPC1, S9 },

    { S9, ConnectC2, S3 }, { S9, ConnectC1WithWill, S8 }, { S9, PublishQoS0C2, S9 },
    { S9, PublishQoS1C1, S9 }, { S9, SubscribeC1, S9 }, { S9, UnSubScribeC1, S9 },
    { S9, SubscribeC2, S9 }, { S9, UnSubScribeC2, S9 }, { S9, DisconnectTCPC1, S9 },

    { S10, ConnectC2, S15 }, { S10, ConnectC1WithWill, S7 }, { S10, PublishQoS0C2, S10 },
    { S10, PublishQoS1C1, S10 }, { S10, SubscribeC1, S10 }, { S10, UnSubScribeC1, S6 },
    { S10, SubscribeC2, S10 }, { S10, UnSubScribeC2, S11 }, { S10, DisconnectTCPC1, S13 },

    { S11, ConnectC2, S15 }, { S11, ConnectC1WithWill, S5 }, { S11, PublishQoS0C2, S11 },
    { S11, PublishQoS1C1, S11 }, { S11, SubscribeC1, S11 }, { S11, UnSubScribeC1, S2 },
    { S11, SubscribeC2, S10 }, { S11, UnSubScribeC2, S11 }, { S11, DisconnectTCPC1, S3 },

    { S12, ConnectC2, S5 }, { S12, ConnectC1WithWill, S8 }, { S12, PublishQoS0C2, S12 },
    { S12, PublishQoS1C1, S12 }, { S12, SubscribeC1, S12 }, { S12, UnSubScribeC1, S12 },
    { S12, SubscribeC2, S12 }, { S12, UnSubScribeC2, S12 }, { S12, DisconnectTCPC1, S9 },

    { S13, ConnectC2, S9 }, { S13, ConnectC1WithWill, S6 }, { S13, PublishQoS0C2, S13 },
    { S13, PublishQoS1C1, S13 }, { S13, SubscribeC1, S13 }, { S13, UnSubScribeC1, S13 },
    { S13, SubscribeC2, S13 }, { S13, UnSubScribeC2, S3 }, { S13, DisconnectTCPC1, S13 },

    { S14, ConnectC2, S11 }, { S14, ConnectC1WithWill, S4 }, { S14, PublishQoS0C2, S14 },
    { S14, PublishQoS1C1, S14 }, { S14, SubscribeC1, S14 }, { S14, UnSubScribeC1, S1 },
    { S14, SubscribeC2, S14 }, { S14, UnSubScribeC2, S14 }, { S14, DisconnectTCPC1, S0 },

    { S15, ConnectC2, S11 }, { S15, ConnectC1WithWill, S12 }, { S15, PublishQoS0C2, S15 },
    { S15, PublishQoS1C1, S15 }, { S15, SubscribeC1, S15 }, { S15, UnSubScribeC1, S8 },
    { S15, SubscribeC2, S15 }, { S15, UnSubScribeC2, S15 }, { S15, DisconnectTCPC1, S9 },
};

#define TABLE_SIZE (sizeof(transition_table) / sizeof(Transition))

static uint8_t fsm_transition(uint8_t state, uint8_t event) {
    for (uint8_t i = 0; i < TABLE_SIZE; i++) {
        if (transition_table[i].state == state && transition_table[i].event == event) {
            return transition_table[i].next_state;
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
    Serial.println(F("MQTT FSM - Array of Structs pattern ready (ESP32)."));
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

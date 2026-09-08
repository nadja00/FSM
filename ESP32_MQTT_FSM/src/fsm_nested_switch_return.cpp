/*
 * MQTT (mosquitto, dva klijenta) FSM - Nested Switch (return) Pattern - ESP32
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

static uint8_t fsm_transition(uint8_t state, uint8_t event)
{
    switch (state)
    {
    case S0:
        switch (event)
        {
        case ConnectC2:         return S3;
        case ConnectC1WithWill: return S1;
        default:                return S0;
        }
    case S1:
        switch (event)
        {
        case ConnectC2:         return S2;
        case ConnectC1WithWill: return S4;
        case SubscribeC1:       return S14;
        case DisconnectTCPC1:   return S0;
        default:                return S1;
        }
    case S2:
        switch (event)
        {
        case ConnectC2:         return S8;
        case ConnectC1WithWill: return S5;
        case SubscribeC1:       return S11;
        case SubscribeC2:       return S6;
        case DisconnectTCPC1:   return S3;
        default:                return S2;
        }
    case S3:
        switch (event)
        {
        case ConnectC2:         return S9;
        case ConnectC1WithWill: return S2;
        case SubscribeC2:       return S13;
        default:                return S3;
        }
    case S4:
        switch (event)
        {
        case ConnectC2:         return S5;
        case ConnectC1WithWill: return S1;
        case DisconnectTCPC1:   return S0;
        default:                return S4;
        }
    case S5:
        switch (event)
        {
        case ConnectC2:         return S12;
        case ConnectC1WithWill: return S2;
        case SubscribeC2:       return S7;
        case DisconnectTCPC1:   return S3;
        default:                return S5;
        }
    case S6:
        switch (event)
        {
        case ConnectC2:         return S8;
        case ConnectC1WithWill: return S7;
        case SubscribeC1:       return S10;
        case UnSubScribeC2:     return S2;
        case DisconnectTCPC1:   return S13;
        default:                return S6;
        }
    case S7:
        switch (event)
        {
        case ConnectC2:         return S12;
        case ConnectC1WithWill: return S6;
        case UnSubScribeC2:     return S5;
        case DisconnectTCPC1:   return S13;
        default:                return S7;
        }
    case S8:
        switch (event)
        {
        case ConnectC2:         return S2;
        case ConnectC1WithWill: return S12;
        case SubscribeC1:       return S15;
        case DisconnectTCPC1:   return S9;
        default:                return S8;
        }
    case S9:
        switch (event)
        {
        case ConnectC1WithWill: return S8;
        case ConnectC2:         return S3;
        default:                return S9;
        }
    case S10:
        switch (event)
        {
        case ConnectC1WithWill: return S7;
        case ConnectC2:         return S15;
        case DisconnectTCPC1:   return S13;
        case UnSubScribeC2:     return S11;
        case UnSubScribeC1:     return S6;
        default:                return S10;
        }
    case S11:
        switch (event)
        {
        case ConnectC1WithWill: return S5;
        case ConnectC2:         return S15;
        case DisconnectTCPC1:   return S3;
        case SubscribeC2:       return S10;
        case UnSubScribeC1:     return S2;
        default:                return S11;
        }
    case S12:
        switch (event)
        {
        case ConnectC1WithWill: return S8;
        case ConnectC2:         return S5;
        case DisconnectTCPC1:   return S9;
        default:                return S12;
        }
    case S13:
        switch (event)
        {
        case ConnectC1WithWill: return S6;
        case ConnectC2:         return S9;
        case UnSubScribeC2:     return S3;
        default:                return S13;
        }
    case S14:
        switch (event)
        {
        case ConnectC1WithWill: return S4;
        case ConnectC2:         return S11;
        case DisconnectTCPC1:   return S0;
        case UnSubScribeC1:     return S1;
        default:                return S14;
        }
    case S15:
        switch (event)
        {
        case ConnectC1WithWill: return S12;
        case ConnectC2:         return S11;
        case DisconnectTCPC1:   return S9;
        case UnSubScribeC1:     return S8;
        default:                return S15;
        }
    default:
        return S0;
    }
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

static uint32_t measured_transition(uint8_t event)
{
    noInterrupts();
    uint32_t start = cycles_now();
    uint8_t next = fsm_transition(current_state, event);
    uint32_t end = cycles_now();
    interrupts();
    current_state = next;
    return end - start;
}
/*


static void run_benchmark(void)
{
    const uint16_t N = 1000;
    uint32_t min_c = 0xFFFFFFFF, max_c = 0;
    uint64_t sum_c = 0;

    static uint8_t precomputed_events[N];
    for (uint16_t i = 0; i < N; i++) {
        precomputed_events[i] = rand() % NUM_EVENTS;
    }

    Serial.printf("Running benchmark (%u transitions)...\r\n", N);

    for (uint16_t i = 0; i < N; i++)
    {
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


void setup()
{
    Serial.begin(115200);
    delay(200);
    Serial.println();
    Serial.println(F("MQTT FSM - Nested Switch (return) pattern ready (ESP32)."));
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

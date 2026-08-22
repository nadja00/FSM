/*
 * Rad: Adamczyk, P. "The Anthology of the Finite State Machine Design
 * Patterns" - "State DP [GHJV95, pp. 305]" (Gamma, Helm, Johnson, Vlissides,
 * "Design Patterns: Elements of Reusable Object-Oriented Software", 1995) -
 * temeljni State Design Pattern. Pominje se i u: Carlgren, Oskarsson (2023)
 * UPTEC F 23044, sek. 2.8.4 "State Pattern", sek. 3.8 "OOP in C" i sek. 3.9
 * "Basic State Pattern" (struct + function pointer emulacija klase, tacno
 * kao ovde).
 *
 * Access Control FSM - State Object Pattern (OOP-in-C / Basic State Pattern) - ESP32
 * Each state "owns" its own handler function; dispatch via a function-pointer
 * table indexed only by state (not by event).
 *
 * Serial commands (115200 baud):
 *   '1' -> EV_VALID
 *   '0' -> EV_INVALID
 *   't' -> EV_TIMEOUT
 *   'b' -> run automatic benchmark (1000 transitions), prints min/avg/max cycles + resource usage
 *   'r' -> print current resource usage (heap/stack/flash) on demand
 */

#include <Arduino.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

enum State { STATE_IDLE = 0, STATE_CHECKING, STATE_GRANTED, STATE_DENIED };
enum Event { EV_VALID = 0, EV_INVALID, EV_TIMEOUT };

#define NUM_STATES 4

static volatile uint8_t current_state = STATE_IDLE;

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

static uint8_t state_idle_handle(uint8_t event) {
    if (event == EV_VALID) return STATE_CHECKING;
    return STATE_IDLE;
}

static uint8_t state_checking_handle(uint8_t event) {
    if (event == EV_VALID)   return STATE_GRANTED;
    if (event == EV_INVALID) return STATE_DENIED;
    return STATE_CHECKING;
}

static uint8_t state_granted_handle(uint8_t event) {
    if (event == EV_TIMEOUT) return STATE_IDLE;
    return STATE_GRANTED;
}

static uint8_t state_denied_handle(uint8_t event) {
    if (event == EV_TIMEOUT) return STATE_IDLE;
    return STATE_DENIED;
}

typedef uint8_t (*StateHandler)(uint8_t event);

static StateHandler const state_handlers[NUM_STATES] = {
    state_idle_handle,
    state_checking_handle,
    state_granted_handle,
    state_denied_handle,
};

static inline uint8_t fsm_transition(uint8_t state, uint8_t event) {
    return state_handlers[state](event);
}

static const char* state_name(uint8_t s) {
    switch (s) {
        case STATE_IDLE:     return "IDLE";
        case STATE_CHECKING: return "CHECKING";
        case STATE_GRANTED:  return "GRANTED";
        case STATE_DENIED:   return "DENIED";
        default:             return "UNKNOWN";
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
    uint8_t ev_cycle[3] = { EV_VALID, EV_VALID, EV_TIMEOUT };

    Serial.printf("Running benchmark (%u transitions)...\r\n", N);

    for (uint16_t i = 0; i < N; i++) {
        uint8_t event = ev_cycle[i % 3];
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
    Serial.println(F("Access FSM - State Object pattern ready (ESP32)."));
    Serial.println(F("Commands: 1=VALID 0=INVALID t=TIMEOUT b=BENCHMARK r=RESOURCES"));
    Serial.println(F("Current state: IDLE"));
    print_resource_usage();
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
        } else if (c == '1') {
            event = EV_VALID;
        } else if (c == '0') {
            event = EV_INVALID;
        } else if (c == 't') {
            event = EV_TIMEOUT;
        } else {
            return;
        }

        uint32_t cycles = measured_transition(event);
        double mhz = ESP.getCpuFreqMHz();

        Serial.printf("Event handled -> State: %s | Cycles: %u (%.3f us)\r\n",
                      state_name(current_state), cycles, cycles / mhz);
    }
}

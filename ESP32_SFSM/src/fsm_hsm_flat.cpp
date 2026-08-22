/*
 * Rad: Harel, D. (1987). "Statecharts: a Visual Formalism for Complex
 * Systems." Science of Computer Programming, Vol. 8, pp. 231-274 - koncept
 * "bubbling" dogadjaja ka roditelju kad ga dete ne obradi. Mehanizam
 * dispecovanja (EVENT_UNHANDLED bubbling) odgovara "behavioral inheritance"
 * konceptu iz: Moreno, A., Valduvieco, J. "dFSM: Finite State Machines for
 * Embedded Systems" (poziva se na Samek-ov Quantum Framework).
 * NAPOMENA: ovde su SVI roditelji -1 (nema prave hijerarhije) - to je nasa
 * sopstvena kontrolna/bazna varijanta (Test 4a) da izmerimo cist trosak
 * bubbling mehanizma bez ikakve koristi od nasledjivanja ponasanja; koncept
 * "flat state machine" pominje i Carlgren & Oskarsson (2023) UPTEC F 23044,
 * sek. 2.1.
 *
 * Access Control FSM - Manual HSM Pattern, Test 4a: FLAT (no real hierarchy) - ESP32
 * Measurement: Xtensa CPU cycle counter via ESP.getCycleCount().
 *
 * This implements the general HSM dispatch mechanism (event bubbling to parent
 * state if the current state's handler does not consume the event). In THIS
 * test every state's parent is NULL (no real hierarchy) - measures the pure
 * overhead of the HSM dispatch mechanism with zero benefit from inheritance.
 * Compare directly with fsm_hsm_nested.cpp, where a real parent/child relationship exists.
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

enum State { STATE_IDLE = 0, STATE_CHECKING, STATE_GRANTED, STATE_DENIED, NUM_STATES };
enum Event { EV_VALID = 0, EV_INVALID, EV_TIMEOUT };

#define EVENT_UNHANDLED 0xFF

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

typedef uint8_t (*StateHandler)(uint8_t event);

typedef struct {
    StateHandler handler;
    int8_t parent; // index into state_table[], or -1 if no parent (top state)
} StateNode;

static uint8_t state_idle_handle(uint8_t event) {
    if (event == EV_VALID) return STATE_CHECKING;
    return EVENT_UNHANDLED;
}

static uint8_t state_checking_handle(uint8_t event) {
    if (event == EV_VALID)   return STATE_GRANTED;
    if (event == EV_INVALID) return STATE_DENIED;
    return EVENT_UNHANDLED;
}

static uint8_t state_granted_handle(uint8_t event) {
    if (event == EV_TIMEOUT) return STATE_IDLE;
    return EVENT_UNHANDLED;
}

static uint8_t state_denied_handle(uint8_t event) {
    if (event == EV_TIMEOUT) return STATE_IDLE;
    return EVENT_UNHANDLED;
}

static const StateNode state_table[NUM_STATES] = {
    { state_idle_handle,     -1 },
    { state_checking_handle, -1 },
    { state_granted_handle,  -1 },
    { state_denied_handle,   -1 },
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
    Serial.println(F("Access FSM - Manual HSM (FLAT, Test 4a) ready (ESP32)."));
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

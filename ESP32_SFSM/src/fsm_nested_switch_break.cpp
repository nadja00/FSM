/*
 * Rad: Carlgren, J., Oskarsson, P. W. (2023). "State Machine Model-To-Code
 * Transformation In C." UPTEC F 23044, Uppsala University - sekcija 2.8.1
 * "Nested Switch/If Statements" i sekcija 3.5 "Nested Switch".
 * Pominje se i u: Adamczyk, P. "The Anthology of the Finite State Machine
 * Design Patterns" (Introduction, "nested switch statements [vGB99]"); Kadam,
 * Jogalekar, Hembade (2023) "Model a Finite State Machine as a Construct in
 * Computer Programming" - FSM2Construct algoritam (Listing 2) koristi isti
 * switch(CurrentState) dispatch.
 *
 * Access Control FSM - Nested Switch (break) Pattern - ESP32
 * FSM: 4 states - IDLE, CHECKING, GRANTED, DENIED
 * Events: EV_VALID, EV_INVALID, EV_TIMEOUT
 *
 * Same as fsm_nested_switch_return.cpp but uses a `next_state` local variable
 * and `break` statements instead of directly `return`-ing from the switch.
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

static uint8_t fsm_transition(uint8_t state, uint8_t event) {
    uint8_t next_state = STATE_IDLE;
    switch (state) {
        case STATE_IDLE:
            switch (event) {
                case EV_VALID:   next_state = STATE_CHECKING; break;
                default:         next_state = STATE_IDLE;     break;
            }
            break;
        case STATE_CHECKING:
            switch (event) {
                case EV_VALID:   next_state = STATE_GRANTED;  break;
                case EV_INVALID: next_state = STATE_DENIED;   break;
                default:         next_state = STATE_CHECKING; break;
            }
            break;
        case STATE_GRANTED:
            switch (event) {
                case EV_TIMEOUT: next_state = STATE_IDLE;    break;
                default:         next_state = STATE_GRANTED; break;
            }
            break;
        case STATE_DENIED:
            switch (event) {
                case EV_TIMEOUT: next_state = STATE_IDLE;   break;
                default:         next_state = STATE_DENIED; break;
            }
            break;
        default:
            next_state = STATE_IDLE;
            break;
    }
    return next_state;
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
    Serial.println(F("Access FSM - Nested Switch (break) pattern ready (ESP32)."));
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

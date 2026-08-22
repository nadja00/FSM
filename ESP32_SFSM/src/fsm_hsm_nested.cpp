/*
 * Rad: Harel, D. (1987). "Statecharts: a Visual Formalism for Complex
 * Systems." Science of Computer Programming, Vol. 8, pp. 231-274 - hijerarhija
 * i composite state (vidi i Carlgren & Oskarsson (2023) UPTEC F 23044, sek.
 * 2.3.1 "Hierarchy and composite state" i sek. 2.8.6/3.11 "Hierarchical State
 * Pattern" po Sunitha & Samuel (2019), IEEE Access 7:8591-8608).
 * "Behavioral inheritance" (dete ne obradi dogadjaj -> automatski ga
 * nasledjuje roditelj) opisano i u: Moreno, A., Valduvieco, J. "dFSM: Finite
 * State Machines for Embedded Systems" (poziva se na Samek-ov Quantum
 * Framework). Za varijantu sa punim entry()/exit() lancem akcija (prava
 * Harel-ova statechart semantika), vidi fsm_hsm_statechart.cpp.
 *
 * Access Control FSM - Manual HSM Pattern, Test 4b: NESTED (real hierarchy) - ESP32
 * Measurement: Xtensa CPU cycle counter via ESP.getCycleCount().
 *
 * FSM: IDLE, CHECKING, DENIED (unchanged) + GRANTED is now a SUPERSTATE with two
 * substates: NORMAL_ACCESS and ADMIN_ACCESS.
 *
 *   GRANTED (super)
 *     - EV_TIMEOUT -> IDLE   (defined ONCE on the parent, inherited by both children)
 *     |-- NORMAL_ACCESS  -- EV_ADMIN -> ADMIN_ACCESS
 *     |-- ADMIN_ACCESS   -- EV_ADMIN -> NORMAL_ACCESS (toggle, just for demo)
 *
 * Serial commands (115200 baud):
 *   '1' -> EV_VALID
 *   '0' -> EV_INVALID
 *   't' -> EV_TIMEOUT
 *   'a' -> EV_ADMIN (toggle NORMAL_ACCESS <-> ADMIN_ACCESS while inside GRANTED)
 *   'b' -> run automatic benchmark (1000 transitions), measuring the INHERITED EV_TIMEOUT cost
 *   'r' -> print current resource usage (heap/stack/flash) on demand
 */

#include <Arduino.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

enum State {
    STATE_IDLE = 0,
    STATE_CHECKING,
    STATE_DENIED,
    STATE_GRANTED,        // superstate (has its own handler for EV_TIMEOUT only)
    STATE_NORMAL_ACCESS,  // substate of GRANTED
    STATE_ADMIN_ACCESS,   // substate of GRANTED
    NUM_STATES
};
enum Event { EV_VALID = 0, EV_INVALID, EV_TIMEOUT, EV_ADMIN };

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
    int8_t parent; // -1 = top state (no parent)
} StateNode;

static uint8_t state_idle_handle(uint8_t event) {
    if (event == EV_VALID) return STATE_CHECKING;
    return EVENT_UNHANDLED;
}

static uint8_t state_checking_handle(uint8_t event) {
    if (event == EV_VALID)   return STATE_NORMAL_ACCESS; // enter GRANTED via its default substate
    if (event == EV_INVALID) return STATE_DENIED;
    return EVENT_UNHANDLED;
}

static uint8_t state_denied_handle(uint8_t event) {
    if (event == EV_TIMEOUT) return STATE_IDLE;
    return EVENT_UNHANDLED;
}

// GRANTED superstate handler: owns the transition shared by BOTH substates.
static uint8_t state_granted_handle(uint8_t event) {
    if (event == EV_TIMEOUT) return STATE_IDLE;
    return EVENT_UNHANDLED;
}

// NORMAL_ACCESS substate: only handles what is specific to it (EV_ADMIN).
// EV_TIMEOUT is NOT handled here - it bubbles up to state_granted_handle().
static uint8_t state_normal_access_handle(uint8_t event) {
    if (event == EV_ADMIN) return STATE_ADMIN_ACCESS;
    return EVENT_UNHANDLED;
}

// ADMIN_ACCESS substate: same idea, its own specific behavior only.
static uint8_t state_admin_access_handle(uint8_t event) {
    if (event == EV_ADMIN) return STATE_NORMAL_ACCESS;
    return EVENT_UNHANDLED;
}

static const StateNode state_table[NUM_STATES] = {
    /* STATE_IDLE          */ { state_idle_handle,          -1 },
    /* STATE_CHECKING      */ { state_checking_handle,      -1 },
    /* STATE_DENIED        */ { state_denied_handle,        -1 },
    /* STATE_GRANTED       */ { state_granted_handle,       -1 },
    /* STATE_NORMAL_ACCESS */ { state_normal_access_handle, STATE_GRANTED }, // child of GRANTED
    /* STATE_ADMIN_ACCESS  */ { state_admin_access_handle,  STATE_GRANTED }, // child of GRANTED
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
        case STATE_IDLE:          return "IDLE";
        case STATE_CHECKING:      return "CHECKING";
        case STATE_DENIED:        return "DENIED";
        case STATE_GRANTED:       return "GRANTED";
        case STATE_NORMAL_ACCESS: return "NORMAL_ACCESS";
        case STATE_ADMIN_ACCESS:  return "ADMIN_ACCESS";
        default:                  return "UNKNOWN";
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

// Measures the INHERITED transition: force state=NORMAL_ACCESS, fire EV_TIMEOUT
// each time, isolating the bubbling overhead.
static void run_benchmark(void) {
    const uint16_t N = 1000;
    uint32_t min_c = 0xFFFFFFFF, max_c = 0;
    uint64_t sum_c = 0;

    Serial.printf("Running benchmark (%u transitions, forcing state=NORMAL_ACCESS, event=EV_TIMEOUT each time)...\r\n", N);

    for (uint16_t i = 0; i < N; i++) {
        current_state = STATE_NORMAL_ACCESS; // force substate before each measured bubble-up
        uint32_t c = measured_transition(EV_TIMEOUT);
        if (c < min_c) min_c = c;
        if (c > max_c) max_c = c;
        sum_c += c;
    }

    double mhz = ESP.getCpuFreqMHz();
    uint32_t avg_c = (uint32_t)(sum_c / N);
    Serial.printf("Min cycles: %u (%.3f us)\r\n", min_c, min_c / mhz);
    Serial.printf("Max cycles: %u (%.3f us)\r\n", max_c, max_c / mhz);
    Serial.printf("Avg cycles: %u (%.3f us)\r\n", avg_c, avg_c / mhz);
    current_state = STATE_IDLE; // reset after benchmark

    print_resource_usage();
}

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println();
    Serial.println(F("Access FSM - Manual HSM (NESTED, Test 4b) ready (ESP32)."));
    Serial.println(F("Commands: 1=VALID 0=INVALID t=TIMEOUT a=ADMIN_TOGGLE b=BENCHMARK r=RESOURCES"));
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
        } else if (c == 'a') {
            event = EV_ADMIN;
        } else {
            return;
        }

        uint32_t cycles = measured_transition(event);
        double mhz = ESP.getCpuFreqMHz();

        Serial.printf("Event handled -> State: %s | Cycles: %u (%.3f us)\r\n",
                      state_name(current_state), cycles, cycles / mhz);
    }
}

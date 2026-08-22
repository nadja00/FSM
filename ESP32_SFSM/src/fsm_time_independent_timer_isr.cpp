/*
 * NAPOMENA: ovaj fajl NE modeluje konkretan FSM pattern iz literature - to je
 * nasa sopstvena provera metodologije merenja (dokazuje da fsm_transition()
 * trosak ne zavisi od toga koliko je FSM prethodno stajao u stanju, cak i uz
 * pravi hardverski tajmer/ISR koji izaziva cekanje). Srodna metodologija
 * merenja ciklusa (broj ciklusa umesto apsolutnog vremena, radi prenosivosti
 * izmedju MK-ova) opisana je u: Katin, P., Chmelov, V., Shemaev, V. (2020).
 * "Development of Typical 'State' Software Patterns for Cortex-M
 * Microcontrollers in Real Time." Eastern-European Journal of Enterprise
 * Technologies, 3/9(105) - sek. 5.3, Cycle Count (DWT_CYCCNT) metodologija.
 *
 * Time-independence proof - ESP32 (esp_timer one-shot instead of AVR Timer0 CTC+ISR)
 *
 * Proves that fsm_transition() cost does not depend on how long the FSM stayed
 * in the previous state, even when that wait is driven by a real hardware/ISR timer.
 *
 * ESP-IDF `esp_timer` is a 64-bit microsecond timer, so the ~3s wait is done in a
 * single one-shot alarm (esp_timer_start_once) - no manual tick accumulation needed.
 *
 * Serial commands (115200 baud):
 *   'a' -> Scenario A (immediate transition from CHECKING, no wait)
 *   'b' -> Scenario B (transition from GRANTED after ~3s esp_timer-driven wait)
 *   'r' -> repeat both scenarios 5x and print min/avg/max for each
 *   'u' -> print current resource usage (heap/stack/flash) on demand
 */

#include <Arduino.h>
#include <stdint.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ---------- States & Events (identical to original access-control FSM) ----------
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

// ---------- esp_timer one-shot: state-duration timer (simulates e.g. GRANTED timeout) ----------
#define DURATION_WAIT_US (3000000ULL) // ~3s

static volatile uint8_t duration_elapsed = 0;
static esp_timer_handle_t duration_timer = NULL;

static void IRAM_ATTR duration_timer_callback(void* /*arg*/) {
    duration_elapsed = 1;
}

// Blocks (via polling of a flag set by the esp_timer ISR callback) until ~3s have elapsed.
static void wait_for_state_duration(void) {
    duration_elapsed = 0;

    const esp_timer_create_args_t args = {
        .callback = &duration_timer_callback,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "duration_wait",
        .skip_unhandled_events = true,
    };
    esp_timer_create(&args, &duration_timer);
    esp_timer_start_once(duration_timer, DURATION_WAIT_US);

    while (!duration_elapsed) {
        // CPU is free here - could yield()/vTaskDelay() in a real low-power design.
    }
    esp_timer_delete(duration_timer);
    duration_timer = NULL;
}

// ---------- Nested Switch FSM transition (identical logic to the original test) ----------
static uint8_t fsm_transition(uint8_t state, uint8_t event) {
    switch (state) {
        case STATE_IDLE:
            switch (event) {
                case EV_VALID:   return STATE_CHECKING;
                default:         return STATE_IDLE;
            }
        case STATE_CHECKING:
            switch (event) {
                case EV_VALID:   return STATE_GRANTED;
                case EV_INVALID: return STATE_DENIED;
                default:         return STATE_CHECKING;
            }
        case STATE_GRANTED:
            switch (event) {
                case EV_TIMEOUT: return STATE_IDLE;
                default:         return STATE_GRANTED;
            }
        case STATE_DENIED:
            switch (event) {
                case EV_TIMEOUT: return STATE_IDLE;
                default:         return STATE_DENIED;
            }
        default:
            return STATE_IDLE;
    }
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

// ---------- Single measured transition (unchanged methodology) ----------
static uint32_t measured_transition(uint8_t event) {
    noInterrupts();
    uint32_t start = cycles_now();
    uint8_t next = fsm_transition(current_state, event);
    uint32_t end = cycles_now();
    interrupts();
    current_state = next;
    return end - start;
}

// ---------- Scenario A: immediate transition, no wait ----------
static uint32_t scenario_A(void) {
    current_state = STATE_CHECKING;
    return measured_transition(EV_VALID); // CHECKING -> GRANTED, immediately
}

// ---------- Scenario B: transition after ~3s esp_timer-driven wait ----------
static uint32_t scenario_B(void) {
    current_state = STATE_GRANTED;
    wait_for_state_duration();              // ~3s, driven by esp_timer one-shot ISR
    return measured_transition(EV_TIMEOUT); // GRANTED -> IDLE, AFTER the wait
}

// ---------- Repeat both scenarios and compare statistics ----------
static void run_comparison(uint16_t N) {
    uint32_t minA = 0xFFFFFFFF, maxA = 0; uint64_t sumA = 0;
    uint32_t minB = 0xFFFFFFFF, maxB = 0; uint64_t sumB = 0;

    Serial.printf("Running %u repetitions of each scenario (esp_timer-driven ~3s wait each time)...\r\n", N);

    for (uint16_t i = 0; i < N; i++) {
        uint32_t a = scenario_A();
        if (a < minA) minA = a;
        if (a > maxA) maxA = a;
        sumA += a;

        uint32_t b = scenario_B();
        if (b < minB) minB = b;
        if (b > maxB) maxB = b;
        sumB += b;
    }

    double mhz = ESP.getCpuFreqMHz();
    Serial.println();
    Serial.println(F("--- Scenario A: CHECKING->GRANTED, NO wait ---"));
    Serial.printf("Min: %u  Avg: %u  Max: %u (cycles)\r\n", minA, (uint32_t)(sumA / N), maxA);
    Serial.printf("Min: %.3f  Avg: %.3f  Max: %.3f (us)\r\n", minA / mhz, (sumA / (double)N) / mhz, maxA / mhz);

    Serial.println(F("--- Scenario B: GRANTED->IDLE, AFTER ~3s esp_timer wait ---"));
    Serial.printf("Min: %u  Avg: %u  Max: %u (cycles)\r\n", minB, (uint32_t)(sumB / N), maxB);
    Serial.printf("Min: %.3f  Avg: %.3f  Max: %.3f (us)\r\n", minB / mhz, (sumB / (double)N) / mhz, maxB / mhz);

    Serial.println();
    Serial.println(F("If the two ranges overlap/match, transition cost is proven"));
    Serial.println(F("independent of prior state duration, even with an ISR-driven wait."));

    print_resource_usage();
}

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println();
    Serial.println(F("Time-independence proof (ESP32 esp_timer version) ready."));
    Serial.println(F("Commands: a=Scenario A (immediate)  b=Scenario B (~3s wait)  r=Repeat comparison (5x)  u=RESOURCES"));
    print_resource_usage();
}

void loop() {
    if (Serial.available()) {
        char c = Serial.read();

        if (c == 'a') {
            uint32_t cyc = scenario_A();
            double mhz = ESP.getCpuFreqMHz();
            Serial.printf("Scenario A -> State: %s | Cycles: %u (%.3f us)\r\n",
                          state_name(current_state), cyc, cyc / mhz);
        } else if (c == 'b') {
            Serial.println(F("Waiting ~3s (esp_timer one-shot)..."));
            uint32_t cyc = scenario_B();
            double mhz = ESP.getCpuFreqMHz();
            Serial.printf("Scenario B -> State: %s | Cycles: %u (%.3f us)\r\n",
                          state_name(current_state), cyc, cyc / mhz);
        } else if (c == 'r') {
            run_comparison(5);
        } else if (c == 'u') {
            print_resource_usage();
        }
    }
}

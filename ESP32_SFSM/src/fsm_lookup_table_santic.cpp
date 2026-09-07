/*
 * Rad: Santic, J. "Writing Efficient State Machines in C."
 * http://johnsantic.com/comp/state.html
 *
 * Access Control FSM - "Santic" Lookup Table Pattern (void akcijske procedure) - ESP32
 * FSM: 4 states - IDLE, CHECKING, GRANTED, DENIED
 * Events: EV_VALID, EV_INVALID, EV_TIMEOUT
 *
 * Razlika od fsm_function_pointers_esp32.cpp (Carlgren & Oskarsson, Figure 8):
 * tamo handler VRACA next_state (uint8_t (*)(void)), a current_state upisuje
 * pozivalac spolja. Ovde, tacno po Santic-u, akcijska procedura je void
 * (void (*)(void)) i SAMA upisuje current_state kao sporedni efekat - broj
 * indirektnih poziva je isti (jedan), razlikuje se samo MEHANIZAM prenosa
 * sledeceg stanja (return vrednost naspram direktnog upisa globalne
 * promenljive iznutra).
 *
 * Po Santicevom upozorenju da tabela pretrage - za razliku od switch-a - nema
 * default granu za nevalidne indekse (sto bi dovelo do neodredjenog
 * ponasanja/pada programa), ovde je zadrzana njegova eksplicitna provera
 * granica pre poziva iz tabele.
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
#define NUM_EVENTS 3

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

// ---------- Akcijske procedure (void) - svaka SAMA upisuje current_state ----------
// Tacno po Santic-u: "In an action procedure, you do whatever processing is
// required... you might have to set a new state." Procedure koje ne menjaju
// stanje su prazne (no-op), umesto da vracaju trenutno stanje nazad.

static void action_idle_valid(void)       { current_state = STATE_CHECKING; }
static void action_idle_noop(void)        { /* ostaje u IDLE, nema sta da se radi */ }

static void action_checking_valid(void)   { current_state = STATE_GRANTED; }
static void action_checking_invalid(void) { current_state = STATE_DENIED; }
static void action_checking_noop(void)    { /* ostaje u CHECKING */ }

static void action_granted_noop(void)     { /* ostaje u GRANTED */ }
static void action_granted_timeout(void)  { current_state = STATE_IDLE; }

static void action_denied_noop(void)      { /* ostaje u DENIED */ }
static void action_denied_timeout(void)   { current_state = STATE_IDLE; }

typedef void (*ActionProcedure)(void);

// 2D niz pokazivaca na void procedure - svaka celija se poziva radi sporednog
// efekta (upisa current_state), ne zbog povratne vrednosti
static const ActionProcedure state_table[NUM_STATES][NUM_EVENTS] = {
    /*                     EV_VALID                EV_INVALID                EV_TIMEOUT             */
    /* STATE_IDLE     */ { action_idle_valid,      action_idle_noop,         action_idle_noop       },
    /* STATE_CHECKING */ { action_checking_valid,  action_checking_invalid,  action_checking_noop  },
    /* STATE_GRANTED  */ { action_granted_noop,    action_granted_noop,      action_granted_timeout },
    /* STATE_DENIED   */ { action_denied_noop,     action_denied_noop,      action_denied_timeout  },
};

static const char* state_name(uint8_t s) {
    switch (s) {
        case STATE_IDLE:     return "IDLE";
        case STATE_CHECKING: return "CHECKING";
        case STATE_GRANTED:  return "GRANTED";
        case STATE_DENIED:   return "DENIED";
        default:             return "UNKNOWN";
    }
}

// Za razliku od ostalih implementacija u ovom radu, ovde nema fsm_transition()
// koja vraca next_state - tranzicija je sporedni efekat poziva iz tabele.
// Eksplicitna provera granica pre poziva odrazava Santicevo upozorenje da
// tabela pretrage, za razliku od switch-a, nema default granu.
static uint32_t measured_transition(uint8_t event) {
    noInterrupts();
    uint32_t start = cycles_now();
    if (((event < NUM_EVENTS)) && ((current_state < NUM_STATES))) {
        state_table[current_state][event](); // poziv radi sporednog efekta na current_state
    }
    uint32_t end = cycles_now();
    interrupts();
    return end - start;
}

static void run_benchmark(void) {
    const uint16_t N = 1000;
    uint32_t min_c = 0xFFFFFFFF, max_c = 0;
    uint64_t sum_c = 0;
    uint8_t ev_cycle[3] = { EV_VALID, EV_VALID, EV_TIMEOUT }; // IDLE->CHECKING->GRANTED->IDLE

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
    Serial.println(F("Access FSM - Santic Lookup Table (void action procedures) pattern ready (ESP32)."));
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
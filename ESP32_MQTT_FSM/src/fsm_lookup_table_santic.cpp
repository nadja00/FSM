/*
 *
 * Razlika od fsm_function_pointers_esp32.cpp (Carlgren & Oskarsson, Figure 8):
 * tamo handler VRACA next_state, a current_state upisuje pozivalac spolja.
 * Ovde, tacno po Santicu, akcijska procedura je void i SAMA upisuje
 * current_state kao sporedni efekat - broj indirektnih poziva je isti
 * (jedan po tranziciji), razlikuje se samo MEHANIZAM prenosa sledeceg stanja.
 *
 * Po Santicevom upozorenju da tabela pretrage - za razliku od switch-a -
 * nema default granu za nevalidne indekse, ovde je zadrzana njegova
 * eksplicitna provera granica pre poziva iz tabele.
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

// ---------- Akcijske procedure (void) - svaka SAMA upisuje current_state ----------
// Generisano i verifikovano protiv .dot specifikacije (svih 144 kombinacija).
static void action_s0_connectc2(void) { current_state = S3; }
static void action_s0_connectc1withwill(void) { current_state = S1; }
static void action_s0_publishqos0c2(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s0_publishqos1c1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s0_subscribec1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s0_unsubscribec1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s0_subscribec2(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s0_unsubscribec2(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s0_disconnecttcpc1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s1_connectc2(void) { current_state = S2; }
static void action_s1_connectc1withwill(void) { current_state = S4; }
static void action_s1_publishqos0c2(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s1_publishqos1c1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s1_subscribec1(void) { current_state = S14; }
static void action_s1_unsubscribec1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s1_subscribec2(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s1_unsubscribec2(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s1_disconnecttcpc1(void) { current_state = S0; }
static void action_s2_connectc2(void) { current_state = S8; }
static void action_s2_connectc1withwill(void) { current_state = S5; }
static void action_s2_publishqos0c2(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s2_publishqos1c1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s2_subscribec1(void) { current_state = S11; }
static void action_s2_unsubscribec1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s2_subscribec2(void) { current_state = S6; }
static void action_s2_unsubscribec2(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s2_disconnecttcpc1(void) { current_state = S3; }
static void action_s3_connectc2(void) { current_state = S9; }
static void action_s3_connectc1withwill(void) { current_state = S2; }
static void action_s3_publishqos0c2(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s3_publishqos1c1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s3_subscribec1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s3_unsubscribec1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s3_subscribec2(void) { current_state = S13; }
static void action_s3_unsubscribec2(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s3_disconnecttcpc1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s4_connectc2(void) { current_state = S5; }
static void action_s4_connectc1withwill(void) { current_state = S1; }
static void action_s4_publishqos0c2(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s4_publishqos1c1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s4_subscribec1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s4_unsubscribec1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s4_subscribec2(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s4_unsubscribec2(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s4_disconnecttcpc1(void) { current_state = S0; }
static void action_s5_connectc2(void) { current_state = S12; }
static void action_s5_connectc1withwill(void) { current_state = S2; }
static void action_s5_publishqos0c2(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s5_publishqos1c1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s5_subscribec1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s5_unsubscribec1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s5_subscribec2(void) { current_state = S7; }
static void action_s5_unsubscribec2(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s5_disconnecttcpc1(void) { current_state = S3; }
static void action_s6_connectc2(void) { current_state = S8; }
static void action_s6_connectc1withwill(void) { current_state = S7; }
static void action_s6_publishqos0c2(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s6_publishqos1c1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s6_subscribec1(void) { current_state = S10; }
static void action_s6_unsubscribec1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s6_subscribec2(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s6_unsubscribec2(void) { current_state = S2; }
static void action_s6_disconnecttcpc1(void) { current_state = S13; }
static void action_s7_connectc2(void) { current_state = S12; }
static void action_s7_connectc1withwill(void) { current_state = S6; }
static void action_s7_publishqos0c2(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s7_publishqos1c1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s7_subscribec1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s7_unsubscribec1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s7_subscribec2(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s7_unsubscribec2(void) { current_state = S5; }
static void action_s7_disconnecttcpc1(void) { current_state = S13; }
static void action_s8_connectc2(void) { current_state = S2; }
static void action_s8_connectc1withwill(void) { current_state = S12; }
static void action_s8_publishqos0c2(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s8_publishqos1c1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s8_subscribec1(void) { current_state = S15; }
static void action_s8_unsubscribec1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s8_subscribec2(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s8_unsubscribec2(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s8_disconnecttcpc1(void) { current_state = S9; }
static void action_s9_connectc2(void) { current_state = S3; }
static void action_s9_connectc1withwill(void) { current_state = S8; }
static void action_s9_publishqos0c2(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s9_publishqos1c1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s9_subscribec1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s9_unsubscribec1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s9_subscribec2(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s9_unsubscribec2(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s9_disconnecttcpc1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s10_connectc2(void) { current_state = S15; }
static void action_s10_connectc1withwill(void) { current_state = S7; }
static void action_s10_publishqos0c2(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s10_publishqos1c1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s10_subscribec1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s10_unsubscribec1(void) { current_state = S6; }
static void action_s10_subscribec2(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s10_unsubscribec2(void) { current_state = S11; }
static void action_s10_disconnecttcpc1(void) { current_state = S13; }
static void action_s11_connectc2(void) { current_state = S15; }
static void action_s11_connectc1withwill(void) { current_state = S5; }
static void action_s11_publishqos0c2(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s11_publishqos1c1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s11_subscribec1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s11_unsubscribec1(void) { current_state = S2; }
static void action_s11_subscribec2(void) { current_state = S10; }
static void action_s11_unsubscribec2(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s11_disconnecttcpc1(void) { current_state = S3; }
static void action_s12_connectc2(void) { current_state = S5; }
static void action_s12_connectc1withwill(void) { current_state = S8; }
static void action_s12_publishqos0c2(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s12_publishqos1c1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s12_subscribec1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s12_unsubscribec1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s12_subscribec2(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s12_unsubscribec2(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s12_disconnecttcpc1(void) { current_state = S9; }
static void action_s13_connectc2(void) { current_state = S9; }
static void action_s13_connectc1withwill(void) { current_state = S6; }
static void action_s13_publishqos0c2(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s13_publishqos1c1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s13_subscribec1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s13_unsubscribec1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s13_subscribec2(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s13_unsubscribec2(void) { current_state = S3; }
static void action_s13_disconnecttcpc1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s14_connectc2(void) { current_state = S11; }
static void action_s14_connectc1withwill(void) { current_state = S4; }
static void action_s14_publishqos0c2(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s14_publishqos1c1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s14_subscribec1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s14_unsubscribec1(void) { current_state = S1; }
static void action_s14_subscribec2(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s14_unsubscribec2(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s14_disconnecttcpc1(void) { current_state = S0; }
static void action_s15_connectc2(void) { current_state = S11; }
static void action_s15_connectc1withwill(void) { current_state = S12; }
static void action_s15_publishqos0c2(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s15_publishqos1c1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s15_subscribec1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s15_unsubscribec1(void) { current_state = S8; }
static void action_s15_subscribec2(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s15_unsubscribec2(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s15_disconnecttcpc1(void) { current_state = S9; }

typedef void (*ActionProcedure)(void);

// 2D niz pokazivaca na void procedure - svaka celija se poziva radi sporednog
// efekta (upisa current_state), ne zbog povratne vrednosti.
static const ActionProcedure state_table[NUM_STATES][NUM_EVENTS] = {
    { action_s0_connectc2, action_s0_connectc1withwill, action_s0_publishqos0c2, action_s0_publishqos1c1, action_s0_subscribec1, action_s0_unsubscribec1, action_s0_subscribec2, action_s0_unsubscribec2, action_s0_disconnecttcpc1 }, // S0
    { action_s1_connectc2, action_s1_connectc1withwill, action_s1_publishqos0c2, action_s1_publishqos1c1, action_s1_subscribec1, action_s1_unsubscribec1, action_s1_subscribec2, action_s1_unsubscribec2, action_s1_disconnecttcpc1 }, // S1
    { action_s2_connectc2, action_s2_connectc1withwill, action_s2_publishqos0c2, action_s2_publishqos1c1, action_s2_subscribec1, action_s2_unsubscribec1, action_s2_subscribec2, action_s2_unsubscribec2, action_s2_disconnecttcpc1 }, // S2
    { action_s3_connectc2, action_s3_connectc1withwill, action_s3_publishqos0c2, action_s3_publishqos1c1, action_s3_subscribec1, action_s3_unsubscribec1, action_s3_subscribec2, action_s3_unsubscribec2, action_s3_disconnecttcpc1 }, // S3
    { action_s4_connectc2, action_s4_connectc1withwill, action_s4_publishqos0c2, action_s4_publishqos1c1, action_s4_subscribec1, action_s4_unsubscribec1, action_s4_subscribec2, action_s4_unsubscribec2, action_s4_disconnecttcpc1 }, // S4
    { action_s5_connectc2, action_s5_connectc1withwill, action_s5_publishqos0c2, action_s5_publishqos1c1, action_s5_subscribec1, action_s5_unsubscribec1, action_s5_subscribec2, action_s5_unsubscribec2, action_s5_disconnecttcpc1 }, // S5
    { action_s6_connectc2, action_s6_connectc1withwill, action_s6_publishqos0c2, action_s6_publishqos1c1, action_s6_subscribec1, action_s6_unsubscribec1, action_s6_subscribec2, action_s6_unsubscribec2, action_s6_disconnecttcpc1 }, // S6
    { action_s7_connectc2, action_s7_connectc1withwill, action_s7_publishqos0c2, action_s7_publishqos1c1, action_s7_subscribec1, action_s7_unsubscribec1, action_s7_subscribec2, action_s7_unsubscribec2, action_s7_disconnecttcpc1 }, // S7
    { action_s8_connectc2, action_s8_connectc1withwill, action_s8_publishqos0c2, action_s8_publishqos1c1, action_s8_subscribec1, action_s8_unsubscribec1, action_s8_subscribec2, action_s8_unsubscribec2, action_s8_disconnecttcpc1 }, // S8
    { action_s9_connectc2, action_s9_connectc1withwill, action_s9_publishqos0c2, action_s9_publishqos1c1, action_s9_subscribec1, action_s9_unsubscribec1, action_s9_subscribec2, action_s9_unsubscribec2, action_s9_disconnecttcpc1 }, // S9
    { action_s10_connectc2, action_s10_connectc1withwill, action_s10_publishqos0c2, action_s10_publishqos1c1, action_s10_subscribec1, action_s10_unsubscribec1, action_s10_subscribec2, action_s10_unsubscribec2, action_s10_disconnecttcpc1 }, // S10
    { action_s11_connectc2, action_s11_connectc1withwill, action_s11_publishqos0c2, action_s11_publishqos1c1, action_s11_subscribec1, action_s11_unsubscribec1, action_s11_subscribec2, action_s11_unsubscribec2, action_s11_disconnecttcpc1 }, // S11
    { action_s12_connectc2, action_s12_connectc1withwill, action_s12_publishqos0c2, action_s12_publishqos1c1, action_s12_subscribec1, action_s12_unsubscribec1, action_s12_subscribec2, action_s12_unsubscribec2, action_s12_disconnecttcpc1 }, // S12
    { action_s13_connectc2, action_s13_connectc1withwill, action_s13_publishqos0c2, action_s13_publishqos1c1, action_s13_subscribec1, action_s13_unsubscribec1, action_s13_subscribec2, action_s13_unsubscribec2, action_s13_disconnecttcpc1 }, // S13
    { action_s14_connectc2, action_s14_connectc1withwill, action_s14_publishqos0c2, action_s14_publishqos1c1, action_s14_subscribec1, action_s14_unsubscribec1, action_s14_subscribec2, action_s14_unsubscribec2, action_s14_disconnecttcpc1 }, // S14
    { action_s15_connectc2, action_s15_connectc1withwill, action_s15_publishqos0c2, action_s15_publishqos1c1, action_s15_subscribec1, action_s15_unsubscribec1, action_s15_subscribec2, action_s15_unsubscribec2, action_s15_disconnecttcpc1 }, // S15
};

static const char *state_name(uint8_t s)
{
    switch (s) {
        case S0:  return "S0";  case S1:  return "S1";  case S2:  return "S2";  case S3:  return "S3";
        case S4:  return "S4";  case S5:  return "S5";  case S6:  return "S6";  case S7:  return "S7";
        case S8:  return "S8";  case S9:  return "S9";  case S10: return "S10"; case S11: return "S11";
        case S12: return "S12"; case S13: return "S13"; case S14: return "S14"; case S15: return "S15";
        default:  return "UNKNOWN";
    }
}

// Za razliku od ostalih implementacija, ovde nema fsm_transition() koja vraca
// next_state - tranzicija je sporedni efekat poziva iz tabele. Eksplicitna
// provera granica pre poziva odrazava Santicevo upozorenje da tabela
// pretrage, za razliku od switch-a, nema default granu.
static uint32_t measured_transition(uint8_t event) {
    noInterrupts();
    uint32_t start = cycles_now();
    if ((event < NUM_EVENTS) && (current_state < NUM_STATES)) {
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
    Serial.println(F("MQTT FSM - Santic Lookup Table (void action procedures) pattern ready (ESP32)."));
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

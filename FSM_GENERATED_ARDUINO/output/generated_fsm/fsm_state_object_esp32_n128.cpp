/*
 * AUTO-GENERISANO fajlom fsm_generator.py
 * Pattern: state_object | Platforma: esp32 | N_STATES=128 | N_EVENTS=3
 * NE MENJATI RUCNO - regenerisati skriptom radi konzistentnosti sweep-a.
 */
#include <Arduino.h>
#include <stdint.h>

static void uart_init(void) { Serial.begin(9600); while (!Serial) {} }
static void uart_putc(char c) { Serial.write(c); }
static void uart_puts(const char *s) { Serial.print(s); }
static void uart_put_uint(uint32_t v) { Serial.print(v); }
static uint8_t uart_available(void) { return Serial.available() > 0; }
static char uart_getc(void) { while (!Serial.available()) {} return Serial.read(); }

static inline uint32_t cycles_start(void) { return ESP.getCycleCount(); }
static inline uint32_t cycles_stop(uint32_t start) { return ESP.getCycleCount() - start; }
typedef uint32_t cycle_t;


#define NUM_STATES 128
#define NUM_EVENTS 3
enum Event {EV_0, EV_1, EV_2};

static volatile uint8_t current_state = 0;

static uint8_t state_0_handle(uint8_t event) {
    if (event == EV_0) return 1;
    return 0;
}
static uint8_t state_1_handle(uint8_t event) {
    if (event == EV_0) return 2;
    return 1;
}
static uint8_t state_2_handle(uint8_t event) {
    if (event == EV_0) return 3;
    return 2;
}
static uint8_t state_3_handle(uint8_t event) {
    if (event == EV_0) return 4;
    return 3;
}
static uint8_t state_4_handle(uint8_t event) {
    if (event == EV_0) return 5;
    return 4;
}
static uint8_t state_5_handle(uint8_t event) {
    if (event == EV_0) return 6;
    return 5;
}
static uint8_t state_6_handle(uint8_t event) {
    if (event == EV_0) return 7;
    return 6;
}
static uint8_t state_7_handle(uint8_t event) {
    if (event == EV_0) return 8;
    return 7;
}
static uint8_t state_8_handle(uint8_t event) {
    if (event == EV_0) return 9;
    return 8;
}
static uint8_t state_9_handle(uint8_t event) {
    if (event == EV_0) return 10;
    return 9;
}
static uint8_t state_10_handle(uint8_t event) {
    if (event == EV_0) return 11;
    return 10;
}
static uint8_t state_11_handle(uint8_t event) {
    if (event == EV_0) return 12;
    return 11;
}
static uint8_t state_12_handle(uint8_t event) {
    if (event == EV_0) return 13;
    return 12;
}
static uint8_t state_13_handle(uint8_t event) {
    if (event == EV_0) return 14;
    return 13;
}
static uint8_t state_14_handle(uint8_t event) {
    if (event == EV_0) return 15;
    return 14;
}
static uint8_t state_15_handle(uint8_t event) {
    if (event == EV_0) return 16;
    return 15;
}
static uint8_t state_16_handle(uint8_t event) {
    if (event == EV_0) return 17;
    return 16;
}
static uint8_t state_17_handle(uint8_t event) {
    if (event == EV_0) return 18;
    return 17;
}
static uint8_t state_18_handle(uint8_t event) {
    if (event == EV_0) return 19;
    return 18;
}
static uint8_t state_19_handle(uint8_t event) {
    if (event == EV_0) return 20;
    return 19;
}
static uint8_t state_20_handle(uint8_t event) {
    if (event == EV_0) return 21;
    return 20;
}
static uint8_t state_21_handle(uint8_t event) {
    if (event == EV_0) return 22;
    return 21;
}
static uint8_t state_22_handle(uint8_t event) {
    if (event == EV_0) return 23;
    return 22;
}
static uint8_t state_23_handle(uint8_t event) {
    if (event == EV_0) return 24;
    return 23;
}
static uint8_t state_24_handle(uint8_t event) {
    if (event == EV_0) return 25;
    return 24;
}
static uint8_t state_25_handle(uint8_t event) {
    if (event == EV_0) return 26;
    return 25;
}
static uint8_t state_26_handle(uint8_t event) {
    if (event == EV_0) return 27;
    return 26;
}
static uint8_t state_27_handle(uint8_t event) {
    if (event == EV_0) return 28;
    return 27;
}
static uint8_t state_28_handle(uint8_t event) {
    if (event == EV_0) return 29;
    return 28;
}
static uint8_t state_29_handle(uint8_t event) {
    if (event == EV_0) return 30;
    return 29;
}
static uint8_t state_30_handle(uint8_t event) {
    if (event == EV_0) return 31;
    return 30;
}
static uint8_t state_31_handle(uint8_t event) {
    if (event == EV_0) return 32;
    return 31;
}
static uint8_t state_32_handle(uint8_t event) {
    if (event == EV_0) return 33;
    return 32;
}
static uint8_t state_33_handle(uint8_t event) {
    if (event == EV_0) return 34;
    return 33;
}
static uint8_t state_34_handle(uint8_t event) {
    if (event == EV_0) return 35;
    return 34;
}
static uint8_t state_35_handle(uint8_t event) {
    if (event == EV_0) return 36;
    return 35;
}
static uint8_t state_36_handle(uint8_t event) {
    if (event == EV_0) return 37;
    return 36;
}
static uint8_t state_37_handle(uint8_t event) {
    if (event == EV_0) return 38;
    return 37;
}
static uint8_t state_38_handle(uint8_t event) {
    if (event == EV_0) return 39;
    return 38;
}
static uint8_t state_39_handle(uint8_t event) {
    if (event == EV_0) return 40;
    return 39;
}
static uint8_t state_40_handle(uint8_t event) {
    if (event == EV_0) return 41;
    return 40;
}
static uint8_t state_41_handle(uint8_t event) {
    if (event == EV_0) return 42;
    return 41;
}
static uint8_t state_42_handle(uint8_t event) {
    if (event == EV_0) return 43;
    return 42;
}
static uint8_t state_43_handle(uint8_t event) {
    if (event == EV_0) return 44;
    return 43;
}
static uint8_t state_44_handle(uint8_t event) {
    if (event == EV_0) return 45;
    return 44;
}
static uint8_t state_45_handle(uint8_t event) {
    if (event == EV_0) return 46;
    return 45;
}
static uint8_t state_46_handle(uint8_t event) {
    if (event == EV_0) return 47;
    return 46;
}
static uint8_t state_47_handle(uint8_t event) {
    if (event == EV_0) return 48;
    return 47;
}
static uint8_t state_48_handle(uint8_t event) {
    if (event == EV_0) return 49;
    return 48;
}
static uint8_t state_49_handle(uint8_t event) {
    if (event == EV_0) return 50;
    return 49;
}
static uint8_t state_50_handle(uint8_t event) {
    if (event == EV_0) return 51;
    return 50;
}
static uint8_t state_51_handle(uint8_t event) {
    if (event == EV_0) return 52;
    return 51;
}
static uint8_t state_52_handle(uint8_t event) {
    if (event == EV_0) return 53;
    return 52;
}
static uint8_t state_53_handle(uint8_t event) {
    if (event == EV_0) return 54;
    return 53;
}
static uint8_t state_54_handle(uint8_t event) {
    if (event == EV_0) return 55;
    return 54;
}
static uint8_t state_55_handle(uint8_t event) {
    if (event == EV_0) return 56;
    return 55;
}
static uint8_t state_56_handle(uint8_t event) {
    if (event == EV_0) return 57;
    return 56;
}
static uint8_t state_57_handle(uint8_t event) {
    if (event == EV_0) return 58;
    return 57;
}
static uint8_t state_58_handle(uint8_t event) {
    if (event == EV_0) return 59;
    return 58;
}
static uint8_t state_59_handle(uint8_t event) {
    if (event == EV_0) return 60;
    return 59;
}
static uint8_t state_60_handle(uint8_t event) {
    if (event == EV_0) return 61;
    return 60;
}
static uint8_t state_61_handle(uint8_t event) {
    if (event == EV_0) return 62;
    return 61;
}
static uint8_t state_62_handle(uint8_t event) {
    if (event == EV_0) return 63;
    return 62;
}
static uint8_t state_63_handle(uint8_t event) {
    if (event == EV_0) return 64;
    return 63;
}
static uint8_t state_64_handle(uint8_t event) {
    if (event == EV_0) return 65;
    return 64;
}
static uint8_t state_65_handle(uint8_t event) {
    if (event == EV_0) return 66;
    return 65;
}
static uint8_t state_66_handle(uint8_t event) {
    if (event == EV_0) return 67;
    return 66;
}
static uint8_t state_67_handle(uint8_t event) {
    if (event == EV_0) return 68;
    return 67;
}
static uint8_t state_68_handle(uint8_t event) {
    if (event == EV_0) return 69;
    return 68;
}
static uint8_t state_69_handle(uint8_t event) {
    if (event == EV_0) return 70;
    return 69;
}
static uint8_t state_70_handle(uint8_t event) {
    if (event == EV_0) return 71;
    return 70;
}
static uint8_t state_71_handle(uint8_t event) {
    if (event == EV_0) return 72;
    return 71;
}
static uint8_t state_72_handle(uint8_t event) {
    if (event == EV_0) return 73;
    return 72;
}
static uint8_t state_73_handle(uint8_t event) {
    if (event == EV_0) return 74;
    return 73;
}
static uint8_t state_74_handle(uint8_t event) {
    if (event == EV_0) return 75;
    return 74;
}
static uint8_t state_75_handle(uint8_t event) {
    if (event == EV_0) return 76;
    return 75;
}
static uint8_t state_76_handle(uint8_t event) {
    if (event == EV_0) return 77;
    return 76;
}
static uint8_t state_77_handle(uint8_t event) {
    if (event == EV_0) return 78;
    return 77;
}
static uint8_t state_78_handle(uint8_t event) {
    if (event == EV_0) return 79;
    return 78;
}
static uint8_t state_79_handle(uint8_t event) {
    if (event == EV_0) return 80;
    return 79;
}
static uint8_t state_80_handle(uint8_t event) {
    if (event == EV_0) return 81;
    return 80;
}
static uint8_t state_81_handle(uint8_t event) {
    if (event == EV_0) return 82;
    return 81;
}
static uint8_t state_82_handle(uint8_t event) {
    if (event == EV_0) return 83;
    return 82;
}
static uint8_t state_83_handle(uint8_t event) {
    if (event == EV_0) return 84;
    return 83;
}
static uint8_t state_84_handle(uint8_t event) {
    if (event == EV_0) return 85;
    return 84;
}
static uint8_t state_85_handle(uint8_t event) {
    if (event == EV_0) return 86;
    return 85;
}
static uint8_t state_86_handle(uint8_t event) {
    if (event == EV_0) return 87;
    return 86;
}
static uint8_t state_87_handle(uint8_t event) {
    if (event == EV_0) return 88;
    return 87;
}
static uint8_t state_88_handle(uint8_t event) {
    if (event == EV_0) return 89;
    return 88;
}
static uint8_t state_89_handle(uint8_t event) {
    if (event == EV_0) return 90;
    return 89;
}
static uint8_t state_90_handle(uint8_t event) {
    if (event == EV_0) return 91;
    return 90;
}
static uint8_t state_91_handle(uint8_t event) {
    if (event == EV_0) return 92;
    return 91;
}
static uint8_t state_92_handle(uint8_t event) {
    if (event == EV_0) return 93;
    return 92;
}
static uint8_t state_93_handle(uint8_t event) {
    if (event == EV_0) return 94;
    return 93;
}
static uint8_t state_94_handle(uint8_t event) {
    if (event == EV_0) return 95;
    return 94;
}
static uint8_t state_95_handle(uint8_t event) {
    if (event == EV_0) return 96;
    return 95;
}
static uint8_t state_96_handle(uint8_t event) {
    if (event == EV_0) return 97;
    return 96;
}
static uint8_t state_97_handle(uint8_t event) {
    if (event == EV_0) return 98;
    return 97;
}
static uint8_t state_98_handle(uint8_t event) {
    if (event == EV_0) return 99;
    return 98;
}
static uint8_t state_99_handle(uint8_t event) {
    if (event == EV_0) return 100;
    return 99;
}
static uint8_t state_100_handle(uint8_t event) {
    if (event == EV_0) return 101;
    return 100;
}
static uint8_t state_101_handle(uint8_t event) {
    if (event == EV_0) return 102;
    return 101;
}
static uint8_t state_102_handle(uint8_t event) {
    if (event == EV_0) return 103;
    return 102;
}
static uint8_t state_103_handle(uint8_t event) {
    if (event == EV_0) return 104;
    return 103;
}
static uint8_t state_104_handle(uint8_t event) {
    if (event == EV_0) return 105;
    return 104;
}
static uint8_t state_105_handle(uint8_t event) {
    if (event == EV_0) return 106;
    return 105;
}
static uint8_t state_106_handle(uint8_t event) {
    if (event == EV_0) return 107;
    return 106;
}
static uint8_t state_107_handle(uint8_t event) {
    if (event == EV_0) return 108;
    return 107;
}
static uint8_t state_108_handle(uint8_t event) {
    if (event == EV_0) return 109;
    return 108;
}
static uint8_t state_109_handle(uint8_t event) {
    if (event == EV_0) return 110;
    return 109;
}
static uint8_t state_110_handle(uint8_t event) {
    if (event == EV_0) return 111;
    return 110;
}
static uint8_t state_111_handle(uint8_t event) {
    if (event == EV_0) return 112;
    return 111;
}
static uint8_t state_112_handle(uint8_t event) {
    if (event == EV_0) return 113;
    return 112;
}
static uint8_t state_113_handle(uint8_t event) {
    if (event == EV_0) return 114;
    return 113;
}
static uint8_t state_114_handle(uint8_t event) {
    if (event == EV_0) return 115;
    return 114;
}
static uint8_t state_115_handle(uint8_t event) {
    if (event == EV_0) return 116;
    return 115;
}
static uint8_t state_116_handle(uint8_t event) {
    if (event == EV_0) return 117;
    return 116;
}
static uint8_t state_117_handle(uint8_t event) {
    if (event == EV_0) return 118;
    return 117;
}
static uint8_t state_118_handle(uint8_t event) {
    if (event == EV_0) return 119;
    return 118;
}
static uint8_t state_119_handle(uint8_t event) {
    if (event == EV_0) return 120;
    return 119;
}
static uint8_t state_120_handle(uint8_t event) {
    if (event == EV_0) return 121;
    return 120;
}
static uint8_t state_121_handle(uint8_t event) {
    if (event == EV_0) return 122;
    return 121;
}
static uint8_t state_122_handle(uint8_t event) {
    if (event == EV_0) return 123;
    return 122;
}
static uint8_t state_123_handle(uint8_t event) {
    if (event == EV_0) return 124;
    return 123;
}
static uint8_t state_124_handle(uint8_t event) {
    if (event == EV_0) return 125;
    return 124;
}
static uint8_t state_125_handle(uint8_t event) {
    if (event == EV_0) return 126;
    return 125;
}
static uint8_t state_126_handle(uint8_t event) {
    if (event == EV_0) return 127;
    return 126;
}
static uint8_t state_127_handle(uint8_t event) {
    if (event == EV_0) return 0;
    return 127;
}

typedef uint8_t (*StateHandler)(uint8_t event);
static StateHandler const state_handlers[NUM_STATES] = {
    state_0_handle,
    state_1_handle,
    state_2_handle,
    state_3_handle,
    state_4_handle,
    state_5_handle,
    state_6_handle,
    state_7_handle,
    state_8_handle,
    state_9_handle,
    state_10_handle,
    state_11_handle,
    state_12_handle,
    state_13_handle,
    state_14_handle,
    state_15_handle,
    state_16_handle,
    state_17_handle,
    state_18_handle,
    state_19_handle,
    state_20_handle,
    state_21_handle,
    state_22_handle,
    state_23_handle,
    state_24_handle,
    state_25_handle,
    state_26_handle,
    state_27_handle,
    state_28_handle,
    state_29_handle,
    state_30_handle,
    state_31_handle,
    state_32_handle,
    state_33_handle,
    state_34_handle,
    state_35_handle,
    state_36_handle,
    state_37_handle,
    state_38_handle,
    state_39_handle,
    state_40_handle,
    state_41_handle,
    state_42_handle,
    state_43_handle,
    state_44_handle,
    state_45_handle,
    state_46_handle,
    state_47_handle,
    state_48_handle,
    state_49_handle,
    state_50_handle,
    state_51_handle,
    state_52_handle,
    state_53_handle,
    state_54_handle,
    state_55_handle,
    state_56_handle,
    state_57_handle,
    state_58_handle,
    state_59_handle,
    state_60_handle,
    state_61_handle,
    state_62_handle,
    state_63_handle,
    state_64_handle,
    state_65_handle,
    state_66_handle,
    state_67_handle,
    state_68_handle,
    state_69_handle,
    state_70_handle,
    state_71_handle,
    state_72_handle,
    state_73_handle,
    state_74_handle,
    state_75_handle,
    state_76_handle,
    state_77_handle,
    state_78_handle,
    state_79_handle,
    state_80_handle,
    state_81_handle,
    state_82_handle,
    state_83_handle,
    state_84_handle,
    state_85_handle,
    state_86_handle,
    state_87_handle,
    state_88_handle,
    state_89_handle,
    state_90_handle,
    state_91_handle,
    state_92_handle,
    state_93_handle,
    state_94_handle,
    state_95_handle,
    state_96_handle,
    state_97_handle,
    state_98_handle,
    state_99_handle,
    state_100_handle,
    state_101_handle,
    state_102_handle,
    state_103_handle,
    state_104_handle,
    state_105_handle,
    state_106_handle,
    state_107_handle,
    state_108_handle,
    state_109_handle,
    state_110_handle,
    state_111_handle,
    state_112_handle,
    state_113_handle,
    state_114_handle,
    state_115_handle,
    state_116_handle,
    state_117_handle,
    state_118_handle,
    state_119_handle,
    state_120_handle,
    state_121_handle,
    state_122_handle,
    state_123_handle,
    state_124_handle,
    state_125_handle,
    state_126_handle,
    state_127_handle
};

static inline uint8_t fsm_transition(uint8_t state, uint8_t event) {
    return state_handlers[state](event);
}

static cycle_t measured_transition(uint8_t event) {
    noInterrupts();
    uint32_t t0 = cycles_start();
    uint8_t next = fsm_transition(current_state, event);
    cycle_t cycles = cycles_stop(t0);
    interrupts();
    current_state = next;
    return cycles;
}

static void run_benchmark(void) {
    const uint16_t ITER = 1000;
    cycle_t min_c = 0xFFFFFFFF, max_c = 0;
    uint32_t sum_c = 0;

    uart_puts("Running benchmark (");
    uart_put_uint(ITER);
    uart_puts(" transitions)...\r\n");

    for (uint16_t i = 0; i < ITER; i++) {
        cycle_t c = measured_transition(EV_0);
        if (c < min_c) min_c = c;
        if (c > max_c) max_c = c;
        sum_c += c;
    }

    uart_puts("N_STATES: "); uart_put_uint(NUM_STATES); uart_puts("\r\n");
    uart_puts("N_EVENTS: "); uart_put_uint(NUM_EVENTS); uart_puts("\r\n");
    uart_puts("Min cycles: "); uart_put_uint((uint32_t)min_c); uart_puts("\r\n");
    uart_puts("Max cycles: "); uart_put_uint((uint32_t)max_c); uart_puts("\r\n");
    uart_puts("Avg cycles: "); uart_put_uint((uint32_t)(sum_c / ITER)); uart_puts("\r\n");
}

void setup() {
    uart_init();
    uart_puts("\r\nParametrized FSM ready.\r\n");
}

void loop() {
    if (uart_available()) {
        char c = uart_getc();
        if (c == 'b') { run_benchmark(); }
    }
}

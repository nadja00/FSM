#!/usr/bin/env python3
"""
fsm_generator.py
================
Generator parametrizovanih FSM .cpp fajlova (Arduino/PlatformIO, AVR i ESP32)
za skaliranu analizu broja ciklusa i zauzeca memorije u funkciji broja stanja N
(i, za HSM, dubine hijerarhije D).

Zadrzava IDENTICNU mernu infrastrukturu kao rucno pisani fajlovi iz projekta:
- UART protokol (9600 baud, komande '1'/'0'/'t'/'b', bare-metal register pristup)
- cli()/sei() oko mernog prozora
- Timer1 (TCNT1, no prescaler) na AVR / ccount registar na ESP32
- run_benchmark() sa N_ITERACIJA=1000 ponovljenih tranzicija, min/avg/max izlaz

Topologija generisanog FSM-a (za date parametre states=N, events=M):
- N stanja u prstenu: S0 -> S1 -> ... -> S(N-1) -> S0, tranzicija na EV_0 (event 0)
- Svako stanje ima jos (M-1) "no-op" dogadjaja (EV_1..EV_(M-1)) koji vracaju
  isto stanje (self-loop) - ovo drzi array_of_structs "sparse" (tacno N validnih
  grana ukupno) a indexed_table/function_pointers "dense" (N*M popunjenih celija),
  tacno kao teorijska tabela slozenosti iz ESP32_sekcija.docx.
- Benchmark uvek salje EV_0 (napredujuca tranzicija), cime se FSM ciklicno vrti
  kroz sva N stanja tokom 1000 iteracija - i to je slucaj koji se meri.

Za hsm_nested: topologija je LANAC roditelja dubine D (chain_0 -> parent ->
chain_1 -> parent -> ... -> chain_(D-1) je top), a benchmark uvek startuje iz
najdubljeg lista i salje event koji se obradjuje SAMO na root-u (top state),
cime se izmeri trosak bubbling-a kroz D nivoa (najgori slucaj, u skladu sa
metodologijom Test 4a/4b iz Arduino_sekcija.docx).

Upotreba:
    python fsm_generator.py --pattern indexed_table --n 10 --events 3 --platform avr -o out.cpp
    python fsm_generator.py --pattern hsm_nested --depth 5 --platform esp32 -o out.cpp
    python fsm_generator.py --sweep   # generise SVE fajlove za sweep definisan u SWEEP_CONFIG

Podrzani pattern-i: nested_switch, array_of_structs, indexed_table, state_object, hsm_nested
Podrzane platforme: avr (Arduino Uno), esp32
"""

import argparse
import os

# ==============================================================================
# ZAJEDNICKI DELOVI KODA (identicni za sve pattern-e, razlikuju se po platformi)
# ==============================================================================

AVR_HEADER = """#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdlib.h>

#define BAUD 9600
#define UBRR_VAL ((F_CPU / (16UL * BAUD)) - 1)

static void uart_init(void) {
    UBRR0H = (uint8_t)(UBRR_VAL >> 8);
    UBRR0L = (uint8_t)UBRR_VAL;
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}
static void uart_putc(char c) { while (!(UCSR0A & (1 << UDRE0))); UDR0 = c; }
static void uart_puts(const char *s) { while (*s) uart_putc(*s++); }
static void uart_put_uint(uint16_t v) {
    char buf[6]; uint8_t i = 0;
    if (v == 0) { uart_putc('0'); return; }
    while (v > 0) { buf[i++] = '0' + (v % 10); v /= 10; }
    while (i > 0) uart_putc(buf[--i]);
}
static uint8_t uart_available(void) { return (UCSR0A & (1 << RXC0)) != 0; }
static char uart_getc(void) { while (!(UCSR0A & (1 << RXC0))); return UDR0; }

static inline void cycles_start(void) { TCCR1B = 0; TCNT1 = 0; TCCR1B = (1 << CS10); }
static inline uint16_t cycles_stop(void) { TCCR1B = 0; return TCNT1; }
typedef uint16_t cycle_t;
"""

ESP32_HEADER = """#include <Arduino.h>
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
"""

def gen_benchmark_and_main(platform, extra_report=""):
    """measured_transition/run_benchmark/setup/loop - identicni protokol na obe platforme,
    samo se razlikuje potpis cycles_start/stop (AVR: start/stop bez argumenta i sa cli/sei;
    ESP32: start vraca pocetnu vrednost, stop racuna razliku, koristi noInterrupts/interrupts)."""
    if platform == "avr":
        measured = """
static cycle_t measured_transition(uint8_t event) {
    cli();
    cycles_start();
    uint8_t next = fsm_transition(current_state, event);
    cycle_t cycles = cycles_stop();
    sei();
    current_state = next;
    return cycles;
}
"""
    else:  # esp32
        measured = """
static cycle_t measured_transition(uint8_t event) {
    noInterrupts();
    uint32_t t0 = cycles_start();
    uint8_t next = fsm_transition(current_state, event);
    cycle_t cycles = cycles_stop(t0);
    interrupts();
    current_state = next;
    return cycles;
}
"""
    body = measured + """
static void run_benchmark(void) {
    const uint16_t ITER = 1000;
    cycle_t min_c = 0xFFFFFFFF, max_c = 0;
    uint32_t sum_c = 0;

    uart_puts("Running benchmark (");
    uart_put_uint(ITER);
    uart_puts(" transitions)...\\r\\n");

    for (uint16_t i = 0; i < ITER; i++) {
        cycle_t c = measured_transition(EV_0);
        if (c < min_c) min_c = c;
        if (c > max_c) max_c = c;
        sum_c += c;
    }

    uart_puts("N_STATES: "); uart_put_uint(NUM_STATES); uart_puts("\\r\\n");
    uart_puts("N_EVENTS: "); uart_put_uint(NUM_EVENTS); uart_puts("\\r\\n");
    uart_puts("Min cycles: "); uart_put_uint((uint32_t)min_c); uart_puts("\\r\\n");
    uart_puts("Max cycles: "); uart_put_uint((uint32_t)max_c); uart_puts("\\r\\n");
    uart_puts("Avg cycles: "); uart_put_uint((uint32_t)(sum_c / ITER)); uart_puts("\\r\\n");
}
"""
    if platform == "avr":
        body += """
void setup() {
    uart_init();
    uart_puts("\\r\\nParametrized FSM ready.\\r\\n");
}

void loop() {
    if (uart_available()) {
        char c = uart_getc();
        if (c == 'b') { run_benchmark(); return; }
    }
}
"""
    else:
        body += """
void setup() {
    uart_init();
    uart_puts("\\r\\nParametrized FSM ready.\\r\\n");
}

void loop() {
    if (uart_available()) {
        char c = uart_getc();
        if (c == 'b') { run_benchmark(); }
    }
}
"""
    return body


# ==============================================================================
# GENERATORI PO PATTERN-U
# ==============================================================================

def gen_nested_switch(n_states, n_events):
    """Ugnjezdeni switch(state){ switch(event){...} }. Kompajler cesto svede
    switch sa gustim, uzastopnim case vrednostima na jump-table (O(1)), ali to
    NIJE garantovano (zavisi od optimizacije/gustine) - zato je zanimljiv za
    poredjenje sa indexed_table koji je O(1) PO KONSTRUKCIJI."""
    lines = ["static uint8_t fsm_transition(uint8_t state, uint8_t event) {",
             "    switch (state) {"]
    for s in range(n_states):
        nxt = (s + 1) % n_states
        lines.append(f"    case {s}:")
        lines.append("        switch (event) {")
        lines.append(f"        case EV_0: return {nxt};")
        for e in range(1, n_events):
            lines.append(f"        case EV_{e}: return {s};")
        lines.append(f"        default: return {s};")
        lines.append("        }")
    lines.append("    default: return state;")
    lines.append("    }")
    lines.append("}")
    return "\n".join(lines)


def gen_array_of_structs(n_states, n_events):
    """Linearna pretraga kroz SPARSE niz {state,event,next_state} - samo N
    validnih (napredujucih) tranzicija je u tabeli (self-loop na EV_1..EV_(M-1)
    se NE upisuje, fsm_transition vraca isto stanje kad ne nadje poklapanje -
    ovo cuva O(broj validnih tranzicija) memoriju, kao u fsm_array_of_structs.cpp)."""
    entries = [f"    {{ {s}, EV_0, {(s+1) % n_states} }}," for s in range(n_states)]
    table = "\n".join(entries)
    return f"""typedef struct {{ uint8_t state; uint8_t event; uint8_t next_state; }} Transition;

static const Transition transition_table[] = {{
{table}
}};
#define TABLE_SIZE (sizeof(transition_table) / sizeof(Transition))

static uint8_t fsm_transition(uint8_t state, uint8_t event) {{
    for (uint16_t i = 0; i < TABLE_SIZE; i++) {{
        if (transition_table[i].state == state && transition_table[i].event == event) {{
            return transition_table[i].next_state;
        }}
    }}
    return state;
}}"""


def gen_indexed_table(n_states, n_events):
    """Gusta NxM matrica - SVE (state,event) kombinacije popunjene, O(1) lookup
    ali O(N*M) memorija (kvadratni rast sa brojem stanja ako M raste sa N,
    linearan po N ako je M fiksno) - vidi teorijsku tabelu u ESP32_sekcija.docx."""
    rows = []
    for s in range(n_states):
        row = [str((s + 1) % n_states)] + [str(s)] * (n_events - 1)
        rows.append("    { " + ", ".join(row) + " },")
    table = "\n".join(rows)
    return f"""static const uint8_t transition_table[NUM_STATES][NUM_EVENTS] = {{
{table}
}};

static inline uint8_t fsm_transition(uint8_t state, uint8_t event) {{
    return transition_table[state][event];
}}"""


def gen_state_object(n_states, n_events):
    """Niz pokazivaca na funkcije, indeksiran SAMO po state - svaki handler
    sam odlucuje sta radi sa event-om (if-else lanac unutra), O(states) memorija
    za tabelu pokazivaca + O(events) grananje unutar handlera."""
    handlers = []
    for s in range(n_states):
        nxt = (s + 1) % n_states
        handlers.append(f"static uint8_t state_{s}_handle(uint8_t event) {{")
        handlers.append(f"    if (event == EV_0) return {nxt};")
        handlers.append(f"    return {s};")
        handlers.append("}")
    handler_defs = "\n".join(handlers)
    table = ",\n".join([f"    state_{s}_handle" for s in range(n_states)])
    return f"""{handler_defs}

typedef uint8_t (*StateHandler)(uint8_t event);
static StateHandler const state_handlers[NUM_STATES] = {{
{table}
}};

static inline uint8_t fsm_transition(uint8_t state, uint8_t event) {{
    return state_handlers[state](event);
}}"""


def gen_hsm_nested(depth, n_events):
    """Lanac roditelja dubine D: leaf_0 (najdublji list) -> leaf_1 -> ... ->
    leaf_(D-1) (top/root, parent = -1). SAMO root obradjuje EV_0 - svi ostali
    handleri vracaju EVENT_UNHANDLED za EV_0, sto forsira bubbling kroz D-1
    nivoa pre nego sto se dogadjaj konzumira. Ovo direktno mapira teorijsko
    O(dubina hijerarhije) iz ESP32_sekcija.docx (poredjenje sa Test 4a/4b iz
    Arduino_sekcija.docx, gde je D=2 poseban slucaj ovog opsteg mehanizma)."""
    n_states = depth
    handlers = []
    for s in range(n_states):
        if s == n_states - 1:  # root/top state - jedini koji obradjuje EV_0
            handlers.append(f"static uint8_t state_{s}_handle(uint8_t event) {{")
            handlers.append(f"    if (event == EV_0) return 0;")  # vrati na najdublji list
            handlers.append("    return EVENT_UNHANDLED;")
            handlers.append("}")
        else:
            handlers.append(f"static uint8_t state_{s}_handle(uint8_t event) {{")
            handlers.append("    return EVENT_UNHANDLED;")  # nikad ne obradjuje - forsira bubbling
            handlers.append("}")
    handler_defs = "\n".join(handlers)

    nodes = []
    for s in range(n_states):
        parent = s + 1 if s < n_states - 1 else -1
        nodes.append(f"    {{ state_{s}_handle, {parent} }},")
    node_table = "\n".join(nodes)

    return f"""#define EVENT_UNHANDLED 0xFF
typedef uint8_t (*StateHandler)(uint8_t event);
typedef struct {{ StateHandler handler; int16_t parent; }} StateNode;

{handler_defs}

static const StateNode state_table[NUM_STATES] = {{
{node_table}
}};

static uint8_t fsm_transition(uint8_t state, uint8_t event) {{
    int16_t node = state;
    while (node != -1) {{
        uint8_t result = state_table[node].handler(event);
        if (result != EVENT_UNHANDLED) return result;
        node = state_table[node].parent;
    }}
    return state;
}}"""


PATTERN_GENERATORS = {
    "nested_switch": gen_nested_switch,
    "array_of_structs": gen_array_of_structs,
    "indexed_table": gen_indexed_table,
    "state_object": gen_state_object,
}


def generate_cpp(pattern, platform, n_states=None, n_events=3, depth=None):
    header = AVR_HEADER if platform == "avr" else ESP32_HEADER

    if pattern == "hsm_nested":
        assert depth is not None, "hsm_nested zahteva --depth"
        n_states_eff = depth
        n_events_eff = n_events
        fsm_code = gen_hsm_nested(depth, n_events)
    else:
        assert n_states is not None, "ovaj pattern zahteva --n"
        n_states_eff = n_states
        n_events_eff = n_events
        fsm_code = PATTERN_GENERATORS[pattern](n_states, n_events)

    enums = "enum Event {" + ", ".join([f"EV_{e}" for e in range(n_events_eff)]) + "};"

    preamble = f"""/*
 * AUTO-GENERISANO fajlom fsm_generator.py
 * Pattern: {pattern} | Platforma: {platform} | N_STATES={n_states_eff} | N_EVENTS={n_events_eff}
 * NE MENJATI RUCNO - regenerisati skriptom radi konzistentnosti sweep-a.
 */
{header}

#define NUM_STATES {n_states_eff}
#define NUM_EVENTS {n_events_eff}
{enums}

static volatile uint8_t current_state = 0;

"""
    footer = gen_benchmark_and_main(platform)
    return preamble + fsm_code + "\n" + footer


# ==============================================================================
# SWEEP: generisanje kompletnog seta fajlova + platformio.ini env blokova
# ==============================================================================

SWEEP_CONFIG = {
    "n_values": [4, 8, 16, 32, 64, 128],          # broj stanja (fiksno n_events=3)
    "depth_values": [2, 4, 6, 8, 10, 12],          # dubina hijerarhije za hsm_nested
    "n_events": 3,
    "patterns_scaling_states": ["nested_switch", "array_of_structs", "indexed_table", "state_object"],
    "platforms": ["avr", "esp32"],
}


def env_name(pattern, platform, size):
    return f"fsm_{pattern}_{platform}_n{size}"


def sweep_generate(outdir="output/generated_fsm"):
    os.makedirs(outdir, exist_ok=True)
    manifest = []  # (env_name, filename, pattern, platform, size_param, size_value)

    for platform in SWEEP_CONFIG["platforms"]:
        for pattern in SWEEP_CONFIG["patterns_scaling_states"]:
            for n in SWEEP_CONFIG["n_values"]:
                code = generate_cpp(pattern, platform, n_states=n, n_events=SWEEP_CONFIG["n_events"])
                ename = env_name(pattern, platform, n)
                fname = f"{ename}.cpp"
                with open(os.path.join(outdir, fname), "w") as f:
                    f.write(code)
                manifest.append({
                    "env": ename, "file": fname, "pattern": pattern,
                    "platform": platform, "param": "n_states", "value": n
                })
        for d in SWEEP_CONFIG["depth_values"]:
            code = generate_cpp("hsm_nested", platform, depth=d, n_events=SWEEP_CONFIG["n_events"])
            ename = env_name("hsm_nested", platform, d)
            fname = f"{ename}.cpp"
            with open(os.path.join(outdir, fname), "w") as f:
                f.write(code)
            manifest.append({
                "env": ename, "file": fname, "pattern": "hsm_nested",
                "platform": platform, "param": "depth", "value": d
            })

    return manifest


def write_platformio_ini(manifest, platform, outpath="output/platformio_sweep.ini"):
    """Generise kompletan platformio.ini sa env-om po fajlu iz manifest-a,
    filtrirano na jednu platformu (avr -> board uno, esp32 -> board esp32dev).
    auto_benchmark.py treba samo da menja default_envs / iterira kroz envs
    umesto da regenerise ceo ini kao sada."""
    envs = [m for m in manifest if m["platform"] == platform]
    board_block = (
        "platform = atmelavr\nboard = uno\nframework = arduino\n"
        if platform == "avr" else
        "platform = espressif32\nboard = esp32dev\nframework = arduino\n"
    )
    lines = ["[platformio]", "default_envs = " + envs[0]["env"] if envs else "", ""]
    # ISPRAVKA: generisani .cpp fajlovi zive u output/generated_fsm/, ne u
    # podrazumevanom src/ - bez ovoga pio ne bi nasao nijedan build_src_filter
    # fajl jer bi trazio u pogresnom direktorijumu.
    lines.append("src_dir = output/generated_fsm")
    lines.append("")
    lines.append("[env]")
    lines.append(board_block)
    lines.append("monitor_speed = 9600")
    lines.append("")
    for m in envs:
        lines.append(f"[env:{m['env']}]")
        lines.append(f"build_src_filter = +<{m['file']}> -<*>")
        lines.append("")
    with open(outpath, "w") as f:
        f.write("\n".join(lines))
    return outpath


# ==============================================================================
# CLI
# ==============================================================================

def main():
    parser = argparse.ArgumentParser(description="Generator parametrizovanih FSM .cpp fajlova")
    parser.add_argument("--pattern", choices=list(PATTERN_GENERATORS.keys()) + ["hsm_nested"])
    parser.add_argument("--n", type=int, help="broj stanja (za sve pattern-e osim hsm_nested)")
    parser.add_argument("--events", type=int, default=3, help="broj dogadjaja (default 3)")
    parser.add_argument("--depth", type=int, help="dubina hijerarhije (samo hsm_nested)")
    parser.add_argument("--platform", choices=["avr", "esp32"], default="avr")
    parser.add_argument("-o", "--output", help="izlazni .cpp fajl")
    parser.add_argument("--sweep", action="store_true", help="generisi kompletan sweep set po SWEEP_CONFIG")
    args = parser.parse_args()

    if args.sweep:
        manifest = sweep_generate()
        import csv
        with open("output/sweep_manifest.csv", "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=["env", "file", "pattern", "platform", "param", "value"])
            w.writeheader()
            w.writerows(manifest)
        write_platformio_ini(manifest, "avr", "output/platformio_sweep_avr.ini")
        write_platformio_ini(manifest, "esp32", "output/platformio_sweep_esp32.ini")
        print(f"Generisano {len(manifest)} fajlova u output/generated_fsm/")
        print("Manifest: output/sweep_manifest.csv")
        print("PlatformIO ini fajlovi: output/platformio_sweep_avr.ini, output/platformio_sweep_esp32.ini")
        return

    if not args.pattern:
        parser.error("potrebno --pattern ili --sweep")

    code = generate_cpp(args.pattern, args.platform, n_states=args.n, n_events=args.events, depth=args.depth)
    if args.output:
        with open(args.output, "w") as f:
            f.write(code)
        print(f"Napisano: {args.output}")
    else:
        print(code)


if __name__ == "__main__":
    main()

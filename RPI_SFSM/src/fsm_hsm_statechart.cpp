/*
 * Radovi:
 *  - Harel, D. (1987). "Statecharts: a Visual Formalism for Complex Systems."
 *    Science of Computer Programming, Vol. 8, pp. 231-274 (entry/exit akcije,
 *    hijerarhijske tranzicije koje se racunaju preko LCA - najnizeg zajednickog
 *    pretka).
 *  - Sunitha, E. V., Samuel, P. (2019). "Automatic Code Generation From UML
 *    State Chart Diagrams." IEEE Access, 7:8591-8608 - implementirano i u
 *    Carlgren & Oskarsson (2023) UPTEC F 23044, sek. 3.11 "Hierarchical State
 *    Pattern" (Figure 12).
 *  - Adamczyk, P. "The Anthology of the Finite State Machine Design Patterns"
 *    - paterni "Basic Statechart" i "Hierarchical Statechart" (entry()/exit()
 *    metode po stanju).
 *
 * Access Control FSM - Manual HSM sa PUNIM entry()/exit() lancem akcija -
 * Raspberry Pi / Linux port. Razlika od fsm_hsm_nested.cpp: taj fajl radi SAMO
 * bubbling dogadjaja bez ikakvih entry/exit poziva. Ovde se, kao u pravoj
 * statechart semantici, pri svakoj tranziciji izlazi iz svih stanja od
 * trenutnog lista do owning_node-a (ukljucujuci), pa ulazi od LCA do ciljnog
 * lista. Vidi fsm_nested_switch_return.cpp za napomene o Linux adaptaciji.
 *
 * FSM: IDLE, CHECKING, DENIED (top-level) + GRANTED (superstate) sa decom
 * NORMAL_ACCESS i ADMIN_ACCESS - identicna hijerarhija kao u fsm_hsm_nested.cpp.
 * Events: EV_VALID, EV_INVALID, EV_TIMEOUT, EV_ADMIN
 *
 * Komande (bez pritiska na Enter):
 *   '1' -> EV_VALID
 *   '0' -> EV_INVALID
 *   't' -> EV_TIMEOUT
 *   'a' -> EV_ADMIN (toggle NORMAL_ACCESS <-> ADMIN_ACCESS unutar GRANTED)
 *   'b' -> run automatic benchmark (1000 transitions), meri INHERITED EV_TIMEOUT
 *          trosak (bubbling + pun exit/entry lanac), za poredjenje sa fsm_hsm_nested.cpp
 *   'r' -> print current resource usage (VmRSS/VmHWM) on demand
 *   'q' -> quit
 */
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <cerrno>
#include <unistd.h>
#include <sched.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <linux/perf_event.h>
#include <asm/unistd.h>

enum State {
    STATE_IDLE = 0,
    STATE_CHECKING,
    STATE_DENIED,
    STATE_GRANTED,        // superstate
    STATE_NORMAL_ACCESS,  // substate of GRANTED
    STATE_ADMIN_ACCESS,   // substate of GRANTED
    NUM_STATES
};
enum Event { EV_VALID = 0, EV_INVALID, EV_TIMEOUT, EV_ADMIN };

#define EVENT_UNHANDLED 0xFF

static uint8_t current_state = STATE_IDLE;
static int perf_fd = -1;
static struct termios orig_termios;

static long perf_event_open(struct perf_event_attr *hw_event, pid_t pid, int cpu, int group_fd, unsigned long flags) {
    return syscall(__NR_perf_event_open, hw_event, pid, cpu, group_fd, flags);
}

static void cycles_init(void) {
    struct perf_event_attr pe;
    memset(&pe, 0, sizeof(pe));
    pe.type = PERF_TYPE_HARDWARE;
    pe.size = sizeof(pe);
    pe.config = PERF_COUNT_HW_CPU_CYCLES;
    pe.disabled = 1;
    pe.exclude_kernel = 1;
    pe.exclude_hv = 1;

    perf_fd = perf_event_open(&pe, 0, -1, -1, 0);
    if (perf_fd == -1) {
        fprintf(stderr, "perf_event_open failed (errno=%d: %s)\n", errno, strerror(errno));
        fprintf(stderr, "Probaj: sudo sysctl -w kernel.perf_event_paranoid=-1  (ili pokreni sa sudo)\n");
        exit(1);
    }
}

static inline void cycles_start(void) {
    ioctl(perf_fd, PERF_EVENT_IOC_RESET, 0);
    ioctl(perf_fd, PERF_EVENT_IOC_ENABLE, 0);
}

static inline uint64_t cycles_stop(void) {
    ioctl(perf_fd, PERF_EVENT_IOC_DISABLE, 0);
    uint64_t count = 0;
    ssize_t rd = read(perf_fd, &count, sizeof(count));
    (void)rd;
    return count;
}

static void pin_to_cpu0(void) {
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(0, &set);
    sched_setaffinity(0, sizeof(set), &set);
}

static void restore_terminal(void) {
    tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios);
}

static void set_raw_terminal(void) {
    tcgetattr(STDIN_FILENO, &orig_termios);
    struct termios raw = orig_termios;
    raw.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    atexit(restore_terminal);
}

static void print_resource_usage(void) {
    printf("\n--- Linux resource usage (proces) ---\n");
    FILE *f = fopen("/proc/self/status", "r");
    if (f) {
        char line[256];
        while (fgets(line, sizeof(line), f)) {
            if (!strncmp(line, "VmRSS:", 6) || !strncmp(line, "VmHWM:", 6) ||
                !strncmp(line, "VmSize:", 7) || !strncmp(line, "Threads:", 8)) {
                printf("%s", line);
            }
        }
        fclose(f);
    }
    printf("\n");
}

typedef uint8_t (*StateHandler)(uint8_t event);
typedef void (*StateAction)(void);

typedef struct {
    StateHandler handler;
    StateAction enter;
    StateAction exit;
    int8_t parent;
} StateNode;

static uint8_t state_idle_handle(uint8_t event) {
    if (event == EV_VALID) return STATE_CHECKING;
    return EVENT_UNHANDLED;
}

static uint8_t state_checking_handle(uint8_t event) {
    if (event == EV_VALID)   return STATE_NORMAL_ACCESS;
    if (event == EV_INVALID) return STATE_DENIED;
    return EVENT_UNHANDLED;
}

static uint8_t state_denied_handle(uint8_t event) {
    if (event == EV_TIMEOUT) return STATE_IDLE;
    return EVENT_UNHANDLED;
}

static uint8_t state_granted_handle(uint8_t event) {
    if (event == EV_TIMEOUT) return STATE_IDLE;
    return EVENT_UNHANDLED;
}

static uint8_t state_normal_access_handle(uint8_t event) {
    if (event == EV_ADMIN) return STATE_ADMIN_ACCESS;
    return EVENT_UNHANDLED;
}

static uint8_t state_admin_access_handle(uint8_t event) {
    if (event == EV_ADMIN) return STATE_NORMAL_ACCESS;
    return EVENT_UNHANDLED;
}

// entry/exit akcije su namerno prazne (no-op) - zanima nas SAMO trosak poziva
static void enter_idle(void)          {}
static void exit_idle(void)           {}
static void enter_checking(void)      {}
static void exit_checking(void)       {}
static void enter_denied(void)        {}
static void exit_denied(void)         {}
static void enter_granted(void)       {}
static void exit_granted(void)        {}
static void enter_normal_access(void) {}
static void exit_normal_access(void)  {}
static void enter_admin_access(void)  {}
static void exit_admin_access(void)   {}

static const StateNode state_table[NUM_STATES] = {
    /* STATE_IDLE          */ { state_idle_handle,          enter_idle,          exit_idle,          -1 },
    /* STATE_CHECKING      */ { state_checking_handle,      enter_checking,      exit_checking,      -1 },
    /* STATE_DENIED        */ { state_denied_handle,        enter_denied,        exit_denied,        -1 },
    /* STATE_GRANTED       */ { state_granted_handle,       enter_granted,       exit_granted,       -1 },
    /* STATE_NORMAL_ACCESS */ { state_normal_access_handle, enter_normal_access, exit_normal_access, STATE_GRANTED },
    /* STATE_ADMIN_ACCESS  */ { state_admin_access_handle,  enter_admin_access,  exit_admin_access,  STATE_GRANTED },
};

static uint8_t fsm_transition(uint8_t state, uint8_t event) {
    int8_t owning_node = state;
    uint8_t result = EVENT_UNHANDLED;
    while (owning_node != -1) {
        result = state_table[owning_node].handler(event);
        if (result != EVENT_UNHANDLED) break;
        owning_node = state_table[owning_node].parent;
    }
    if (owning_node == -1) {
        return state;
    }

    uint8_t next_state = result;

    int8_t node = state;
    while (1) {
        state_table[node].exit();
        if (node == owning_node) break;
        node = state_table[node].parent;
    }

    int8_t preserved_ancestor = state_table[owning_node].parent;
    int8_t path[NUM_STATES];
    uint8_t count = 0;
    int8_t n = next_state;
    while (state_table[n].parent != preserved_ancestor) {
        path[count++] = n;
        n = state_table[n].parent;
    }
    path[count++] = n;
    for (int8_t i = count - 1; i >= 0; i--) {
        state_table[path[i]].enter();
    }

    return next_state;
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

static uint64_t measured_transition(uint8_t event) {
    cycles_start();
    uint8_t next = fsm_transition(current_state, event);
    uint64_t cycles = cycles_stop();
    current_state = next;
    return cycles;
}

static void run_benchmark(void) {
    const int N = 1000;
    uint64_t min_c = UINT64_MAX, max_c = 0, sum_c = 0;

    printf("Running benchmark (%d transitions, forcing state=NORMAL_ACCESS, event=EV_TIMEOUT each time)...\n", N);
    for (int i = 0; i < N; i++) {
        current_state = STATE_NORMAL_ACCESS;
        uint64_t c = measured_transition(EV_TIMEOUT);
        if (c < min_c) min_c = c;
        if (c > max_c) max_c = c;
        sum_c += c;
    }

    printf("Min cycles: %lu\n", (unsigned long)min_c);
    printf("Max cycles: %lu\n", (unsigned long)max_c);
    printf("Avg cycles: %lu\n", (unsigned long)(sum_c / N));
    current_state = STATE_IDLE;
    print_resource_usage();
}

int main(void) {
    pin_to_cpu0();
    cycles_init();
    set_raw_terminal();

    printf("\nAccess FSM - HSM Statechart (entry/exit) pattern ready (Raspberry Pi / Linux).\n");
    printf("Commands: 1=VALID 0=INVALID t=TIMEOUT a=ADMIN_TOGGLE b=BENCHMARK r=RESOURCES q=QUIT\n");
    printf("Current state: IDLE\n");
    print_resource_usage();

    char c;
    while (read(STDIN_FILENO, &c, 1) == 1) {
        uint8_t event;
        if (c == 'q') {
            break;
        } else if (c == 'b') {
            run_benchmark();
            continue;
        } else if (c == 'r') {
            print_resource_usage();
            continue;
        } else if (c == '1') {
            event = EV_VALID;
        } else if (c == '0') {
            event = EV_INVALID;
        } else if (c == 't') {
            event = EV_TIMEOUT;
        } else if (c == 'a') {
            event = EV_ADMIN;
        } else {
            continue;
        }

        uint64_t cycles = measured_transition(event);
        printf("Event handled -> State: %s | Cycles: %lu\n", state_name(current_state), (unsigned long)cycles);
    }

    close(perf_fd);
    return 0;
}

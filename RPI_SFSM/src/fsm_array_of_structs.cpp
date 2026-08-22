/*
 * Rad: A. Kumar, "How to implement finite state machine in C," aticleworld.com,
 * avgust 2017 - originalni clanak na kom je zasnovan ovaj pristup (direktno
 * next_state u struct-u, linearna pretraga). Pominje se i u: Carlgren,
 * Oskarsson (2023) UPTEC F 23044, sek. 2.8.2 "Array of Structs" (referenca [2]
 * = isti A. Kumar clanak). Za varijantu sa eventHandler pokazivacem na funkciju
 * (Figure 9 u radu), vidi fsm_array_of_structs_handler.cpp.
 *
 * Access Control FSM - Array of Structs Pattern - Raspberry Pi / Linux port.
 * Transition lookup je LINEARNA PRETRAGA kroz niz {state, event, next_state}
 * struct-ova. Vidi fsm_nested_switch_return.cpp za napomene o Linux adaptaciji.
 *
 * Komande (bez pritiska na Enter):
 *   '1' -> EV_VALID
 *   '0' -> EV_INVALID
 *   't' -> EV_TIMEOUT
 *   'b' -> run automatic benchmark (1000 transitions), prints min/avg/max cycles
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

enum State { STATE_IDLE = 0, STATE_CHECKING, STATE_GRANTED, STATE_DENIED };
enum Event { EV_VALID = 0, EV_INVALID, EV_TIMEOUT };

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

typedef struct {
    uint8_t state;
    uint8_t event;
    uint8_t next_state;
} Transition;

static const Transition transition_table[] = {
    { STATE_IDLE,     EV_VALID,   STATE_CHECKING },
    { STATE_CHECKING, EV_VALID,   STATE_GRANTED  },
    { STATE_CHECKING, EV_INVALID, STATE_DENIED   },
    { STATE_GRANTED,  EV_TIMEOUT, STATE_IDLE     },
    { STATE_DENIED,   EV_TIMEOUT, STATE_IDLE     },
};

#define TABLE_SIZE (sizeof(transition_table) / sizeof(Transition))

static uint8_t fsm_transition(uint8_t state, uint8_t event) {
    for (uint8_t i = 0; i < TABLE_SIZE; i++) {
        if (transition_table[i].state == state && transition_table[i].event == event) {
            return transition_table[i].next_state;
        }
    }
    return state; // no match -> stay in current state
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
    uint8_t ev_cycle[3] = { EV_VALID, EV_VALID, EV_TIMEOUT };

    printf("Running benchmark (%d transitions)...\n", N);
    for (int i = 0; i < N; i++) {
        uint8_t event = ev_cycle[i % 3];
        uint64_t c = measured_transition(event);
        if (c < min_c) min_c = c;
        if (c > max_c) max_c = c;
        sum_c += c;
    }

    printf("Min cycles: %lu\n", (unsigned long)min_c);
    printf("Max cycles: %lu\n", (unsigned long)max_c);
    printf("Avg cycles: %lu\n", (unsigned long)(sum_c / N));
    print_resource_usage();
}

int main(void) {
    pin_to_cpu0();
    cycles_init();
    set_raw_terminal();

    printf("\nAccess FSM - Array of Structs pattern ready (Raspberry Pi / Linux).\n");
    printf("Commands: 1=VALID 0=INVALID t=TIMEOUT b=BENCHMARK r=RESOURCES q=QUIT\n");
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
        } else {
            continue;
        }

        uint64_t cycles = measured_transition(event);
        printf("Event handled -> State: %s | Cycles: %lu\n", state_name(current_state), (unsigned long)cycles);
    }

    close(perf_fd);
    return 0;
}

/*
 * NAPOMENA: ovaj fajl NE modeluje konkretan FSM pattern iz literature - to je
 * nasa sopstvena provera metodologije merenja (dokazuje da fsm_transition()
 * trosak ne zavisi od toga koliko je FSM prethodno stajao u stanju, cak i uz
 * pravi OS tajmer koji izaziva cekanje). Srodna metodologija merenja ciklusa
 * (broj ciklusa umesto apsolutnog vremena) opisana je u: Katin, P., Chmelov,
 * V., Shemaev, V. (2020). "Development of Typical 'State' Software Patterns
 * for Cortex-M Microcontrollers in Real Time." Eastern-European Journal of
 * Enterprise Technologies, 3/9(105) - sek. 5.3.
 *
 * Time-independence proof - Raspberry Pi / Linux port (POSIX timer_create()
 * + signal umesto AVR Timer0 CTC+ISR / ESP32 esp_timer).
 *
 * Proves that fsm_transition() cost does not depend on how long the FSM stayed
 * in the previous state, even when that wait is driven by a real OS timer + signal.
 *
 * POSIX timer_create(CLOCK_MONOTONIC, SIGEV_SIGNAL) salje SIGRTMIN signal kad
 * istekne ~3s jednokratni alarm; signal handler samo postavlja
 * volatile sig_atomic_t flag (bezbedno unutar signal handler-a), glavna petlja
 * ga polluje - isti duh kao AVR ISR / ESP32 esp_timer callback.
 *
 * Komande (bez pritiska na Enter):
 *   'a' -> Scenario A (immediate transition from CHECKING, no wait)
 *   'b' -> Scenario B (transition from GRANTED after ~3s tajmer-driven wait)
 *   'r' -> repeat both scenarios 5x and print min/avg/max for each
 *   'u' -> print current resource usage (VmRSS/VmHWM) on demand
 *   'q' -> quit
 */
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <cerrno>
#include <csignal>
#include <ctime>
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

// ---------- POSIX tajmer sa signalom (analog AVR Timer0 ISR / ESP32 esp_timer) ----------
#define DURATION_WAIT_SEC 3

static volatile sig_atomic_t duration_elapsed = 0;
static timer_t duration_timer_id;

static void duration_timer_handler(int sig) {
    (void)sig;
    duration_elapsed = 1;
}

static void wait_for_state_duration(void) {
    duration_elapsed = 0;

    struct sigevent sev;
    memset(&sev, 0, sizeof(sev));
    sev.sigev_notify = SIGEV_SIGNAL;
    sev.sigev_signo = SIGRTMIN;
    sev.sigev_value.sival_ptr = &duration_timer_id;
    timer_create(CLOCK_MONOTONIC, &sev, &duration_timer_id);

    struct itimerspec its;
    memset(&its, 0, sizeof(its));
    its.it_value.tv_sec = DURATION_WAIT_SEC;
    its.it_value.tv_nsec = 0;
    its.it_interval.tv_sec = 0; // jednokratni alarm (one-shot)
    its.it_interval.tv_nsec = 0;
    timer_settime(duration_timer_id, 0, &its, NULL);

    while (!duration_elapsed) {
        // CPU je slobodan ovde - moglo bi pause() umesto busy-wait-a
    }
    timer_delete(duration_timer_id);
}

// ---------- Nested Switch FSM transition (ista logika kao originalni test) ----------
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

static uint64_t measured_transition(uint8_t event) {
    cycles_start();
    uint8_t next = fsm_transition(current_state, event);
    uint64_t cycles = cycles_stop();
    current_state = next;
    return cycles;
}

static uint64_t scenario_A(void) {
    current_state = STATE_CHECKING;
    return measured_transition(EV_VALID);
}

static uint64_t scenario_B(void) {
    current_state = STATE_GRANTED;
    wait_for_state_duration();
    return measured_transition(EV_TIMEOUT);
}

static void run_comparison(int N) {
    uint64_t minA = UINT64_MAX, maxA = 0, sumA = 0;
    uint64_t minB = UINT64_MAX, maxB = 0, sumB = 0;

    printf("Running %d repetitions of each scenario (timer-driven ~3s wait each time)...\n", N);

    for (int i = 0; i < N; i++) {
        uint64_t a = scenario_A();
        if (a < minA) minA = a;
        if (a > maxA) maxA = a;
        sumA += a;

        uint64_t b = scenario_B();
        if (b < minB) minB = b;
        if (b > maxB) maxB = b;
        sumB += b;
    }

    printf("\n--- Scenario A: CHECKING->GRANTED, NO wait ---\n");
    printf("Min: %lu  Avg: %lu  Max: %lu (cycles)\n", (unsigned long)minA, (unsigned long)(sumA / N), (unsigned long)maxA);

    printf("--- Scenario B: GRANTED->IDLE, AFTER ~3s timer wait ---\n");
    printf("Min: %lu  Avg: %lu  Max: %lu (cycles)\n", (unsigned long)minB, (unsigned long)(sumB / N), (unsigned long)maxB);

    printf("\nAko se dva opsega poklapaju, trosak tranzicije je nezavisan od trajanja\n");
    printf("prethodnog stanja, cak i uz OS tajmer + signal.\n");
    print_resource_usage();
}

int main(void) {
    pin_to_cpu0();
    cycles_init();
    set_raw_terminal();
    signal(SIGRTMIN, duration_timer_handler);

    printf("\nTime-independence proof (Raspberry Pi / Linux, timer_create) ready.\n");
    printf("Commands: a=Scenario A (immediate)  b=Scenario B (~3s wait)  r=Repeat comparison (5x)  u=RESOURCES q=QUIT\n");

    char c;
    while (read(STDIN_FILENO, &c, 1) == 1) {
        if (c == 'q') {
            break;
        } else if (c == 'a') {
            uint64_t cyc = scenario_A();
            printf("Scenario A -> State: %s | Cycles: %lu\n", state_name(current_state), (unsigned long)cyc);
        } else if (c == 'b') {
            printf("Waiting ~3s (timer_create one-shot)...\n");
            uint64_t cyc = scenario_B();
            printf("Scenario B -> State: %s | Cycles: %lu\n", state_name(current_state), (unsigned long)cyc);
        } else if (c == 'r') {
            run_comparison(5);
        } else if (c == 'u') {
            print_resource_usage();
        }
    }

    close(perf_fd);
    return 0;
}

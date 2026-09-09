/*
 *
 * MQTT (mosquitto, dva klijenta) FSM - "Santic" Lookup Table Pattern (void
 * akcijske procedure), 16 stanja, 9 dogadjaja.
 * UART commands (9600 baud):
 *   num(dec 0-8) -> event index (redosled kao u enum Event)
 *   'b' -> run automatic benchmark (1000 transitions), prints min/avg/max cycles
 */

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdint.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>

enum State
{
    S0, S1, S2, S3, S4, S5, S6, S7, S8, S9, S10, S11, S12, S13, S14, S15, S16,
    S17, S18, S19, S20, S21, S22, S23, S24, S25, S26, S27, S28, S29, S30, S31,
    S32, S33, S34, S35, S36, S37, S38, S39, S40, S41, S42, S43, S44, S45, S46,
    S47, S48, S49, S50, S51, S52, S53, S54
};

enum Event
{
    CLOSECONNECTION, ACK_PSH_VV1, SYN_ACK_VV0, RST_VV0, ACCEPT, FIN_ACK_VV0, LISTEN, SYN_VV0,
    RCV, ACK_RST_VV0, CLOSE, SEND, ACK_VV0 
};

#define NUM_STATES 55
#define NUM_EVENTS 13

static volatile uint8_t current_state = S0;

#define BAUD 9600
#define UBRR_VAL ((F_CPU / (16UL * BAUD)) - 1)

static void uart_init(void) {
    UBRR0H = (uint8_t)(UBRR_VAL >> 8);
    UBRR0L = (uint8_t)UBRR_VAL;
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8N1
}

static void uart_putc(char c) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

static void uart_puts(const char *s) {
    while (*s) uart_putc(*s++);
}

static void uart_put_uint(uint16_t v) {
    char buf[6];
    uint8_t i = 0;
    if (v == 0) { uart_putc('0'); return; }
    while (v > 0) { buf[i++] = '0' + (v % 10); v /= 10; }
    while (i > 0) uart_putc(buf[--i]);
}

static uint8_t uart_available(void) {
    return (UCSR0A & (1 << RXC0)) != 0;
}

static char uart_getc(void) {
    while (!(UCSR0A & (1 << RXC0)));
    return UDR0;
}

static inline void cycles_start(void) {
    TCCR1B = 0;
    TCNT1 = 0;
    TCCR1B = (1 << CS10); // no prescaler
}

static inline uint16_t cycles_stop(void) {
    TCCR1B = 0;
    return TCNT1;
}

// ---------- Akcijske procedure (void) - svaka SAMA upisuje current_state ----------
// Generisano i verifikovano protiv .dot specifikacije (svih 144 kombinacija).
// State S0 action handlers
static void action_s0_closeconnection(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s0_ack_psh_vv1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s0_syn_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s0_rst_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s0_accept(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s0_fin_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s0_listen(void) { current_state = S1; }
static void action_s0_syn_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s0_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s0_ack_rst_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s0_close(void) { current_state = S2; }
static void action_s0_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s0_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }

// State S1 action handlers
static void action_s1_closeconnection(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s1_ack_psh_vv1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s1_syn_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s1_rst_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s1_accept(void) { current_state = S4; }
static void action_s1_fin_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s1_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s1_syn_vv0(void) { current_state = S3; }
static void action_s1_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s1_ack_rst_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s1_close(void) { current_state = S2; }
static void action_s1_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s1_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }

// State S2 action handlers
static void action_s2_closeconnection(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s2_ack_psh_vv1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s2_syn_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s2_rst_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s2_accept(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s2_fin_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s2_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s2_syn_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s2_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s2_ack_rst_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s2_close(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s2_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s2_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }

// State S3 action handlers
static void action_s3_closeconnection(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s3_ack_psh_vv1(void) { current_state = S8; }
static void action_s3_syn_ack_vv0(void) { current_state = S6; }
static void action_s3_rst_vv0(void) { current_state = S1; }
static void action_s3_accept(void) { current_state = S9; }
static void action_s3_fin_ack_vv0(void) { current_state = S7; }
static void action_s3_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s3_syn_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s3_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s3_ack_rst_vv0(void) { current_state = S10; }
static void action_s3_close(void) { current_state = S5; }
static void action_s3_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s3_ack_vv0(void) { current_state = S8; }

// State S4 action handlers
static void action_s4_closeconnection(void) { current_state = S1; }
static void action_s4_ack_psh_vv1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s4_syn_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s4_rst_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s4_accept(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s4_fin_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s4_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s4_syn_vv0(void) { current_state = S9; }
static void action_s4_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s4_ack_rst_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s4_close(void) { current_state = S2; }
static void action_s4_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s4_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }

// State S5 action handlers
static void action_s5_closeconnection(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s5_ack_psh_vv1(void) { current_state = S2; }
static void action_s5_syn_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s5_rst_vv0(void) { current_state = S2; }
static void action_s5_accept(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s5_fin_ack_vv0(void) { current_state = S2; }
static void action_s5_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s5_syn_vv0(void) { current_state = S2; }
static void action_s5_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s5_ack_rst_vv0(void) { current_state = S2; }
static void action_s5_close(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s5_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s5_ack_vv0(void) { current_state = S2; }

// State S6 action handlers
static void action_s6_closeconnection(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s6_ack_psh_vv1(void) { current_state = S1; }
static void action_s6_syn_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s6_rst_vv0(void) { current_state = S1; }
static void action_s6_accept(void) { current_state = S11; }
static void action_s6_fin_ack_vv0(void) { current_state = S1; }
static void action_s6_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s6_syn_vv0(void) { current_state = S3; }
static void action_s6_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s6_ack_rst_vv0(void) { current_state = S1; }
static void action_s6_close(void) { current_state = S5; }
static void action_s6_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s6_ack_vv0(void) { current_state = S1; }

// State S7 action handlers
static void action_s7_closeconnection(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s7_ack_psh_vv1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s7_syn_ack_vv0(void) { current_state = S12; }
static void action_s7_rst_vv0(void) { current_state = S12; }
static void action_s7_accept(void) { current_state = S13; }
static void action_s7_fin_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s7_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s7_syn_vv0(void) { current_state = S12; }
static void action_s7_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s7_ack_rst_vv0(void) { current_state = S12; }
static void action_s7_close(void) { current_state = S2; }
static void action_s7_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s7_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }

// State S8 action handlers
static void action_s8_closeconnection(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s8_ack_psh_vv1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s8_syn_ack_vv0(void) { current_state = S12; }
static void action_s8_rst_vv0(void) { current_state = S12; }
static void action_s8_accept(void) { current_state = S14; }
static void action_s8_fin_ack_vv0(void) { current_state = S7; }
static void action_s8_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s8_syn_vv0(void) { current_state = S12; }
static void action_s8_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s8_ack_rst_vv0(void) { current_state = S12; }
static void action_s8_close(void) { current_state = S2; }
static void action_s8_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s8_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }

// State S9 action handlers
static void action_s9_closeconnection(void) { current_state = S3; }
static void action_s9_ack_psh_vv1(void) { current_state = S14; }
static void action_s9_syn_ack_vv0(void) { current_state = S11; }
static void action_s9_rst_vv0(void) { current_state = S4; }
static void action_s9_accept(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s9_fin_ack_vv0(void) { current_state = S13; }
static void action_s9_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s9_syn_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s9_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s9_ack_rst_vv0(void) { current_state = S15; }
static void action_s9_close(void) { current_state = S5; }
static void action_s9_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s9_ack_vv0(void) { current_state = S14; }

// State S10 action handlers
static void action_s10_closeconnection(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s10_ack_psh_vv1(void) { current_state = S1; }
static void action_s10_syn_ack_vv0(void) { current_state = S1; }
static void action_s10_rst_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s10_accept(void) { current_state = S15; }
static void action_s10_fin_ack_vv0(void) { current_state = S1; }
static void action_s10_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s10_syn_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s10_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s10_ack_rst_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s10_close(void) { current_state = S2; }
static void action_s10_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s10_ack_vv0(void) { current_state = S1; }

// State S11 action handlers
static void action_s11_closeconnection(void) { current_state = S6; }
static void action_s11_ack_psh_vv1(void) { current_state = S4; }
static void action_s11_syn_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s11_rst_vv0(void) { current_state = S4; }
static void action_s11_accept(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s11_fin_ack_vv0(void) { current_state = S4; }
static void action_s11_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s11_syn_vv0(void) { current_state = S9; }
static void action_s11_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s11_ack_rst_vv0(void) { current_state = S4; }
static void action_s11_close(void) { current_state = S5; }
static void action_s11_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s11_ack_vv0(void) { current_state = S4; }

// State S12 action handlers
static void action_s12_closeconnection(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s12_ack_psh_vv1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s12_syn_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s12_rst_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s12_accept(void) { current_state = S1; }
static void action_s12_fin_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s12_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s12_syn_vv0(void) { current_state = S16; }
static void action_s12_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s12_ack_rst_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s12_close(void) { current_state = S2; }
static void action_s12_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s12_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }

// State S13 action handlers
static void action_s13_closeconnection(void) { current_state = S18; }
static void action_s13_ack_psh_vv1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s13_syn_ack_vv0(void) { current_state = S19; }
static void action_s13_rst_vv0(void) { current_state = S19; }
static void action_s13_accept(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s13_fin_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s13_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s13_syn_vv0(void) { current_state = S19; }
static void action_s13_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s13_ack_rst_vv0(void) { current_state = S19; }
static void action_s13_close(void) { current_state = S17; }
static void action_s13_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s13_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }

// State S14 action handlers
static void action_s14_closeconnection(void) { current_state = S21; }
static void action_s14_ack_psh_vv1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s14_syn_ack_vv0(void) { current_state = S19; }
static void action_s14_rst_vv0(void) { current_state = S19; }
static void action_s14_accept(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s14_fin_ack_vv0(void) { current_state = S13; }
static void action_s14_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s14_syn_vv0(void) { current_state = S19; }
static void action_s14_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s14_ack_rst_vv0(void) { current_state = S19; }
static void action_s14_close(void) { current_state = S20; }
static void action_s14_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s14_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }

// State S15 action handlers
static void action_s15_closeconnection(void) { current_state = S10; }
static void action_s15_ack_psh_vv1(void) { current_state = S4; }
static void action_s15_syn_ack_vv0(void) { current_state = S4; }
static void action_s15_rst_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s15_accept(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s15_fin_ack_vv0(void) { current_state = S4; }
static void action_s15_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s15_syn_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s15_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s15_ack_rst_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s15_close(void) { current_state = S2; }
static void action_s15_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s15_ack_vv0(void) { current_state = S4; }

// State S16 action handlers
static void action_s16_closeconnection(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s16_ack_psh_vv1(void) { current_state = S23; }
static void action_s16_syn_ack_vv0(void) { current_state = S22; }
static void action_s16_rst_vv0(void) { current_state = S12; }
static void action_s16_accept(void) { current_state = S3; }
static void action_s16_fin_ack_vv0(void) { current_state = S24; }
static void action_s16_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s16_syn_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s16_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s16_ack_rst_vv0(void) { current_state = S25; }
static void action_s16_close(void) { current_state = S5; }
static void action_s16_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s16_ack_vv0(void) { current_state = S23; }

// State S17 action handlers
static void action_s17_closeconnection(void) { current_state = S26; }
static void action_s17_ack_psh_vv1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s17_syn_ack_vv0(void) { current_state = S2; }
static void action_s17_rst_vv0(void) { current_state = S2; }
static void action_s17_accept(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s17_fin_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s17_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s17_syn_vv0(void) { current_state = S2; }
static void action_s17_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s17_ack_rst_vv0(void) { current_state = S2; }
static void action_s17_close(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s17_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s17_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }

// State S18 action handlers
static void action_s18_closeconnection(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s18_ack_psh_vv1(void) { current_state = S1; }
static void action_s18_syn_ack_vv0(void) { current_state = S1; }
static void action_s18_rst_vv0(void) { current_state = S1; }
static void action_s18_accept(void) { current_state = S27; }
static void action_s18_fin_ack_vv0(void) { current_state = S6; }
static void action_s18_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s18_syn_vv0(void) { current_state = S1; }
static void action_s18_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s18_ack_rst_vv0(void) { current_state = S1; }
static void action_s18_close(void) { current_state = S26; }
static void action_s18_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s18_ack_vv0(void) { current_state = S6; }

// State S19 action handlers
static void action_s19_closeconnection(void) { current_state = S1; }
static void action_s19_ack_psh_vv1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s19_syn_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s19_rst_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s19_accept(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s19_fin_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s19_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s19_syn_vv0(void) { current_state = S28; }
static void action_s19_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s19_ack_rst_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s19_close(void) { current_state = S2; }
static void action_s19_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s19_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }

// State S20 action handlers
static void action_s20_closeconnection(void) { current_state = S29; }
static void action_s20_ack_psh_vv1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s20_syn_ack_vv0(void) { current_state = S2; }
static void action_s20_rst_vv0(void) { current_state = S2; }
static void action_s20_accept(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s20_fin_ack_vv0(void) { current_state = S17; }
static void action_s20_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s20_syn_vv0(void) { current_state = S2; }
static void action_s20_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s20_ack_rst_vv0(void) { current_state = S2; }
static void action_s20_close(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s20_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s20_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }

// State S21 action handlers
static void action_s21_closeconnection(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s21_ack_psh_vv1(void) { current_state = S1; }
static void action_s21_syn_ack_vv0(void) { current_state = S1; }
static void action_s21_rst_vv0(void) { current_state = S1; }
static void action_s21_accept(void) { current_state = S30; }
static void action_s21_fin_ack_vv0(void) { current_state = S31; }
static void action_s21_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s21_syn_vv0(void) { current_state = S1; }
static void action_s21_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s21_ack_rst_vv0(void) { current_state = S1; }
static void action_s21_close(void) { current_state = S29; }
static void action_s21_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s21_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }

// State S22 action handlers
static void action_s22_closeconnection(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s22_ack_psh_vv1(void) { current_state = S12; }
static void action_s22_syn_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s22_rst_vv0(void) { current_state = S12; }
static void action_s22_accept(void) { current_state = S6; }
static void action_s22_fin_ack_vv0(void) { current_state = S12; }
static void action_s22_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s22_syn_vv0(void) { current_state = S16; }
static void action_s22_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s22_ack_rst_vv0(void) { current_state = S12; }
static void action_s22_close(void) { current_state = S5; }
static void action_s22_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s22_ack_vv0(void) { current_state = S12; }

// State S23 action handlers
static void action_s23_closeconnection(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s23_ack_psh_vv1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s23_syn_ack_vv0(void) { current_state = S32; }
static void action_s23_rst_vv0(void) { current_state = S32; }
static void action_s23_accept(void) { current_state = S8; }
static void action_s23_fin_ack_vv0(void) { current_state = S24; }
static void action_s23_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s23_syn_vv0(void) { current_state = S32; }
static void action_s23_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s23_ack_rst_vv0(void) { current_state = S32; }
static void action_s23_close(void) { current_state = S2; }
static void action_s23_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s23_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }

// State S24 action handlers
static void action_s24_closeconnection(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s24_ack_psh_vv1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s24_syn_ack_vv0(void) { current_state = S32; }
static void action_s24_rst_vv0(void) { current_state = S32; }
static void action_s24_accept(void) { current_state = S7; }
static void action_s24_fin_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s24_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s24_syn_vv0(void) { current_state = S32; }
static void action_s24_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s24_ack_rst_vv0(void) { current_state = S32; }
static void action_s24_close(void) { current_state = S2; }
static void action_s24_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s24_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }

// State S25 action handlers
static void action_s25_closeconnection(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s25_ack_psh_vv1(void) { current_state = S12; }
static void action_s25_syn_ack_vv0(void) { current_state = S12; }
static void action_s25_rst_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s25_accept(void) { current_state = S10; }
static void action_s25_fin_ack_vv0(void) { current_state = S12; }
static void action_s25_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s25_syn_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s25_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s25_ack_rst_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s25_close(void) { current_state = S2; }
static void action_s25_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s25_ack_vv0(void) { current_state = S12; }

// State S26 action handlers
static void action_s26_closeconnection(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s26_ack_psh_vv1(void) { current_state = S2; }
static void action_s26_syn_ack_vv0(void) { current_state = S2; }
static void action_s26_rst_vv0(void) { current_state = S2; }
static void action_s26_accept(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s26_fin_ack_vv0(void) { current_state = S5; }
static void action_s26_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s26_syn_vv0(void) { current_state = S2; }
static void action_s26_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s26_ack_rst_vv0(void) { current_state = S2; }
static void action_s26_close(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s26_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s26_ack_vv0(void) { current_state = S5; }

// State S27 action handlers
static void action_s27_closeconnection(void) { current_state = S18; }
static void action_s27_ack_psh_vv1(void) { current_state = S4; }
static void action_s27_syn_ack_vv0(void) { current_state = S4; }
static void action_s27_rst_vv0(void) { current_state = S4; }
static void action_s27_accept(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s27_fin_ack_vv0(void) { current_state = S11; }
static void action_s27_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s27_syn_vv0(void) { current_state = S4; }
static void action_s27_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s27_ack_rst_vv0(void) { current_state = S4; }
static void action_s27_close(void) { current_state = S26; }
static void action_s27_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s27_ack_vv0(void) { current_state = S11; }

// State S28 action handlers
static void action_s28_closeconnection(void) { current_state = S3; }
static void action_s28_ack_psh_vv1(void) { current_state = S34; }
static void action_s28_syn_ack_vv0(void) { current_state = S36; }
static void action_s28_rst_vv0(void) { current_state = S19; }
static void action_s28_accept(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s28_fin_ack_vv0(void) { current_state = S33; }
static void action_s28_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s28_syn_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s28_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s28_ack_rst_vv0(void) { current_state = S35; }
static void action_s28_close(void) { current_state = S5; }
static void action_s28_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s28_ack_vv0(void) { current_state = S34; }

// State S29 action handlers
static void action_s29_closeconnection(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s29_ack_psh_vv1(void) { current_state = S2; }
static void action_s29_syn_ack_vv0(void) { current_state = S2; }
static void action_s29_rst_vv0(void) { current_state = S2; }
static void action_s29_accept(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s29_fin_ack_vv0(void) { current_state = S37; }
static void action_s29_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s29_syn_vv0(void) { current_state = S2; }
static void action_s29_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s29_ack_rst_vv0(void) { current_state = S2; }
static void action_s29_close(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s29_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s29_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }

// State S30 action handlers
static void action_s30_closeconnection(void) { current_state = S21; }
static void action_s30_ack_psh_vv1(void) { current_state = S4; }
static void action_s30_syn_ack_vv0(void) { current_state = S4; }
static void action_s30_rst_vv0(void) { current_state = S4; }
static void action_s30_accept(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s30_fin_ack_vv0(void) { current_state = S38; }
static void action_s30_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s30_syn_vv0(void) { current_state = S4; }
static void action_s30_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s30_ack_rst_vv0(void) { current_state = S4; }
static void action_s30_close(void) { current_state = S29; }
static void action_s30_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s30_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }

// State S31 action handlers
static void action_s31_closeconnection(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s31_ack_psh_vv1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s31_syn_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s31_rst_vv0(void) { current_state = S39; }
static void action_s31_accept(void) { current_state = S38; }
static void action_s31_fin_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s31_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s31_syn_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s31_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s31_ack_rst_vv0(void) { current_state = S39; }
static void action_s31_close(void) { current_state = S37; }
static void action_s31_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s31_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }

// State S32 action handlers
static void action_s32_closeconnection(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s32_ack_psh_vv1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s32_syn_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s32_rst_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s32_accept(void) { current_state = S12; }
static void action_s32_fin_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s32_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s32_syn_vv0(void) { current_state = S40; }
static void action_s32_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s32_ack_rst_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s32_close(void) { current_state = S2; }
static void action_s32_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s32_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }

// State S33 action handlers
static void action_s33_closeconnection(void) { current_state = S7; }
static void action_s33_ack_psh_vv1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s33_syn_ack_vv0(void) { current_state = S41; }
static void action_s33_rst_vv0(void) { current_state = S41; }
static void action_s33_accept(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s33_fin_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s33_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s33_syn_vv0(void) { current_state = S41; }
static void action_s33_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s33_ack_rst_vv0(void) { current_state = S41; }
static void action_s33_close(void) { current_state = S2; }
static void action_s33_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s33_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }

// State S34 action handlers
static void action_s34_closeconnection(void) { current_state = S8; }
static void action_s34_ack_psh_vv1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s34_syn_ack_vv0(void) { current_state = S41; }
static void action_s34_rst_vv0(void) { current_state = S41; }
static void action_s34_accept(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s34_fin_ack_vv0(void) { current_state = S33; }
static void action_s34_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s34_syn_vv0(void) { current_state = S41; }
static void action_s34_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s34_ack_rst_vv0(void) { current_state = S41; }
static void action_s34_close(void) { current_state = S2; }
static void action_s34_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s34_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }

// State S35 action handlers
static void action_s35_closeconnection(void) { current_state = S10; }
static void action_s35_ack_psh_vv1(void) { current_state = S19; }
static void action_s35_syn_ack_vv0(void) { current_state = S19; }
static void action_s35_rst_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s35_accept(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s35_fin_ack_vv0(void) { current_state = S19; }
static void action_s35_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s35_syn_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s35_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s35_ack_rst_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s35_close(void) { current_state = S2; }
static void action_s35_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s35_ack_vv0(void) { current_state = S19; }

// State S36 action handlers
static void action_s36_closeconnection(void) { current_state = S6; }
static void action_s36_ack_psh_vv1(void) { current_state = S19; }
static void action_s36_syn_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s36_rst_vv0(void) { current_state = S19; }
static void action_s36_accept(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s36_fin_ack_vv0(void) { current_state = S19; }
static void action_s36_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s36_syn_vv0(void) { current_state = S28; }
static void action_s36_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s36_ack_rst_vv0(void) { current_state = S19; }
static void action_s36_close(void) { current_state = S5; }
static void action_s36_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s36_ack_vv0(void) { current_state = S19; }

// State S37 action handlers
static void action_s37_closeconnection(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s37_ack_psh_vv1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s37_syn_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s37_rst_vv0(void) { current_state = S42; }
static void action_s37_accept(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s37_fin_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s37_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s37_syn_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s37_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s37_ack_rst_vv0(void) { current_state = S42; }
static void action_s37_close(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s37_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s37_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }

// State S38 action handlers
static void action_s38_closeconnection(void) { current_state = S31; }
static void action_s38_ack_psh_vv1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s38_syn_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s38_rst_vv0(void) { current_state = S43; }
static void action_s38_accept(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s38_fin_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s38_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s38_syn_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s38_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s38_ack_rst_vv0(void) { current_state = S43; }
static void action_s38_close(void) { current_state = S37; }
static void action_s38_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s38_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }

// State S39 action handlers
static void action_s39_closeconnection(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s39_ack_psh_vv1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s39_syn_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s39_rst_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s39_accept(void) { current_state = S43; }
static void action_s39_fin_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s39_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s39_syn_vv0(void) { current_state = S3; }
static void action_s39_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s39_ack_rst_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s39_close(void) { current_state = S42; }
static void action_s39_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s39_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }

// State S40 action handlers
static void action_s40_closeconnection(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s40_ack_psh_vv1(void) { current_state = S32; }
static void action_s40_syn_ack_vv0(void) { current_state = S44; }
static void action_s40_rst_vv0(void) { current_state = S32; }
static void action_s40_accept(void) { current_state = S16; }
static void action_s40_fin_ack_vv0(void) { current_state = S32; }
static void action_s40_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s40_syn_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s40_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s40_ack_rst_vv0(void) { current_state = S45; }
static void action_s40_close(void) { current_state = S5; }
static void action_s40_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s40_ack_vv0(void) { current_state = S32; }

// State S41 action handlers
static void action_s41_closeconnection(void) { current_state = S12; }
static void action_s41_ack_psh_vv1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s41_syn_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s41_rst_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s41_accept(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s41_fin_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s41_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s41_syn_vv0(void) { current_state = S46; }
static void action_s41_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s41_ack_rst_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s41_close(void) { current_state = S2; }
static void action_s41_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s41_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }

// State S42 action handlers
static void action_s42_closeconnection(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s42_ack_psh_vv1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s42_syn_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s42_rst_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s42_accept(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s42_fin_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s42_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s42_syn_vv0(void) { current_state = S2; }
static void action_s42_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s42_ack_rst_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s42_close(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s42_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s42_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }

// State S43 action handlers
static void action_s43_closeconnection(void) { current_state = S39; }
static void action_s43_ack_psh_vv1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s43_syn_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s43_rst_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s43_accept(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s43_fin_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s43_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s43_syn_vv0(void) { current_state = S9; }
static void action_s43_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s43_ack_rst_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s43_close(void) { current_state = S42; }
static void action_s43_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s43_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }

// State S44 action handlers
static void action_s44_closeconnection(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s44_ack_psh_vv1(void) { current_state = S32; }
static void action_s44_syn_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s44_rst_vv0(void) { current_state = S32; }
static void action_s44_accept(void) { current_state = S22; }
static void action_s44_fin_ack_vv0(void) { current_state = S32; }
static void action_s44_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s44_syn_vv0(void) { current_state = S40; }
static void action_s44_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s44_ack_rst_vv0(void) { current_state = S32; }
static void action_s44_close(void) { current_state = S5; }
static void action_s44_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s44_ack_vv0(void) { current_state = S32; }

// State S45 action handlers
static void action_s45_closeconnection(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s45_ack_psh_vv1(void) { current_state = S32; }
static void action_s45_syn_ack_vv0(void) { current_state = S32; }
static void action_s45_rst_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s45_accept(void) { current_state = S25; }
static void action_s45_fin_ack_vv0(void) { current_state = S32; }
static void action_s45_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s45_syn_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s45_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s45_ack_rst_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s45_close(void) { current_state = S2; }
static void action_s45_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s45_ack_vv0(void) { current_state = S32; }

// State S46 action handlers
static void action_s46_closeconnection(void) { current_state = S16; }
static void action_s46_ack_psh_vv1(void) { current_state = S48; }
static void action_s46_syn_ack_vv0(void) { current_state = S49; }
static void action_s46_rst_vv0(void) { current_state = S41; }
static void action_s46_accept(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s46_fin_ack_vv0(void) { current_state = S50; }
static void action_s46_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s46_syn_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s46_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s46_ack_rst_vv0(void) { current_state = S47; }
static void action_s46_close(void) { current_state = S5; }
static void action_s46_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s46_ack_vv0(void) { current_state = S48; }

// State S47 action handlers
static void action_s47_closeconnection(void) { current_state = S25; }
static void action_s47_ack_psh_vv1(void) { current_state = S41; }
static void action_s47_syn_ack_vv0(void) { current_state = S41; }
static void action_s47_rst_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s47_accept(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s47_fin_ack_vv0(void) { current_state = S41; }
static void action_s47_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s47_syn_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s47_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s47_ack_rst_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s47_close(void) { current_state = S2; }
static void action_s47_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s47_ack_vv0(void) { current_state = S41; }

// State S48 action handlers
static void action_s48_closeconnection(void) { current_state = S23; }
static void action_s48_ack_psh_vv1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s48_syn_ack_vv0(void) { current_state = S51; }
static void action_s48_rst_vv0(void) { current_state = S51; }
static void action_s48_accept(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s48_fin_ack_vv0(void) { current_state = S50; }
static void action_s48_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s48_syn_vv0(void) { current_state = S51; }
static void action_s48_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s48_ack_rst_vv0(void) { current_state = S51; }
static void action_s48_close(void) { current_state = S2; }
static void action_s48_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s48_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }

// State S49 action handlers
static void action_s49_closeconnection(void) { current_state = S22; }
static void action_s49_ack_psh_vv1(void) { current_state = S41; }
static void action_s49_syn_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s49_rst_vv0(void) { current_state = S41; }
static void action_s49_accept(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s49_fin_ack_vv0(void) { current_state = S41; }
static void action_s49_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s49_syn_vv0(void) { current_state = S46; }
static void action_s49_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s49_ack_rst_vv0(void) { current_state = S41; }
static void action_s49_close(void) { current_state = S5; }
static void action_s49_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s49_ack_vv0(void) { current_state = S41; }

// State S50 action handlers
static void action_s50_closeconnection(void) { current_state = S24; }
static void action_s50_ack_psh_vv1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s50_syn_ack_vv0(void) { current_state = S51; }
static void action_s50_rst_vv0(void) { current_state = S51; }
static void action_s50_accept(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s50_fin_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s50_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s50_syn_vv0(void) { current_state = S51; }
static void action_s50_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s50_ack_rst_vv0(void) { current_state = S51; }
static void action_s50_close(void) { current_state = S2; }
static void action_s50_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s50_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }

// State S51 action handlers
static void action_s51_closeconnection(void) { current_state = S32; }
static void action_s51_ack_psh_vv1(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s51_syn_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s51_rst_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s51_accept(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s51_fin_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s51_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s51_syn_vv0(void) { current_state = S52; }
static void action_s51_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s51_ack_rst_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s51_close(void) { current_state = S2; }
static void action_s51_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s51_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }

// State S52 action handlers
static void action_s52_closeconnection(void) { current_state = S40; }
static void action_s52_ack_psh_vv1(void) { current_state = S51; }
static void action_s52_syn_ack_vv0(void) { current_state = S53; }
static void action_s52_rst_vv0(void) { current_state = S51; }
static void action_s52_accept(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s52_fin_ack_vv0(void) { current_state = S51; }
static void action_s52_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s52_syn_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s52_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s52_ack_rst_vv0(void) { current_state = S54; }
static void action_s52_close(void) { current_state = S5; }
static void action_s52_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s52_ack_vv0(void) { current_state = S51; }

// State S53 action handlers
static void action_s53_closeconnection(void) { current_state = S44; }
static void action_s53_ack_psh_vv1(void) { current_state = S51; }
static void action_s53_syn_ack_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s53_rst_vv0(void) { current_state = S51; }
static void action_s53_accept(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s53_fin_ack_vv0(void) { current_state = S51; }
static void action_s53_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s53_syn_vv0(void) { current_state = S52; }
static void action_s53_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s53_ack_rst_vv0(void) { current_state = S51; }
static void action_s53_close(void) { current_state = S5; }
static void action_s53_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s53_ack_vv0(void) { current_state = S51; }

// State S54 action handlers
static void action_s54_closeconnection(void) { current_state = S45; }
static void action_s54_ack_psh_vv1(void) { current_state = S51; }
static void action_s54_syn_ack_vv0(void) { current_state = S51; }
static void action_s54_rst_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s54_accept(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s54_fin_ack_vv0(void) { current_state = S51; }
static void action_s54_listen(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s54_syn_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s54_rcv(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s54_ack_rst_vv0(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s54_close(void) { current_state = S2; }
static void action_s54_send(void) { /* ostaje u istom stanju, nema sta da se radi */ }
static void action_s54_ack_vv0(void) { current_state = S51; }

typedef void (*ActionProcedure)(void);

// 2D niz pokazivaca na void procedure - svaka celija se poziva radi sporednog
// efekta (upisa current_state), ne zbog povratne vrednosti.
static const ActionProcedure state_table[NUM_STATES][NUM_EVENTS] PROGMEM = {
    { action_s0_closeconnection, action_s0_ack_psh_vv1, action_s0_syn_ack_vv0, action_s0_rst_vv0, action_s0_accept, action_s0_fin_ack_vv0, action_s0_listen, action_s0_syn_vv0, action_s0_rcv, action_s0_ack_rst_vv0, action_s0_close, action_s0_send, action_s0_ack_vv0 }, // S0
    { action_s1_closeconnection, action_s1_ack_psh_vv1, action_s1_syn_ack_vv0, action_s1_rst_vv0, action_s1_accept, action_s1_fin_ack_vv0, action_s1_listen, action_s1_syn_vv0, action_s1_rcv, action_s1_ack_rst_vv0, action_s1_close, action_s1_send, action_s1_ack_vv0 }, // S1
    { action_s2_closeconnection, action_s2_ack_psh_vv1, action_s2_syn_ack_vv0, action_s2_rst_vv0, action_s2_accept, action_s2_fin_ack_vv0, action_s2_listen, action_s2_syn_vv0, action_s2_rcv, action_s2_ack_rst_vv0, action_s2_close, action_s2_send, action_s2_ack_vv0 }, // S2
    { action_s3_closeconnection, action_s3_ack_psh_vv1, action_s3_syn_ack_vv0, action_s3_rst_vv0, action_s3_accept, action_s3_fin_ack_vv0, action_s3_listen, action_s3_syn_vv0, action_s3_rcv, action_s3_ack_rst_vv0, action_s3_close, action_s3_send, action_s3_ack_vv0 }, // S3
    { action_s4_closeconnection, action_s4_ack_psh_vv1, action_s4_syn_ack_vv0, action_s4_rst_vv0, action_s4_accept, action_s4_fin_ack_vv0, action_s4_listen, action_s4_syn_vv0, action_s4_rcv, action_s4_ack_rst_vv0, action_s4_close, action_s4_send, action_s4_ack_vv0 }, // S4
    { action_s5_closeconnection, action_s5_ack_psh_vv1, action_s5_syn_ack_vv0, action_s5_rst_vv0, action_s5_accept, action_s5_fin_ack_vv0, action_s5_listen, action_s5_syn_vv0, action_s5_rcv, action_s5_ack_rst_vv0, action_s5_close, action_s5_send, action_s5_ack_vv0 }, // S5
    { action_s6_closeconnection, action_s6_ack_psh_vv1, action_s6_syn_ack_vv0, action_s6_rst_vv0, action_s6_accept, action_s6_fin_ack_vv0, action_s6_listen, action_s6_syn_vv0, action_s6_rcv, action_s6_ack_rst_vv0, action_s6_close, action_s6_send, action_s6_ack_vv0 }, // S6
    { action_s7_closeconnection, action_s7_ack_psh_vv1, action_s7_syn_ack_vv0, action_s7_rst_vv0, action_s7_accept, action_s7_fin_ack_vv0, action_s7_listen, action_s7_syn_vv0, action_s7_rcv, action_s7_ack_rst_vv0, action_s7_close, action_s7_send, action_s7_ack_vv0 }, // S7
    { action_s8_closeconnection, action_s8_ack_psh_vv1, action_s8_syn_ack_vv0, action_s8_rst_vv0, action_s8_accept, action_s8_fin_ack_vv0, action_s8_listen, action_s8_syn_vv0, action_s8_rcv, action_s8_ack_rst_vv0, action_s8_close, action_s8_send, action_s8_ack_vv0 }, // S8
    { action_s9_closeconnection, action_s9_ack_psh_vv1, action_s9_syn_ack_vv0, action_s9_rst_vv0, action_s9_accept, action_s9_fin_ack_vv0, action_s9_listen, action_s9_syn_vv0, action_s9_rcv, action_s9_ack_rst_vv0, action_s9_close, action_s9_send, action_s9_ack_vv0 }, // S9
    { action_s10_closeconnection, action_s10_ack_psh_vv1, action_s10_syn_ack_vv0, action_s10_rst_vv0, action_s10_accept, action_s10_fin_ack_vv0, action_s10_listen, action_s10_syn_vv0, action_s10_rcv, action_s10_ack_rst_vv0, action_s10_close, action_s10_send, action_s10_ack_vv0 }, // S10
    { action_s11_closeconnection, action_s11_ack_psh_vv1, action_s11_syn_ack_vv0, action_s11_rst_vv0, action_s11_accept, action_s11_fin_ack_vv0, action_s11_listen, action_s11_syn_vv0, action_s11_rcv, action_s11_ack_rst_vv0, action_s11_close, action_s11_send, action_s11_ack_vv0 }, // S11
    { action_s12_closeconnection, action_s12_ack_psh_vv1, action_s12_syn_ack_vv0, action_s12_rst_vv0, action_s12_accept, action_s12_fin_ack_vv0, action_s12_listen, action_s12_syn_vv0, action_s12_rcv, action_s12_ack_rst_vv0, action_s12_close, action_s12_send, action_s12_ack_vv0 }, // S12
    { action_s13_closeconnection, action_s13_ack_psh_vv1, action_s13_syn_ack_vv0, action_s13_rst_vv0, action_s13_accept, action_s13_fin_ack_vv0, action_s13_listen, action_s13_syn_vv0, action_s13_rcv, action_s13_ack_rst_vv0, action_s13_close, action_s13_send, action_s13_ack_vv0 }, // S13
    { action_s14_closeconnection, action_s14_ack_psh_vv1, action_s14_syn_ack_vv0, action_s14_rst_vv0, action_s14_accept, action_s14_fin_ack_vv0, action_s14_listen, action_s14_syn_vv0, action_s14_rcv, action_s14_ack_rst_vv0, action_s14_close, action_s14_send, action_s14_ack_vv0 }, // S14
    { action_s15_closeconnection, action_s15_ack_psh_vv1, action_s15_syn_ack_vv0, action_s15_rst_vv0, action_s15_accept, action_s15_fin_ack_vv0, action_s15_listen, action_s15_syn_vv0, action_s15_rcv, action_s15_ack_rst_vv0, action_s15_close, action_s15_send, action_s15_ack_vv0 }, // S15
    { action_s16_closeconnection, action_s16_ack_psh_vv1, action_s16_syn_ack_vv0, action_s16_rst_vv0, action_s16_accept, action_s16_fin_ack_vv0, action_s16_listen, action_s16_syn_vv0, action_s16_rcv, action_s16_ack_rst_vv0, action_s16_close, action_s16_send, action_s16_ack_vv0 }, // S16
    { action_s17_closeconnection, action_s17_ack_psh_vv1, action_s17_syn_ack_vv0, action_s17_rst_vv0, action_s17_accept, action_s17_fin_ack_vv0, action_s17_listen, action_s17_syn_vv0, action_s17_rcv, action_s17_ack_rst_vv0, action_s17_close, action_s17_send, action_s17_ack_vv0 }, // S17
    { action_s18_closeconnection, action_s18_ack_psh_vv1, action_s18_syn_ack_vv0, action_s18_rst_vv0, action_s18_accept, action_s18_fin_ack_vv0, action_s18_listen, action_s18_syn_vv0, action_s18_rcv, action_s18_ack_rst_vv0, action_s18_close, action_s18_send, action_s18_ack_vv0 }, // S18
    { action_s19_closeconnection, action_s19_ack_psh_vv1, action_s19_syn_ack_vv0, action_s19_rst_vv0, action_s19_accept, action_s19_fin_ack_vv0, action_s19_listen, action_s19_syn_vv0, action_s19_rcv, action_s19_ack_rst_vv0, action_s19_close, action_s19_send, action_s19_ack_vv0 }, // S19
    { action_s20_closeconnection, action_s20_ack_psh_vv1, action_s20_syn_ack_vv0, action_s20_rst_vv0, action_s20_accept, action_s20_fin_ack_vv0, action_s20_listen, action_s20_syn_vv0, action_s20_rcv, action_s20_ack_rst_vv0, action_s20_close, action_s20_send, action_s20_ack_vv0 }, // S20
    { action_s21_closeconnection, action_s21_ack_psh_vv1, action_s21_syn_ack_vv0, action_s21_rst_vv0, action_s21_accept, action_s21_fin_ack_vv0, action_s21_listen, action_s21_syn_vv0, action_s21_rcv, action_s21_ack_rst_vv0, action_s21_close, action_s21_send, action_s21_ack_vv0 }, // S21
    { action_s22_closeconnection, action_s22_ack_psh_vv1, action_s22_syn_ack_vv0, action_s22_rst_vv0, action_s22_accept, action_s22_fin_ack_vv0, action_s22_listen, action_s22_syn_vv0, action_s22_rcv, action_s22_ack_rst_vv0, action_s22_close, action_s22_send, action_s22_ack_vv0 }, // S22
    { action_s23_closeconnection, action_s23_ack_psh_vv1, action_s23_syn_ack_vv0, action_s23_rst_vv0, action_s23_accept, action_s23_fin_ack_vv0, action_s23_listen, action_s23_syn_vv0, action_s23_rcv, action_s23_ack_rst_vv0, action_s23_close, action_s23_send, action_s23_ack_vv0 }, // S23
    { action_s24_closeconnection, action_s24_ack_psh_vv1, action_s24_syn_ack_vv0, action_s24_rst_vv0, action_s24_accept, action_s24_fin_ack_vv0, action_s24_listen, action_s24_syn_vv0, action_s24_rcv, action_s24_ack_rst_vv0, action_s24_close, action_s24_send, action_s24_ack_vv0 }, // S24
    { action_s25_closeconnection, action_s25_ack_psh_vv1, action_s25_syn_ack_vv0, action_s25_rst_vv0, action_s25_accept, action_s25_fin_ack_vv0, action_s25_listen, action_s25_syn_vv0, action_s25_rcv, action_s25_ack_rst_vv0, action_s25_close, action_s25_send, action_s25_ack_vv0 }, // S25
    { action_s26_closeconnection, action_s26_ack_psh_vv1, action_s26_syn_ack_vv0, action_s26_rst_vv0, action_s26_accept, action_s26_fin_ack_vv0, action_s26_listen, action_s26_syn_vv0, action_s26_rcv, action_s26_ack_rst_vv0, action_s26_close, action_s26_send, action_s26_ack_vv0 }, // S26
    { action_s27_closeconnection, action_s27_ack_psh_vv1, action_s27_syn_ack_vv0, action_s27_rst_vv0, action_s27_accept, action_s27_fin_ack_vv0, action_s27_listen, action_s27_syn_vv0, action_s27_rcv, action_s27_ack_rst_vv0, action_s27_close, action_s27_send, action_s27_ack_vv0 }, // S27
    { action_s28_closeconnection, action_s28_ack_psh_vv1, action_s28_syn_ack_vv0, action_s28_rst_vv0, action_s28_accept, action_s28_fin_ack_vv0, action_s28_listen, action_s28_syn_vv0, action_s28_rcv, action_s28_ack_rst_vv0, action_s28_close, action_s28_send, action_s28_ack_vv0 }, // S28
    { action_s29_closeconnection, action_s29_ack_psh_vv1, action_s29_syn_ack_vv0, action_s29_rst_vv0, action_s29_accept, action_s29_fin_ack_vv0, action_s29_listen, action_s29_syn_vv0, action_s29_rcv, action_s29_ack_rst_vv0, action_s29_close, action_s29_send, action_s29_ack_vv0 }, // S29
    { action_s30_closeconnection, action_s30_ack_psh_vv1, action_s30_syn_ack_vv0, action_s30_rst_vv0, action_s30_accept, action_s30_fin_ack_vv0, action_s30_listen, action_s30_syn_vv0, action_s30_rcv, action_s30_ack_rst_vv0, action_s30_close, action_s30_send, action_s30_ack_vv0 }, // S30
    { action_s31_closeconnection, action_s31_ack_psh_vv1, action_s31_syn_ack_vv0, action_s31_rst_vv0, action_s31_accept, action_s31_fin_ack_vv0, action_s31_listen, action_s31_syn_vv0, action_s31_rcv, action_s31_ack_rst_vv0, action_s31_close, action_s31_send, action_s31_ack_vv0 }, // S31
    { action_s32_closeconnection, action_s32_ack_psh_vv1, action_s32_syn_ack_vv0, action_s32_rst_vv0, action_s32_accept, action_s32_fin_ack_vv0, action_s32_listen, action_s32_syn_vv0, action_s32_rcv, action_s32_ack_rst_vv0, action_s32_close, action_s32_send, action_s32_ack_vv0 }, // S32
    { action_s33_closeconnection, action_s33_ack_psh_vv1, action_s33_syn_ack_vv0, action_s33_rst_vv0, action_s33_accept, action_s33_fin_ack_vv0, action_s33_listen, action_s33_syn_vv0, action_s33_rcv, action_s33_ack_rst_vv0, action_s33_close, action_s33_send, action_s33_ack_vv0 }, // S33
    { action_s34_closeconnection, action_s34_ack_psh_vv1, action_s34_syn_ack_vv0, action_s34_rst_vv0, action_s34_accept, action_s34_fin_ack_vv0, action_s34_listen, action_s34_syn_vv0, action_s34_rcv, action_s34_ack_rst_vv0, action_s34_close, action_s34_send, action_s34_ack_vv0 }, // S34
    { action_s35_closeconnection, action_s35_ack_psh_vv1, action_s35_syn_ack_vv0, action_s35_rst_vv0, action_s35_accept, action_s35_fin_ack_vv0, action_s35_listen, action_s35_syn_vv0, action_s35_rcv, action_s35_ack_rst_vv0, action_s35_close, action_s35_send, action_s35_ack_vv0 }, // S35
    { action_s36_closeconnection, action_s36_ack_psh_vv1, action_s36_syn_ack_vv0, action_s36_rst_vv0, action_s36_accept, action_s36_fin_ack_vv0, action_s36_listen, action_s36_syn_vv0, action_s36_rcv, action_s36_ack_rst_vv0, action_s36_close, action_s36_send, action_s36_ack_vv0 }, // S36
    { action_s37_closeconnection, action_s37_ack_psh_vv1, action_s37_syn_ack_vv0, action_s37_rst_vv0, action_s37_accept, action_s37_fin_ack_vv0, action_s37_listen, action_s37_syn_vv0, action_s37_rcv, action_s37_ack_rst_vv0, action_s37_close, action_s37_send, action_s37_ack_vv0 }, // S37
    { action_s38_closeconnection, action_s38_ack_psh_vv1, action_s38_syn_ack_vv0, action_s38_rst_vv0, action_s38_accept, action_s38_fin_ack_vv0, action_s38_listen, action_s38_syn_vv0, action_s38_rcv, action_s38_ack_rst_vv0, action_s38_close, action_s38_send, action_s38_ack_vv0 }, // S38
    { action_s39_closeconnection, action_s39_ack_psh_vv1, action_s39_syn_ack_vv0, action_s39_rst_vv0, action_s39_accept, action_s39_fin_ack_vv0, action_s39_listen, action_s39_syn_vv0, action_s39_rcv, action_s39_ack_rst_vv0, action_s39_close, action_s39_send, action_s39_ack_vv0 }, // S39
    { action_s40_closeconnection, action_s40_ack_psh_vv1, action_s40_syn_ack_vv0, action_s40_rst_vv0, action_s40_accept, action_s40_fin_ack_vv0, action_s40_listen, action_s40_syn_vv0, action_s40_rcv, action_s40_ack_rst_vv0, action_s40_close, action_s40_send, action_s40_ack_vv0 }, // S40
    { action_s41_closeconnection, action_s41_ack_psh_vv1, action_s41_syn_ack_vv0, action_s41_rst_vv0, action_s41_accept, action_s41_fin_ack_vv0, action_s41_listen, action_s41_syn_vv0, action_s41_rcv, action_s41_ack_rst_vv0, action_s41_close, action_s41_send, action_s41_ack_vv0 }, // S41
    { action_s42_closeconnection, action_s42_ack_psh_vv1, action_s42_syn_ack_vv0, action_s42_rst_vv0, action_s42_accept, action_s42_fin_ack_vv0, action_s42_listen, action_s42_syn_vv0, action_s42_rcv, action_s42_ack_rst_vv0, action_s42_close, action_s42_send, action_s42_ack_vv0 }, // S42
    { action_s43_closeconnection, action_s43_ack_psh_vv1, action_s43_syn_ack_vv0, action_s43_rst_vv0, action_s43_accept, action_s43_fin_ack_vv0, action_s43_listen, action_s43_syn_vv0, action_s43_rcv, action_s43_ack_rst_vv0, action_s43_close, action_s43_send, action_s43_ack_vv0 }, // S43
    { action_s44_closeconnection, action_s44_ack_psh_vv1, action_s44_syn_ack_vv0, action_s44_rst_vv0, action_s44_accept, action_s44_fin_ack_vv0, action_s44_listen, action_s44_syn_vv0, action_s44_rcv, action_s44_ack_rst_vv0, action_s44_close, action_s44_send, action_s44_ack_vv0 }, // S44
    { action_s45_closeconnection, action_s45_ack_psh_vv1, action_s45_syn_ack_vv0, action_s45_rst_vv0, action_s45_accept, action_s45_fin_ack_vv0, action_s45_listen, action_s45_syn_vv0, action_s45_rcv, action_s45_ack_rst_vv0, action_s45_close, action_s45_send, action_s45_ack_vv0 }, // S45
    { action_s46_closeconnection, action_s46_ack_psh_vv1, action_s46_syn_ack_vv0, action_s46_rst_vv0, action_s46_accept, action_s46_fin_ack_vv0, action_s46_listen, action_s46_syn_vv0, action_s46_rcv, action_s46_ack_rst_vv0, action_s46_close, action_s46_send, action_s46_ack_vv0 }, // S46
    { action_s47_closeconnection, action_s47_ack_psh_vv1, action_s47_syn_ack_vv0, action_s47_rst_vv0, action_s47_accept, action_s47_fin_ack_vv0, action_s47_listen, action_s47_syn_vv0, action_s47_rcv, action_s47_ack_rst_vv0, action_s47_close, action_s47_send, action_s47_ack_vv0 }, // S47
    { action_s48_closeconnection, action_s48_ack_psh_vv1, action_s48_syn_ack_vv0, action_s48_rst_vv0, action_s48_accept, action_s48_fin_ack_vv0, action_s48_listen, action_s48_syn_vv0, action_s48_rcv, action_s48_ack_rst_vv0, action_s48_close, action_s48_send, action_s48_ack_vv0 }, // S48
    { action_s49_closeconnection, action_s49_ack_psh_vv1, action_s49_syn_ack_vv0, action_s49_rst_vv0, action_s49_accept, action_s49_fin_ack_vv0, action_s49_listen, action_s49_syn_vv0, action_s49_rcv, action_s49_ack_rst_vv0, action_s49_close, action_s49_send, action_s49_ack_vv0 }, // S49
    { action_s50_closeconnection, action_s50_ack_psh_vv1, action_s50_syn_ack_vv0, action_s50_rst_vv0, action_s50_accept, action_s50_fin_ack_vv0, action_s50_listen, action_s50_syn_vv0, action_s50_rcv, action_s50_ack_rst_vv0, action_s50_close, action_s50_send, action_s50_ack_vv0 }, // S50
    { action_s51_closeconnection, action_s51_ack_psh_vv1, action_s51_syn_ack_vv0, action_s51_rst_vv0, action_s51_accept, action_s51_fin_ack_vv0, action_s51_listen, action_s51_syn_vv0, action_s51_rcv, action_s51_ack_rst_vv0, action_s51_close, action_s51_send, action_s51_ack_vv0 }, // S51
    { action_s52_closeconnection, action_s52_ack_psh_vv1, action_s52_syn_ack_vv0, action_s52_rst_vv0, action_s52_accept, action_s52_fin_ack_vv0, action_s52_listen, action_s52_syn_vv0, action_s52_rcv, action_s52_ack_rst_vv0, action_s52_close, action_s52_send, action_s52_ack_vv0 }, // S52
    { action_s53_closeconnection, action_s53_ack_psh_vv1, action_s53_syn_ack_vv0, action_s53_rst_vv0, action_s53_accept, action_s53_fin_ack_vv0, action_s53_listen, action_s53_syn_vv0, action_s53_rcv, action_s53_ack_rst_vv0, action_s53_close, action_s53_send, action_s53_ack_vv0 }, // S53
    { action_s54_closeconnection, action_s54_ack_psh_vv1, action_s54_syn_ack_vv0, action_s54_rst_vv0, action_s54_accept, action_s54_fin_ack_vv0, action_s54_listen, action_s54_syn_vv0, action_s54_rcv, action_s54_ack_rst_vv0, action_s54_close, action_s54_send, action_s54_ack_vv0 }  // S54
};

// Za razliku od ostalih implementacija, ovde nema fsm_transition() koja vraca
// next_state - tranzicija je sporedni efekat poziva iz tabele. Eksplicitna
// provera granica pre poziva odrazava Santicevo upozorenje da tabela
// pretrage, za razliku od switch-a, nema default granu.
static uint16_t measured_transition(uint8_t event) {
    cli();
    cycles_start();
    ActionProcedure t_handler;
    if ((event < NUM_EVENTS) && (current_state < NUM_STATES)) {
        t_handler = (ActionProcedure) pgm_read_word(&state_table[current_state][event]);
        t_handler(); // poziv radi sporednog efekta na current_state
    }
    uint16_t cycles = cycles_stop();
    sei();
    return cycles;
}

static void run_benchmark(void)
{
    const uint16_t N = 1000;
    uint16_t min_c = 0xFFFF, max_c = 0;
    uint32_t sum_c = 0;

    // Unapred generisan niz dogadjaja, POTPUNO van merenog prozora - sprecava
    // da -flto premesti rand()%NUM_EVENTS deljenje unutar cli()/sei() bloka.
    static uint8_t precomputed_events[N];
    for (uint16_t i = 0; i < N; i++) {
        precomputed_events[i] = rand() % NUM_EVENTS;
    }

    uart_puts("Running benchmark (");
    uart_put_uint(N);
    uart_puts(" transitions)...\r\n");

    for (uint16_t i = 0; i < N; i++)
    {
        uint16_t c = measured_transition(precomputed_events[i]);
        if (c < min_c)
            min_c = c;
        if (c > max_c)
            max_c = c;
        sum_c += c;
    }
    
    uart_puts("Min cycles: ");
    uart_put_uint(min_c);
    uart_puts("\r\n");
    uart_puts("Max cycles: ");
    uart_put_uint(max_c);
    uart_puts("\r\n");
    uart_puts("Avg cycles: ");
    uart_put_uint((uint16_t)(sum_c / N));
    uart_puts("\r\n");
}

void setup()
{
    uart_init();
    uart_puts("\r\nMQTT FSM - Santic Lookup Table (void action procedures) pattern ready.\r\n");
    uart_puts("Commands: num(dec 0-8)=EVENT b=BENCHMARK\r\n");
    uart_puts("Current state: S0\r\n");

    srand(time(0));
}

void loop()
{
    if (uart_available())
    {
        char c = uart_getc();
        uint8_t event;

        if (c == 'b')
        {
            run_benchmark();
            return;
        }

        event = atoi(&c);

        if (event < 0 || event >= NUM_EVENTS)
        {
            return;
        }

        uint16_t cycles = measured_transition(event);

        uart_puts("Event handled -> State: ");
        uart_puts(" | Cycles: ");
        uart_put_uint(cycles);
        uart_puts("\r\n");
    }
}
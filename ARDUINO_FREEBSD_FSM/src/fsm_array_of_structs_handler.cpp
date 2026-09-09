#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>   // Neophodno zbog memorijskog zauzeca SRAM-a
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

// eventHandler funkcije - svaka vraca sledece stanje (kao u Fig. 6/9 rada)
typedef uint8_t (*EventHandler)(void);

// State S0 handlers
static uint8_t handler_s0_closeconnection(void) { return S0; }
static uint8_t handler_s0_ack_psh_vv1(void)     { return S0; }
static uint8_t handler_s0_syn_ack_vv0(void)     { return S0; }
static uint8_t handler_s0_rst_vv0(void)         { return S0; }
static uint8_t handler_s0_accept(void)          { return S0; }
static uint8_t handler_s0_fin_ack_vv0(void)     { return S0; }
static uint8_t handler_s0_listen(void)          { return S1; }
static uint8_t handler_s0_syn_vv0(void)         { return S0; }
static uint8_t handler_s0_rcv(void)             { return S0; }
static uint8_t handler_s0_ack_rst_vv0(void)     { return S0; }
static uint8_t handler_s0_close(void)           { return S2; }
static uint8_t handler_s0_send(void)            { return S0; }
static uint8_t handler_s0_ack_vv0(void)         { return S0; }

// State S1 handlers
static uint8_t handler_s1_closeconnection(void) { return S1; }
static uint8_t handler_s1_ack_psh_vv1(void)     { return S1; }
static uint8_t handler_s1_syn_ack_vv0(void)     { return S1; }
static uint8_t handler_s1_rst_vv0(void)         { return S1; }
static uint8_t handler_s1_accept(void)          { return S4; }
static uint8_t handler_s1_fin_ack_vv0(void)     { return S1; }
static uint8_t handler_s1_listen(void)          { return S1; }
static uint8_t handler_s1_syn_vv0(void)         { return S3; }
static uint8_t handler_s1_rcv(void)             { return S1; }
static uint8_t handler_s1_ack_rst_vv0(void)     { return S1; }
static uint8_t handler_s1_close(void)           { return S2; }
static uint8_t handler_s1_send(void)            { return S1; }
static uint8_t handler_s1_ack_vv0(void)         { return S1; }

// State S2 handlers
static uint8_t handler_s2_closeconnection(void) { return S2; }
static uint8_t handler_s2_ack_psh_vv1(void)     { return S2; }
static uint8_t handler_s2_syn_ack_vv0(void)     { return S2; }
static uint8_t handler_s2_rst_vv0(void)         { return S2; }
static uint8_t handler_s2_accept(void)          { return S2; }
static uint8_t handler_s2_fin_ack_vv0(void)     { return S2; }
static uint8_t handler_s2_listen(void)          { return S2; }
static uint8_t handler_s2_syn_vv0(void)         { return S2; }
static uint8_t handler_s2_rcv(void)             { return S2; }
static uint8_t handler_s2_ack_rst_vv0(void)     { return S2; }
static uint8_t handler_s2_close(void)           { return S2; }
static uint8_t handler_s2_send(void)            { return S2; }
static uint8_t handler_s2_ack_vv0(void)         { return S2; }

// State S3 handlers
static uint8_t handler_s3_closeconnection(void) { return S3; }
static uint8_t handler_s3_ack_psh_vv1(void)     { return S8; }
static uint8_t handler_s3_syn_ack_vv0(void)     { return S6; }
static uint8_t handler_s3_rst_vv0(void)         { return S1; }
static uint8_t handler_s3_accept(void)          { return S9; }
static uint8_t handler_s3_fin_ack_vv0(void)     { return S7; }
static uint8_t handler_s3_listen(void)          { return S3; }
static uint8_t handler_s3_syn_vv0(void)         { return S3; }
static uint8_t handler_s3_rcv(void)             { return S3; }
static uint8_t handler_s3_ack_rst_vv0(void)     { return S10; }
static uint8_t handler_s3_close(void)           { return S5; }
static uint8_t handler_s3_send(void)            { return S3; }
static uint8_t handler_s3_ack_vv0(void)         { return S8; }

// State S4 handlers
static uint8_t handler_s4_closeconnection(void) { return S1; }
static uint8_t handler_s4_ack_psh_vv1(void)     { return S4; }
static uint8_t handler_s4_syn_ack_vv0(void)     { return S4; }
static uint8_t handler_s4_rst_vv0(void)         { return S4; }
static uint8_t handler_s4_accept(void)          { return S4; }
static uint8_t handler_s4_fin_ack_vv0(void)     { return S4; }
static uint8_t handler_s4_listen(void)          { return S4; }
static uint8_t handler_s4_syn_vv0(void)         { return S9; }
static uint8_t handler_s4_rcv(void)             { return S4; }
static uint8_t handler_s4_ack_rst_vv0(void)     { return S4; }
static uint8_t handler_s4_close(void)           { return S2; }
static uint8_t handler_s4_send(void)            { return S4; }
static uint8_t handler_s4_ack_vv0(void)         { return S4; }

// State S5 handlers
static uint8_t handler_s5_closeconnection(void) { return S5; }
static uint8_t handler_s5_ack_psh_vv1(void)     { return S2; }
static uint8_t handler_s5_syn_ack_vv0(void)     { return S5; }
static uint8_t handler_s5_rst_vv0(void)         { return S2; }
static uint8_t handler_s5_accept(void)          { return S5; }
static uint8_t handler_s5_fin_ack_vv0(void)     { return S2; }
static uint8_t handler_s5_listen(void)          { return S5; }
static uint8_t handler_s5_syn_vv0(void)         { return S2; }
static uint8_t handler_s5_rcv(void)             { return S5; }
static uint8_t handler_s5_ack_rst_vv0(void)     { return S2; }
static uint8_t handler_s5_close(void)           { return S5; }
static uint8_t handler_s5_send(void)            { return S5; }
static uint8_t handler_s5_ack_vv0(void)         { return S2; }

// State S6 handlers
static uint8_t handler_s6_closeconnection(void) { return S6; }
static uint8_t handler_s6_ack_psh_vv1(void)     { return S1; }
static uint8_t handler_s6_syn_ack_vv0(void)     { return S6; }
static uint8_t handler_s6_rst_vv0(void)         { return S1; }
static uint8_t handler_s6_accept(void)          { return S11; }
static uint8_t handler_s6_fin_ack_vv0(void)     { return S1; }
static uint8_t handler_s6_listen(void)          { return S6; }
static uint8_t handler_s6_syn_vv0(void)         { return S3; }
static uint8_t handler_s6_rcv(void)             { return S6; }
static uint8_t handler_s6_ack_rst_vv0(void)     { return S1; }
static uint8_t handler_s6_close(void)           { return S5; }
static uint8_t handler_s6_send(void)            { return S6; }
static uint8_t handler_s6_ack_vv0(void)         { return S1; }

// State S7 handlers
static uint8_t handler_s7_closeconnection(void) { return S7; }
static uint8_t handler_s7_ack_psh_vv1(void)     { return S7; }
static uint8_t handler_s7_syn_ack_vv0(void)     { return S12; }
static uint8_t handler_s7_rst_vv0(void)         { return S12; }
static uint8_t handler_s7_accept(void)          { return S13; }
static uint8_t handler_s7_fin_ack_vv0(void)     { return S7; }
static uint8_t handler_s7_listen(void)          { return S7; }
static uint8_t handler_s7_syn_vv0(void)         { return S12; }
static uint8_t handler_s7_rcv(void)             { return S7; }
static uint8_t handler_s7_ack_rst_vv0(void)     { return S12; }
static uint8_t handler_s7_close(void)           { return S2; }
static uint8_t handler_s7_send(void)            { return S7; }
static uint8_t handler_s7_ack_vv0(void)         { return S7; }

// State S8 handlers
static uint8_t handler_s8_closeconnection(void) { return S8; }
static uint8_t handler_s8_ack_psh_vv1(void)     { return S8; }
static uint8_t handler_s8_syn_ack_vv0(void)     { return S12; }
static uint8_t handler_s8_rst_vv0(void)         { return S12; }
static uint8_t handler_s8_accept(void)          { return S14; }
static uint8_t handler_s8_fin_ack_vv0(void)     { return S7; }
static uint8_t handler_s8_listen(void)          { return S8; }
static uint8_t handler_s8_syn_vv0(void)         { return S12; }
static uint8_t handler_s8_rcv(void)             { return S8; }
static uint8_t handler_s8_ack_rst_vv0(void)     { return S12; }
static uint8_t handler_s8_close(void)           { return S2; }
static uint8_t handler_s8_send(void)            { return S8; }
static uint8_t handler_s8_ack_vv0(void)         { return S8; }

// State S9 handlers
static uint8_t handler_s9_closeconnection(void) { return S3; }
static uint8_t handler_s9_ack_psh_vv1(void)     { return S14; }
static uint8_t handler_s9_syn_ack_vv0(void)     { return S11; }
static uint8_t handler_s9_rst_vv0(void)         { return S4; }
static uint8_t handler_s9_accept(void)          { return S9; }
static uint8_t handler_s9_fin_ack_vv0(void)     { return S13; }
static uint8_t handler_s9_listen(void)          { return S9; }
static uint8_t handler_s9_syn_vv0(void)         { return S9; }
static uint8_t handler_s9_rcv(void)             { return S9; }
static uint8_t handler_s9_ack_rst_vv0(void)     { return S15; }
static uint8_t handler_s9_close(void)           { return S5; }
static uint8_t handler_s9_send(void)            { return S9; }
static uint8_t handler_s9_ack_vv0(void)         { return S14; }

// State S10 handlers
static uint8_t handler_s10_closeconnection(void) { return S10; }
static uint8_t handler_s10_ack_psh_vv1(void)     { return S1; }
static uint8_t handler_s10_syn_ack_vv0(void)     { return S1; }
static uint8_t handler_s10_rst_vv0(void)         { return S10; }
static uint8_t handler_s10_accept(void)          { return S15; }
static uint8_t handler_s10_fin_ack_vv0(void)     { return S1; }
static uint8_t handler_s10_listen(void)          { return S10; }
static uint8_t handler_s10_syn_vv0(void)         { return S10; }
static uint8_t handler_s10_rcv(void)             { return S10; }
static uint8_t handler_s10_ack_rst_vv0(void)     { return S10; }
static uint8_t handler_s10_close(void)           { return S2; }
static uint8_t handler_s10_send(void)            { return S10; }
static uint8_t handler_s10_ack_vv0(void)         { return S1; }

// State S11 handlers
static uint8_t handler_s11_closeconnection(void) { return S6; }
static uint8_t handler_s11_ack_psh_vv1(void)     { return S4; }
static uint8_t handler_s11_syn_ack_vv0(void)     { return S11; }
static uint8_t handler_s11_rst_vv0(void)         { return S4; }
static uint8_t handler_s11_accept(void)          { return S11; }
static uint8_t handler_s11_fin_ack_vv0(void)     { return S4; }
static uint8_t handler_s11_listen(void)          { return S11; }
static uint8_t handler_s11_syn_vv0(void)         { return S9; }
static uint8_t handler_s11_rcv(void)             { return S11; }
static uint8_t handler_s11_ack_rst_vv0(void)     { return S4; }
static uint8_t handler_s11_close(void)           { return S5; }
static uint8_t handler_s11_send(void)            { return S11; }
static uint8_t handler_s11_ack_vv0(void)         { return S4; }

// State S12 handlers
static uint8_t handler_s12_closeconnection(void) { return S12; }
static uint8_t handler_s12_ack_psh_vv1(void)     { return S12; }
static uint8_t handler_s12_syn_ack_vv0(void)     { return S12; }
static uint8_t handler_s12_rst_vv0(void)         { return S12; }
static uint8_t handler_s12_accept(void)          { return S1; }
static uint8_t handler_s12_fin_ack_vv0(void)     { return S12; }
static uint8_t handler_s12_listen(void)          { return S12; }
static uint8_t handler_s12_syn_vv0(void)         { return S16; }
static uint8_t handler_s12_rcv(void)             { return S12; }
static uint8_t handler_s12_ack_rst_vv0(void)     { return S12; }
static uint8_t handler_s12_close(void)           { return S2; }
static uint8_t handler_s12_send(void)            { return S12; }
static uint8_t handler_s12_ack_vv0(void)         { return S12; }

// State S13 handlers
static uint8_t handler_s13_closeconnection(void) { return S18; }
static uint8_t handler_s13_ack_psh_vv1(void)     { return S13; }
static uint8_t handler_s13_syn_ack_vv0(void)     { return S19; }
static uint8_t handler_s13_rst_vv0(void)         { return S19; }
static uint8_t handler_s13_accept(void)          { return S13; }
static uint8_t handler_s13_fin_ack_vv0(void)     { return S13; }
static uint8_t handler_s13_listen(void)          { return S13; }
static uint8_t handler_s13_syn_vv0(void)         { return S19; }
static uint8_t handler_s13_rcv(void)             { return S13; }
static uint8_t handler_s13_ack_rst_vv0(void)     { return S19; }
static uint8_t handler_s13_close(void)           { return S17; }
static uint8_t handler_s13_send(void)            { return S13; }
static uint8_t handler_s13_ack_vv0(void)         { return S13; }

// State S14 handlers
static uint8_t handler_s14_closeconnection(void) { return S21; }
static uint8_t handler_s14_ack_psh_vv1(void)     { return S14; }
static uint8_t handler_s14_syn_ack_vv0(void)     { return S19; }
static uint8_t handler_s14_rst_vv0(void)         { return S19; }
static uint8_t handler_s14_accept(void)          { return S14; }
static uint8_t handler_s14_fin_ack_vv0(void)     { return S13; }
static uint8_t handler_s14_listen(void)          { return S14; }
static uint8_t handler_s14_syn_vv0(void)         { return S19; }
static uint8_t handler_s14_rcv(void)             { return S14; }
static uint8_t handler_s14_ack_rst_vv0(void)     { return S19; }
static uint8_t handler_s14_close(void)           { return S20; }
static uint8_t handler_s14_send(void)            { return S14; }
static uint8_t handler_s14_ack_vv0(void)         { return S14; }

// State S15 handlers
static uint8_t handler_s15_closeconnection(void) { return S10; }
static uint8_t handler_s15_ack_psh_vv1(void)     { return S4; }
static uint8_t handler_s15_syn_ack_vv0(void)     { return S4; }
static uint8_t handler_s15_rst_vv0(void)         { return S15; }
static uint8_t handler_s15_accept(void)          { return S15; }
static uint8_t handler_s15_fin_ack_vv0(void)     { return S4; }
static uint8_t handler_s15_listen(void)          { return S15; }
static uint8_t handler_s15_syn_vv0(void)         { return S15; }
static uint8_t handler_s15_rcv(void)             { return S15; }
static uint8_t handler_s15_ack_rst_vv0(void)     { return S15; }
static uint8_t handler_s15_close(void)           { return S2; }
static uint8_t handler_s15_send(void)            { return S15; }
static uint8_t handler_s15_ack_vv0(void)         { return S4; }

// State S16 handlers
static uint8_t handler_s16_closeconnection(void) { return S16; }
static uint8_t handler_s16_ack_psh_vv1(void)     { return S23; }
static uint8_t handler_s16_syn_ack_vv0(void)     { return S22; }
static uint8_t handler_s16_rst_vv0(void)         { return S12; }
static uint8_t handler_s16_accept(void)          { return S3; }
static uint8_t handler_s16_fin_ack_vv0(void)     { return S24; }
static uint8_t handler_s16_listen(void)          { return S16; }
static uint8_t handler_s16_syn_vv0(void)         { return S16; }
static uint8_t handler_s16_rcv(void)             { return S16; }
static uint8_t handler_s16_ack_rst_vv0(void)     { return S25; }
static uint8_t handler_s16_close(void)           { return S5; }
static uint8_t handler_s16_send(void)            { return S16; }
static uint8_t handler_s16_ack_vv0(void)         { return S23; }

// State S17 handlers
static uint8_t handler_s17_closeconnection(void) { return S26; }
static uint8_t handler_s17_ack_psh_vv1(void)     { return S17; }
static uint8_t handler_s17_syn_ack_vv0(void)     { return S2; }
static uint8_t handler_s17_rst_vv0(void)         { return S2; }
static uint8_t handler_s17_accept(void)          { return S17; }
static uint8_t handler_s17_fin_ack_vv0(void)     { return S17; }
static uint8_t handler_s17_listen(void)          { return S17; }
static uint8_t handler_s17_syn_vv0(void)         { return S2; }
static uint8_t handler_s17_rcv(void)             { return S17; }
static uint8_t handler_s17_ack_rst_vv0(void)     { return S2; }
static uint8_t handler_s17_close(void)           { return S17; }
static uint8_t handler_s17_send(void)            { return S17; }
static uint8_t handler_s17_ack_vv0(void)         { return S17; }

// State S18 handlers
static uint8_t handler_s18_closeconnection(void) { return S18; }
static uint8_t handler_s18_ack_psh_vv1(void)     { return S1; }
static uint8_t handler_s18_syn_ack_vv0(void)     { return S1; }
static uint8_t handler_s18_rst_vv0(void)         { return S1; }
static uint8_t handler_s18_accept(void)          { return S27; }
static uint8_t handler_s18_fin_ack_vv0(void)     { return S6; }
static uint8_t handler_s18_listen(void)          { return S18; }
static uint8_t handler_s18_syn_vv0(void)         { return S1; }
static uint8_t handler_s18_rcv(void)             { return S18; }
static uint8_t handler_s18_ack_rst_vv0(void)     { return S1; }
static uint8_t handler_s18_close(void)           { return S26; }
static uint8_t handler_s18_send(void)            { return S18; }
static uint8_t handler_s18_ack_vv0(void)         { return S6; }

// State S19 handlers
static uint8_t handler_s19_closeconnection(void) { return S1; }
static uint8_t handler_s19_ack_psh_vv1(void)     { return S19; }
static uint8_t handler_s19_syn_ack_vv0(void)     { return S19; }
static uint8_t handler_s19_rst_vv0(void)         { return S19; }
static uint8_t handler_s19_accept(void)          { return S19; }
static uint8_t handler_s19_fin_ack_vv0(void)     { return S19; }
static uint8_t handler_s19_listen(void)          { return S19; }
static uint8_t handler_s19_syn_vv0(void)         { return S28; }
static uint8_t handler_s19_rcv(void)             { return S19; }
static uint8_t handler_s19_ack_rst_vv0(void)     { return S19; }
static uint8_t handler_s19_close(void)           { return S2; }
static uint8_t handler_s19_send(void)            { return S19; }
static uint8_t handler_s19_ack_vv0(void)         { return S19; }

// State S20 handlers
static uint8_t handler_s20_closeconnection(void) { return S29; }
static uint8_t handler_s20_ack_psh_vv1(void)     { return S20; }
static uint8_t handler_s20_syn_ack_vv0(void)     { return S2; }
static uint8_t handler_s20_rst_vv0(void)         { return S2; }
static uint8_t handler_s20_accept(void)          { return S20; }
static uint8_t handler_s20_fin_ack_vv0(void)     { return S17; }
static uint8_t handler_s20_listen(void)          { return S20; }
static uint8_t handler_s20_syn_vv0(void)         { return S2; }
static uint8_t handler_s20_rcv(void)             { return S20; }
static uint8_t handler_s20_ack_rst_vv0(void)     { return S2; }
static uint8_t handler_s20_close(void)           { return S20; }
static uint8_t handler_s20_send(void)            { return S20; }
static uint8_t handler_s20_ack_vv0(void)         { return S20; }

// State S21 handlers
static uint8_t handler_s21_closeconnection(void) { return S21; }
static uint8_t handler_s21_ack_psh_vv1(void)     { return S1; }
static uint8_t handler_s21_syn_ack_vv0(void)     { return S1; }
static uint8_t handler_s21_rst_vv0(void)         { return S1; }
static uint8_t handler_s21_accept(void)          { return S30; }
static uint8_t handler_s21_fin_ack_vv0(void)     { return S31; }
static uint8_t handler_s21_listen(void)          { return S21; }
static uint8_t handler_s21_syn_vv0(void)         { return S1; }
static uint8_t handler_s21_rcv(void)             { return S21; }
static uint8_t handler_s21_ack_rst_vv0(void)     { return S1; }
static uint8_t handler_s21_close(void)           { return S29; }
static uint8_t handler_s21_send(void)            { return S21; }
static uint8_t handler_s21_ack_vv0(void)         { return S21; }

// State S22 handlers
static uint8_t handler_s22_closeconnection(void) { return S22; }
static uint8_t handler_s22_ack_psh_vv1(void)     { return S12; }
static uint8_t handler_s22_syn_ack_vv0(void)     { return S22; }
static uint8_t handler_s22_rst_vv0(void)         { return S12; }
static uint8_t handler_s22_accept(void)          { return S6; }
static uint8_t handler_s22_fin_ack_vv0(void)     { return S12; }
static uint8_t handler_s22_listen(void)          { return S22; }
static uint8_t handler_s22_syn_vv0(void)         { return S16; }
static uint8_t handler_s22_rcv(void)             { return S22; }
static uint8_t handler_s22_ack_rst_vv0(void)     { return S12; }
static uint8_t handler_s22_close(void)           { return S5; }
static uint8_t handler_s22_send(void)            { return S22; }
static uint8_t handler_s22_ack_vv0(void)         { return S12; }

// State S23 handlers
static uint8_t handler_s23_closeconnection(void) { return S23; }
static uint8_t handler_s23_ack_psh_vv1(void)     { return S23; }
static uint8_t handler_s23_syn_ack_vv0(void)     { return S32; }
static uint8_t handler_s23_rst_vv0(void)         { return S32; }
static uint8_t handler_s23_accept(void)          { return S8; }
static uint8_t handler_s23_fin_ack_vv0(void)     { return S24; }
static uint8_t handler_s23_listen(void)          { return S23; }
static uint8_t handler_s23_syn_vv0(void)         { return S32; }
static uint8_t handler_s23_rcv(void)             { return S23; }
static uint8_t handler_s23_ack_rst_vv0(void)     { return S32; }
static uint8_t handler_s23_close(void)           { return S2; }
static uint8_t handler_s23_send(void)            { return S23; }
static uint8_t handler_s23_ack_vv0(void)         { return S23; }

// State S24 handlers
static uint8_t handler_s24_closeconnection(void) { return S24; }
static uint8_t handler_s24_ack_psh_vv1(void)     { return S24; }
static uint8_t handler_s24_syn_ack_vv0(void)     { return S32; }
static uint8_t handler_s24_rst_vv0(void)         { return S32; }
static uint8_t handler_s24_accept(void)          { return S7; }
static uint8_t handler_s24_fin_ack_vv0(void)     { return S24; }
static uint8_t handler_s24_listen(void)          { return S24; }
static uint8_t handler_s24_syn_vv0(void)         { return S32; }
static uint8_t handler_s24_rcv(void)             { return S24; }
static uint8_t handler_s24_ack_rst_vv0(void)     { return S32; }
static uint8_t handler_s24_close(void)           { return S2; }
static uint8_t handler_s24_send(void)            { return S24; }
static uint8_t handler_s24_ack_vv0(void)         { return S24; }

// State S25 handlers
static uint8_t handler_s25_closeconnection(void) { return S25; }
static uint8_t handler_s25_ack_psh_vv1(void)     { return S12; }
static uint8_t handler_s25_syn_ack_vv0(void)     { return S12; }
static uint8_t handler_s25_rst_vv0(void)         { return S25; }
static uint8_t handler_s25_accept(void)          { return S10; }
static uint8_t handler_s25_fin_ack_vv0(void)     { return S12; }
static uint8_t handler_s25_listen(void)          { return S25; }
static uint8_t handler_s25_syn_vv0(void)         { return S25; }
static uint8_t handler_s25_rcv(void)             { return S25; }
static uint8_t handler_s25_ack_rst_vv0(void)     { return S25; }
static uint8_t handler_s25_close(void)           { return S2; }
static uint8_t handler_s25_send(void)            { return S25; }
static uint8_t handler_s25_ack_vv0(void)         { return S12; }

// State S26 handlers
static uint8_t handler_s26_closeconnection(void) { return S26; }
static uint8_t handler_s26_ack_psh_vv1(void)     { return S2; }
static uint8_t handler_s26_syn_ack_vv0(void)     { return S2; }
static uint8_t handler_s26_rst_vv0(void)         { return S2; }
static uint8_t handler_s26_accept(void)          { return S26; }
static uint8_t handler_s26_fin_ack_vv0(void)     { return S5; }
static uint8_t handler_s26_listen(void)          { return S26; }
static uint8_t handler_s26_syn_vv0(void)         { return S2; }
static uint8_t handler_s26_rcv(void)             { return S26; }
static uint8_t handler_s26_ack_rst_vv0(void)     { return S2; }
static uint8_t handler_s26_close(void)           { return S26; }
static uint8_t handler_s26_send(void)            { return S26; }
static uint8_t handler_s26_ack_vv0(void)         { return S5; }

// State S27 handlers
static uint8_t handler_s27_closeconnection(void) { return S18; }
static uint8_t handler_s27_ack_psh_vv1(void)     { return S4; }
static uint8_t handler_s27_syn_ack_vv0(void)     { return S4; }
static uint8_t handler_s27_rst_vv0(void)         { return S4; }
static uint8_t handler_s27_accept(void)          { return S27; }
static uint8_t handler_s27_fin_ack_vv0(void)     { return S11; }
static uint8_t handler_s27_listen(void)          { return S27; }
static uint8_t handler_s27_syn_vv0(void)         { return S4; }
static uint8_t handler_s27_rcv(void)             { return S27; }
static uint8_t handler_s27_ack_rst_vv0(void)     { return S4; }
static uint8_t handler_s27_close(void)           { return S26; }
static uint8_t handler_s27_send(void)            { return S27; }
static uint8_t handler_s27_ack_vv0(void)         { return S11; }

// State S28 handlers
static uint8_t handler_s28_closeconnection(void) { return S3; }
static uint8_t handler_s28_ack_psh_vv1(void)     { return S34; }
static uint8_t handler_s28_syn_ack_vv0(void)     { return S36; }
static uint8_t handler_s28_rst_vv0(void)         { return S19; }
static uint8_t handler_s28_accept(void)          { return S28; }
static uint8_t handler_s28_fin_ack_vv0(void)     { return S33; }
static uint8_t handler_s28_listen(void)          { return S28; }
static uint8_t handler_s28_syn_vv0(void)         { return S28; }
static uint8_t handler_s28_rcv(void)             { return S28; }
static uint8_t handler_s28_ack_rst_vv0(void)     { return S35; }
static uint8_t handler_s28_close(void)           { return S5; }
static uint8_t handler_s28_send(void)            { return S28; }
static uint8_t handler_s28_ack_vv0(void)         { return S34; }

// State S29 handlers
static uint8_t handler_s29_closeconnection(void) { return S29; }
static uint8_t handler_s29_ack_psh_vv1(void)     { return S2; }
static uint8_t handler_s29_syn_ack_vv0(void)     { return S2; }
static uint8_t handler_s29_rst_vv0(void)         { return S2; }
static uint8_t handler_s29_accept(void)          { return S29; }
static uint8_t handler_s29_fin_ack_vv0(void)     { return S37; }
static uint8_t handler_s29_listen(void)          { return S29; }
static uint8_t handler_s29_syn_vv0(void)         { return S2; }
static uint8_t handler_s29_rcv(void)             { return S29; }
static uint8_t handler_s29_ack_rst_vv0(void)     { return S2; }
static uint8_t handler_s29_close(void)           { return S29; }
static uint8_t handler_s29_send(void)            { return S29; }
static uint8_t handler_s29_ack_vv0(void)         { return S29; }

// State S30 handlers
static uint8_t handler_s30_closeconnection(void) { return S21; }
static uint8_t handler_s30_ack_psh_vv1(void)     { return S4; }
static uint8_t handler_s30_syn_ack_vv0(void)     { return S4; }
static uint8_t handler_s30_rst_vv0(void)         { return S4; }
static uint8_t handler_s30_accept(void)          { return S30; }
static uint8_t handler_s30_fin_ack_vv0(void)     { return S38; }
static uint8_t handler_s30_listen(void)          { return S30; }
static uint8_t handler_s30_syn_vv0(void)         { return S4; }
static uint8_t handler_s30_rcv(void)             { return S30; }
static uint8_t handler_s30_ack_rst_vv0(void)     { return S4; }
static uint8_t handler_s30_close(void)           { return S29; }
static uint8_t handler_s30_send(void)            { return S30; }
static uint8_t handler_s30_ack_vv0(void)         { return S30; }

// State S31 handlers
static uint8_t handler_s31_closeconnection(void) { return S31; }
static uint8_t handler_s31_ack_psh_vv1(void)     { return S31; }
static uint8_t handler_s31_syn_ack_vv0(void)     { return S31; }
static uint8_t handler_s31_rst_vv0(void)         { return S39; }
static uint8_t handler_s31_accept(void)          { return S38; }
static uint8_t handler_s31_fin_ack_vv0(void)     { return S31; }
static uint8_t handler_s31_listen(void)          { return S31; }
static uint8_t handler_s31_syn_vv0(void)         { return S31; }
static uint8_t handler_s31_rcv(void)             { return S31; }
static uint8_t handler_s31_ack_rst_vv0(void)     { return S39; }
static uint8_t handler_s31_close(void)           { return S37; }
static uint8_t handler_s31_send(void)            { return S31; }
static uint8_t handler_s31_ack_vv0(void)         { return S31; }

// State S32 handlers
static uint8_t handler_s32_closeconnection(void) { return S32; }
static uint8_t handler_s32_ack_psh_vv1(void)     { return S32; }
static uint8_t handler_s32_syn_ack_vv0(void)     { return S32; }
static uint8_t handler_s32_rst_vv0(void)         { return S32; }
static uint8_t handler_s32_accept(void)          { return S12; }
static uint8_t handler_s32_fin_ack_vv0(void)     { return S32; }
static uint8_t handler_s32_listen(void)          { return S32; }
static uint8_t handler_s32_syn_vv0(void)         { return S40; }
static uint8_t handler_s32_rcv(void)             { return S32; }
static uint8_t handler_s32_ack_rst_vv0(void)     { return S32; }
static uint8_t handler_s32_close(void)           { return S2; }
static uint8_t handler_s32_send(void)            { return S32; }
static uint8_t handler_s32_ack_vv0(void)         { return S32; }

// State S33 handlers
static uint8_t handler_s33_closeconnection(void) { return S7; }
static uint8_t handler_s33_ack_psh_vv1(void)     { return S33; }
static uint8_t handler_s33_syn_ack_vv0(void)     { return S41; }
static uint8_t handler_s33_rst_vv0(void)         { return S41; }
static uint8_t handler_s33_accept(void)          { return S33; }
static uint8_t handler_s33_fin_ack_vv0(void)     { return S33; }
static uint8_t handler_s33_listen(void)          { return S33; }
static uint8_t handler_s33_syn_vv0(void)         { return S41; }
static uint8_t handler_s33_rcv(void)             { return S33; }
static uint8_t handler_s33_ack_rst_vv0(void)     { return S41; }
static uint8_t handler_s33_close(void)           { return S2; }
static uint8_t handler_s33_send(void)            { return S33; }
static uint8_t handler_s33_ack_vv0(void)         { return S33; }

// State S34 handlers
static uint8_t handler_s34_closeconnection(void) { return S8; }
static uint8_t handler_s34_ack_psh_vv1(void)     { return S34; }
static uint8_t handler_s34_syn_ack_vv0(void)     { return S41; }
static uint8_t handler_s34_rst_vv0(void)         { return S41; }
static uint8_t handler_s34_accept(void)          { return S34; }
static uint8_t handler_s34_fin_ack_vv0(void)     { return S33; }
static uint8_t handler_s34_listen(void)          { return S34; }
static uint8_t handler_s34_syn_vv0(void)         { return S41; }
static uint8_t handler_s34_rcv(void)             { return S34; }
static uint8_t handler_s34_ack_rst_vv0(void)     { return S41; }
static uint8_t handler_s34_close(void)           { return S2; }
static uint8_t handler_s34_send(void)            { return S34; }
static uint8_t handler_s34_ack_vv0(void)         { return S34; }

// State S35 handlers
static uint8_t handler_s35_closeconnection(void) { return S10; }
static uint8_t handler_s35_ack_psh_vv1(void)     { return S19; }
static uint8_t handler_s35_syn_ack_vv0(void)     { return S19; }
static uint8_t handler_s35_rst_vv0(void)         { return S35; }
static uint8_t handler_s35_accept(void)          { return S35; }
static uint8_t handler_s35_fin_ack_vv0(void)     { return S19; }
static uint8_t handler_s35_listen(void)          { return S35; }
static uint8_t handler_s35_syn_vv0(void)         { return S35; }
static uint8_t handler_s35_rcv(void)             { return S35; }
static uint8_t handler_s35_ack_rst_vv0(void)     { return S35; }
static uint8_t handler_s35_close(void)           { return S2; }
static uint8_t handler_s35_send(void)            { return S35; }
static uint8_t handler_s35_ack_vv0(void)         { return S19; }

// State S36 handlers
static uint8_t handler_s36_closeconnection(void) { return S6; }
static uint8_t handler_s36_ack_psh_vv1(void)     { return S19; }
static uint8_t handler_s36_syn_ack_vv0(void)     { return S36; }
static uint8_t handler_s36_rst_vv0(void)         { return S19; }
static uint8_t handler_s36_accept(void)          { return S36; }
static uint8_t handler_s36_fin_ack_vv0(void)     { return S19; }
static uint8_t handler_s36_listen(void)          { return S36; }
static uint8_t handler_s36_syn_vv0(void)         { return S28; }
static uint8_t handler_s36_rcv(void)             { return S36; }
static uint8_t handler_s36_ack_rst_vv0(void)     { return S19; }
static uint8_t handler_s36_close(void)           { return S5; }
static uint8_t handler_s36_send(void)            { return S36; }
static uint8_t handler_s36_ack_vv0(void)         { return S19; }

// State S37 handlers
static uint8_t handler_s37_closeconnection(void) { return S37; }
static uint8_t handler_s37_ack_psh_vv1(void)     { return S37; }
static uint8_t handler_s37_syn_ack_vv0(void)     { return S37; }
static uint8_t handler_s37_rst_vv0(void)         { return S42; }
static uint8_t handler_s37_accept(void)          { return S37; }
static uint8_t handler_s37_fin_ack_vv0(void)     { return S37; }
static uint8_t handler_s37_listen(void)          { return S37; }
static uint8_t handler_s37_syn_vv0(void)         { return S37; }
static uint8_t handler_s37_rcv(void)             { return S37; }
static uint8_t handler_s37_ack_rst_vv0(void)     { return S42; }
static uint8_t handler_s37_close(void)           { return S37; }
static uint8_t handler_s37_send(void)            { return S37; }
static uint8_t handler_s37_ack_vv0(void)         { return S37; }

// State S38 handlers
static uint8_t handler_s38_closeconnection(void) { return S31; }
static uint8_t handler_s38_ack_psh_vv1(void)     { return S38; }
static uint8_t handler_s38_syn_ack_vv0(void)     { return S38; }
static uint8_t handler_s38_rst_vv0(void)         { return S43; }
static uint8_t handler_s38_accept(void)          { return S38; }
static uint8_t handler_s38_fin_ack_vv0(void)     { return S38; }
static uint8_t handler_s38_listen(void)          { return S38; }
static uint8_t handler_s38_syn_vv0(void)         { return S38; }
static uint8_t handler_s38_rcv(void)             { return S38; }
static uint8_t handler_s38_ack_rst_vv0(void)     { return S43; }
static uint8_t handler_s38_close(void)           { return S37; }
static uint8_t handler_s38_send(void)            { return S38; }
static uint8_t handler_s38_ack_vv0(void)         { return S38; }

// State S39 handlers
static uint8_t handler_s39_closeconnection(void) { return S39; }
static uint8_t handler_s39_ack_psh_vv1(void)     { return S39; }
static uint8_t handler_s39_syn_ack_vv0(void)     { return S39; }
static uint8_t handler_s39_rst_vv0(void)         { return S39; }
static uint8_t handler_s39_accept(void)          { return S43; }
static uint8_t handler_s39_fin_ack_vv0(void)     { return S39; }
static uint8_t handler_s39_listen(void)          { return S39; }
static uint8_t handler_s39_syn_vv0(void)         { return S3; }
static uint8_t handler_s39_rcv(void)             { return S39; }
static uint8_t handler_s39_ack_rst_vv0(void)     { return S39; }
static uint8_t handler_s39_close(void)           { return S42; }
static uint8_t handler_s39_send(void)            { return S39; }
static uint8_t handler_s39_ack_vv0(void)         { return S39; }

// State S40 handlers
static uint8_t handler_s40_closeconnection(void) { return S40; }
static uint8_t handler_s40_ack_psh_vv1(void)     { return S32; }
static uint8_t handler_s40_syn_ack_vv0(void)     { return S44; }
static uint8_t handler_s40_rst_vv0(void)         { return S32; }
static uint8_t handler_s40_accept(void)          { return S16; }
static uint8_t handler_s40_fin_ack_vv0(void)     { return S32; }
static uint8_t handler_s40_listen(void)          { return S40; }
static uint8_t handler_s40_syn_vv0(void)         { return S40; }
static uint8_t handler_s40_rcv(void)             { return S40; }
static uint8_t handler_s40_ack_rst_vv0(void)     { return S45; }
static uint8_t handler_s40_close(void)           { return S5; }
static uint8_t handler_s40_send(void)            { return S40; }
static uint8_t handler_s40_ack_vv0(void)         { return S32; }

// State S41 handlers
static uint8_t handler_s41_closeconnection(void) { return S12; }
static uint8_t handler_s41_ack_psh_vv1(void)     { return S41; }
static uint8_t handler_s41_syn_ack_vv0(void)     { return S41; }
static uint8_t handler_s41_rst_vv0(void)         { return S41; }
static uint8_t handler_s41_accept(void)          { return S41; }
static uint8_t handler_s41_fin_ack_vv0(void)     { return S41; }
static uint8_t handler_s41_listen(void)          { return S41; }
static uint8_t handler_s41_syn_vv0(void)         { return S46; }
static uint8_t handler_s41_rcv(void)             { return S41; }
static uint8_t handler_s41_ack_rst_vv0(void)     { return S41; }
static uint8_t handler_s41_close(void)           { return S2; }
static uint8_t handler_s41_send(void)            { return S41; }
static uint8_t handler_s41_ack_vv0(void)         { return S41; }

// State S42 handlers
static uint8_t handler_s42_closeconnection(void) { return S42; }
static uint8_t handler_s42_ack_psh_vv1(void)     { return S42; }
static uint8_t handler_s42_syn_ack_vv0(void)     { return S42; }
static uint8_t handler_s42_rst_vv0(void)         { return S42; }
static uint8_t handler_s42_accept(void)          { return S42; }
static uint8_t handler_s42_fin_ack_vv0(void)     { return S42; }
static uint8_t handler_s42_listen(void)          { return S42; }
static uint8_t handler_s42_syn_vv0(void)         { return S2; }
static uint8_t handler_s42_rcv(void)             { return S42; }
static uint8_t handler_s42_ack_rst_vv0(void)     { return S42; }
static uint8_t handler_s42_close(void)           { return S42; }
static uint8_t handler_s42_send(void)            { return S42; }
static uint8_t handler_s42_ack_vv0(void)         { return S42; }

// State S43 handlers
static uint8_t handler_s43_closeconnection(void) { return S39; }
static uint8_t handler_s43_ack_psh_vv1(void)     { return S43; }
static uint8_t handler_s43_syn_ack_vv0(void)     { return S43; }
static uint8_t handler_s43_rst_vv0(void)         { return S43; }
static uint8_t handler_s43_accept(void)          { return S43; }
static uint8_t handler_s43_fin_ack_vv0(void)     { return S43; }
static uint8_t handler_s43_listen(void)          { return S43; }
static uint8_t handler_s43_syn_vv0(void)         { return S9; }
static uint8_t handler_s43_rcv(void)             { return S43; }
static uint8_t handler_s43_ack_rst_vv0(void)     { return S43; }
static uint8_t handler_s43_close(void)           { return S42; }
static uint8_t handler_s43_send(void)            { return S43; }
static uint8_t handler_s43_ack_vv0(void)         { return S43; }

// State S44 handlers
static uint8_t handler_s44_closeconnection(void) { return S44; }
static uint8_t handler_s44_ack_psh_vv1(void)     { return S32; }
static uint8_t handler_s44_syn_ack_vv0(void)     { return S44; }
static uint8_t handler_s44_rst_vv0(void)         { return S32; }
static uint8_t handler_s44_accept(void)          { return S22; }
static uint8_t handler_s44_fin_ack_vv0(void)     { return S32; }
static uint8_t handler_s44_listen(void)          { return S44; }
static uint8_t handler_s44_syn_vv0(void)         { return S40; }
static uint8_t handler_s44_rcv(void)             { return S44; }
static uint8_t handler_s44_ack_rst_vv0(void)     { return S32; }
static uint8_t handler_s44_close(void)           { return S5; }
static uint8_t handler_s44_send(void)            { return S44; }
static uint8_t handler_s44_ack_vv0(void)         { return S32; }

// State S45 handlers
static uint8_t handler_s45_closeconnection(void) { return S45; }
static uint8_t handler_s45_ack_psh_vv1(void)     { return S32; }
static uint8_t handler_s45_syn_ack_vv0(void)     { return S32; }
static uint8_t handler_s45_rst_vv0(void)         { return S45; }
static uint8_t handler_s45_accept(void)          { return S25; }
static uint8_t handler_s45_fin_ack_vv0(void)     { return S32; }
static uint8_t handler_s45_listen(void)          { return S45; }
static uint8_t handler_s45_syn_vv0(void)         { return S45; }
static uint8_t handler_s45_rcv(void)             { return S45; }
static uint8_t handler_s45_ack_rst_vv0(void)     { return S45; }
static uint8_t handler_s45_close(void)           { return S2; }
static uint8_t handler_s45_send(void)            { return S45; }
static uint8_t handler_s45_ack_vv0(void)         { return S32; }

// State S46 handlers
static uint8_t handler_s46_closeconnection(void) { return S16; }
static uint8_t handler_s46_ack_psh_vv1(void)     { return S48; }
static uint8_t handler_s46_syn_ack_vv0(void)     { return S49; }
static uint8_t handler_s46_rst_vv0(void)         { return S41; }
static uint8_t handler_s46_accept(void)          { return S46; }
static uint8_t handler_s46_fin_ack_vv0(void)     { return S50; }
static uint8_t handler_s46_listen(void)          { return S46; }
static uint8_t handler_s46_syn_vv0(void)         { return S46; }
static uint8_t handler_s46_rcv(void)             { return S46; }
static uint8_t handler_s46_ack_rst_vv0(void)     { return S47; }
static uint8_t handler_s46_close(void)           { return S5; }
static uint8_t handler_s46_send(void)            { return S46; }
static uint8_t handler_s46_ack_vv0(void)         { return S48; }

// State S47 handlers
static uint8_t handler_s47_closeconnection(void) { return S25; }
static uint8_t handler_s47_ack_psh_vv1(void)     { return S41; }
static uint8_t handler_s47_syn_ack_vv0(void)     { return S41; }
static uint8_t handler_s47_rst_vv0(void)         { return S47; }
static uint8_t handler_s47_accept(void)          { return S47; }
static uint8_t handler_s47_fin_ack_vv0(void)     { return S41; }
static uint8_t handler_s47_listen(void)          { return S47; }
static uint8_t handler_s47_syn_vv0(void)         { return S47; }
static uint8_t handler_s47_rcv(void)             { return S47; }
static uint8_t handler_s47_ack_rst_vv0(void)     { return S47; }
static uint8_t handler_s47_close(void)           { return S2; }
static uint8_t handler_s47_send(void)            { return S47; }
static uint8_t handler_s47_ack_vv0(void)         { return S41; }

// State S48 handlers
static uint8_t handler_s48_closeconnection(void) { return S23; }
static uint8_t handler_s48_ack_psh_vv1(void)     { return S48; }
static uint8_t handler_s48_syn_ack_vv0(void)     { return S51; }
static uint8_t handler_s48_rst_vv0(void)         { return S51; }
static uint8_t handler_s48_accept(void)          { return S48; }
static uint8_t handler_s48_fin_ack_vv0(void)     { return S50; }
static uint8_t handler_s48_listen(void)          { return S48; }
static uint8_t handler_s48_syn_vv0(void)         { return S51; }
static uint8_t handler_s48_rcv(void)             { return S48; }
static uint8_t handler_s48_ack_rst_vv0(void)     { return S51; }
static uint8_t handler_s48_close(void)           { return S2; }
static uint8_t handler_s48_send(void)            { return S48; }
static uint8_t handler_s48_ack_vv0(void)         { return S48; }

// State S49 handlers
static uint8_t handler_s49_closeconnection(void) { return S22; }
static uint8_t handler_s49_ack_psh_vv1(void)     { return S41; }
static uint8_t handler_s49_syn_ack_vv0(void)     { return S49; }
static uint8_t handler_s49_rst_vv0(void)         { return S41; }
static uint8_t handler_s49_accept(void)          { return S49; }
static uint8_t handler_s49_fin_ack_vv0(void)     { return S41; }
static uint8_t handler_s49_listen(void)          { return S49; }
static uint8_t handler_s49_syn_vv0(void)         { return S46; }
static uint8_t handler_s49_rcv(void)             { return S49; }
static uint8_t handler_s49_ack_rst_vv0(void)     { return S41; }
static uint8_t handler_s49_close(void)           { return S5; }
static uint8_t handler_s49_send(void)            { return S49; }
static uint8_t handler_s49_ack_vv0(void)         { return S41; }

// State S50 handlers
static uint8_t handler_s50_closeconnection(void) { return S24; }
static uint8_t handler_s50_ack_psh_vv1(void)     { return S50; }
static uint8_t handler_s50_syn_ack_vv0(void)     { return S51; }
static uint8_t handler_s50_rst_vv0(void)         { return S51; }
static uint8_t handler_s50_accept(void)          { return S50; }
static uint8_t handler_s50_fin_ack_vv0(void)     { return S50; }
static uint8_t handler_s50_listen(void)          { return S50; }
static uint8_t handler_s50_syn_vv0(void)         { return S51; }
static uint8_t handler_s50_rcv(void)             { return S50; }
static uint8_t handler_s50_ack_rst_vv0(void)     { return S51; }
static uint8_t handler_s50_close(void)           { return S2; }
static uint8_t handler_s50_send(void)            { return S50; }
static uint8_t handler_s50_ack_vv0(void)         { return S50; }

// State S51 handlers
static uint8_t handler_s51_closeconnection(void) { return S32; }
static uint8_t handler_s51_ack_psh_vv1(void)     { return S51; }
static uint8_t handler_s51_syn_ack_vv0(void)     { return S51; }
static uint8_t handler_s51_rst_vv0(void)         { return S51; }
static uint8_t handler_s51_accept(void)          { return S51; }
static uint8_t handler_s51_fin_ack_vv0(void)     { return S51; }
static uint8_t handler_s51_listen(void)          { return S51; }
static uint8_t handler_s51_syn_vv0(void)         { return S52; }
static uint8_t handler_s51_rcv(void)             { return S51; }
static uint8_t handler_s51_ack_rst_vv0(void)     { return S51; }
static uint8_t handler_s51_close(void)           { return S2; }
static uint8_t handler_s51_send(void)            { return S51; }
static uint8_t handler_s51_ack_vv0(void)         { return S51; }

// State S52 handlers
static uint8_t handler_s52_closeconnection(void) { return S40; }
static uint8_t handler_s52_ack_psh_vv1(void)     { return S51; }
static uint8_t handler_s52_syn_ack_vv0(void)     { return S53; }
static uint8_t handler_s52_rst_vv0(void)         { return S51; }
static uint8_t handler_s52_accept(void)          { return S52; }
static uint8_t handler_s52_fin_ack_vv0(void)     { return S51; }
static uint8_t handler_s52_listen(void)          { return S52; }
static uint8_t handler_s52_syn_vv0(void)         { return S52; }
static uint8_t handler_s52_rcv(void)             { return S52; }
static uint8_t handler_s52_ack_rst_vv0(void)     { return S54; }
static uint8_t handler_s52_close(void)           { return S5; }
static uint8_t handler_s52_send(void)            { return S52; }
static uint8_t handler_s52_ack_vv0(void)         { return S51; }

// State S53 handlers
static uint8_t handler_s53_closeconnection(void) { return S44; }
static uint8_t handler_s53_ack_psh_vv1(void)     { return S51; }
static uint8_t handler_s53_syn_ack_vv0(void)     { return S53; }
static uint8_t handler_s53_rst_vv0(void)         { return S51; }
static uint8_t handler_s53_accept(void)          { return S53; }
static uint8_t handler_s53_fin_ack_vv0(void)     { return S51; }
static uint8_t handler_s53_listen(void)          { return S53; }
static uint8_t handler_s53_syn_vv0(void)         { return S52; }
static uint8_t handler_s53_rcv(void)             { return S53; }
static uint8_t handler_s53_ack_rst_vv0(void)     { return S51; }
static uint8_t handler_s53_close(void)           { return S5; }
static uint8_t handler_s53_send(void)            { return S53; }
static uint8_t handler_s53_ack_vv0(void)         { return S51; }

// State S54 handlers
static uint8_t handler_s54_closeconnection(void) { return S45; }
static uint8_t handler_s54_ack_psh_vv1(void)     { return S51; }
static uint8_t handler_s54_syn_ack_vv0(void)     { return S51; }
static uint8_t handler_s54_rst_vv0(void)         { return S54; }
static uint8_t handler_s54_accept(void)          { return S54; }
static uint8_t handler_s54_fin_ack_vv0(void)     { return S51; }
static uint8_t handler_s54_listen(void)          { return S54; }
static uint8_t handler_s54_syn_vv0(void)         { return S54; }
static uint8_t handler_s54_rcv(void)             { return S54; }
static uint8_t handler_s54_ack_rst_vv0(void)     { return S54; }
static uint8_t handler_s54_close(void)           { return S2; }
static uint8_t handler_s54_send(void)            { return S54; }
static uint8_t handler_s54_ack_vv0(void)         { return S51; }


typedef struct {
    uint8_t state;
    uint8_t event;
    EventHandler handler; // pokazivac na funkciju, ne next_state konstanta
} Transition;

static const Transition transition_table[] PROGMEM = {
    // State S0 transitions
    { S0, CLOSECONNECTION, handler_s0_closeconnection },
    { S0, ACK_PSH_VV1, handler_s0_ack_psh_vv1 },
    { S0, SYN_ACK_VV0, handler_s0_syn_ack_vv0 },
    { S0, RST_VV0, handler_s0_rst_vv0 },
    { S0, ACCEPT, handler_s0_accept },
    { S0, FIN_ACK_VV0, handler_s0_fin_ack_vv0 },
    { S0, LISTEN, handler_s0_listen },
    { S0, SYN_VV0, handler_s0_syn_vv0 },
    { S0, RCV, handler_s0_rcv },
    { S0, ACK_RST_VV0, handler_s0_ack_rst_vv0 },
    { S0, CLOSE, handler_s0_close },
    { S0, SEND, handler_s0_send },
    { S0, ACK_VV0, handler_s0_ack_vv0 },

    // State S1 transitions
    { S1, CLOSECONNECTION, handler_s1_closeconnection },
    { S1, ACK_PSH_VV1, handler_s1_ack_psh_vv1 },
    { S1, SYN_ACK_VV0, handler_s1_syn_ack_vv0 },
    { S1, RST_VV0, handler_s1_rst_vv0 },
    { S1, ACCEPT, handler_s1_accept },
    { S1, FIN_ACK_VV0, handler_s1_fin_ack_vv0 },
    { S1, LISTEN, handler_s1_listen },
    { S1, SYN_VV0, handler_s1_syn_vv0 },
    { S1, RCV, handler_s1_rcv },
    { S1, ACK_RST_VV0, handler_s1_ack_rst_vv0 },
    { S1, CLOSE, handler_s1_close },
    { S1, SEND, handler_s1_send },
    { S1, ACK_VV0, handler_s1_ack_vv0 },

    // State S2 transitions
    { S2, CLOSECONNECTION, handler_s2_closeconnection },
    { S2, ACK_PSH_VV1, handler_s2_ack_psh_vv1 },
    { S2, SYN_ACK_VV0, handler_s2_syn_ack_vv0 },
    { S2, RST_VV0, handler_s2_rst_vv0 },
    { S2, ACCEPT, handler_s2_accept },
    { S2, FIN_ACK_VV0, handler_s2_fin_ack_vv0 },
    { S2, LISTEN, handler_s2_listen },
    { S2, SYN_VV0, handler_s2_syn_vv0 },
    { S2, RCV, handler_s2_rcv },
    { S2, ACK_RST_VV0, handler_s2_ack_rst_vv0 },
    { S2, CLOSE, handler_s2_close },
    { S2, SEND, handler_s2_send },
    { S2, ACK_VV0, handler_s2_ack_vv0 },

    // State S3 transitions
    { S3, CLOSECONNECTION, handler_s3_closeconnection },
    { S3, ACK_PSH_VV1, handler_s3_ack_psh_vv1 },
    { S3, SYN_ACK_VV0, handler_s3_syn_ack_vv0 },
    { S3, RST_VV0, handler_s3_rst_vv0 },
    { S3, ACCEPT, handler_s3_accept },
    { S3, FIN_ACK_VV0, handler_s3_fin_ack_vv0 },
    { S3, LISTEN, handler_s3_listen },
    { S3, SYN_VV0, handler_s3_syn_vv0 },
    { S3, RCV, handler_s3_rcv },
    { S3, ACK_RST_VV0, handler_s3_ack_rst_vv0 },
    { S3, CLOSE, handler_s3_close },
    { S3, SEND, handler_s3_send },
    { S3, ACK_VV0, handler_s3_ack_vv0 },

    // State S4 transitions
    { S4, CLOSECONNECTION, handler_s4_closeconnection },
    { S4, ACK_PSH_VV1, handler_s4_ack_psh_vv1 },
    { S4, SYN_ACK_VV0, handler_s4_syn_ack_vv0 },
    { S4, RST_VV0, handler_s4_rst_vv0 },
    { S4, ACCEPT, handler_s4_accept },
    { S4, FIN_ACK_VV0, handler_s4_fin_ack_vv0 },
    { S4, LISTEN, handler_s4_listen },
    { S4, SYN_VV0, handler_s4_syn_vv0 },
    { S4, RCV, handler_s4_rcv },
    { S4, ACK_RST_VV0, handler_s4_ack_rst_vv0 },
    { S4, CLOSE, handler_s4_close },
    { S4, SEND, handler_s4_send },
    { S4, ACK_VV0, handler_s4_ack_vv0 },

    // State S5 transitions
    { S5, CLOSECONNECTION, handler_s5_closeconnection },
    { S5, ACK_PSH_VV1, handler_s5_ack_psh_vv1 },
    { S5, SYN_ACK_VV0, handler_s5_syn_ack_vv0 },
    { S5, RST_VV0, handler_s5_rst_vv0 },
    { S5, ACCEPT, handler_s5_accept },
    { S5, FIN_ACK_VV0, handler_s5_fin_ack_vv0 },
    { S5, LISTEN, handler_s5_listen },
    { S5, SYN_VV0, handler_s5_syn_vv0 },
    { S5, RCV, handler_s5_rcv },
    { S5, ACK_RST_VV0, handler_s5_ack_rst_vv0 },
    { S5, CLOSE, handler_s5_close },
    { S5, SEND, handler_s5_send },
    { S5, ACK_VV0, handler_s5_ack_vv0 },

    // State S6 transitions
    { S6, CLOSECONNECTION, handler_s6_closeconnection },
    { S6, ACK_PSH_VV1, handler_s6_ack_psh_vv1 },
    { S6, SYN_ACK_VV0, handler_s6_syn_ack_vv0 },
    { S6, RST_VV0, handler_s6_rst_vv0 },
    { S6, ACCEPT, handler_s6_accept },
    { S6, FIN_ACK_VV0, handler_s6_fin_ack_vv0 },
    { S6, LISTEN, handler_s6_listen },
    { S6, SYN_VV0, handler_s6_syn_vv0 },
    { S6, RCV, handler_s6_rcv },
    { S6, ACK_RST_VV0, handler_s6_ack_rst_vv0 },
    { S6, CLOSE, handler_s6_close },
    { S6, SEND, handler_s6_send },
    { S6, ACK_VV0, handler_s6_ack_vv0 },

    // State S7 transitions
    { S7, CLOSECONNECTION, handler_s7_closeconnection },
    { S7, ACK_PSH_VV1, handler_s7_ack_psh_vv1 },
    { S7, SYN_ACK_VV0, handler_s7_syn_ack_vv0 },
    { S7, RST_VV0, handler_s7_rst_vv0 },
    { S7, ACCEPT, handler_s7_accept },
    { S7, FIN_ACK_VV0, handler_s7_fin_ack_vv0 },
    { S7, LISTEN, handler_s7_listen },
    { S7, SYN_VV0, handler_s7_syn_vv0 },
    { S7, RCV, handler_s7_rcv },
    { S7, ACK_RST_VV0, handler_s7_ack_rst_vv0 },
    { S7, CLOSE, handler_s7_close },
    { S7, SEND, handler_s7_send },
    { S7, ACK_VV0, handler_s7_ack_vv0 },

    // State S8 transitions
    { S8, CLOSECONNECTION, handler_s8_closeconnection },
    { S8, ACK_PSH_VV1, handler_s8_ack_psh_vv1 },
    { S8, SYN_ACK_VV0, handler_s8_syn_ack_vv0 },
    { S8, RST_VV0, handler_s8_rst_vv0 },
    { S8, ACCEPT, handler_s8_accept },
    { S8, FIN_ACK_VV0, handler_s8_fin_ack_vv0 },
    { S8, LISTEN, handler_s8_listen },
    { S8, SYN_VV0, handler_s8_syn_vv0 },
    { S8, RCV, handler_s8_rcv },
    { S8, ACK_RST_VV0, handler_s8_ack_rst_vv0 },
    { S8, CLOSE, handler_s8_close },
    { S8, SEND, handler_s8_send },
    { S8, ACK_VV0, handler_s8_ack_vv0 },

    // State S9 transitions
    { S9, CLOSECONNECTION, handler_s9_closeconnection },
    { S9, ACK_PSH_VV1, handler_s9_ack_psh_vv1 },
    { S9, SYN_ACK_VV0, handler_s9_syn_ack_vv0 },
    { S9, RST_VV0, handler_s9_rst_vv0 },
    { S9, ACCEPT, handler_s9_accept },
    { S9, FIN_ACK_VV0, handler_s9_fin_ack_vv0 },
    { S9, LISTEN, handler_s9_listen },
    { S9, SYN_VV0, handler_s9_syn_vv0 },
    { S9, RCV, handler_s9_rcv },
    { S9, ACK_RST_VV0, handler_s9_ack_rst_vv0 },
    { S9, CLOSE, handler_s9_close },
    { S9, SEND, handler_s9_send },
    { S9, ACK_VV0, handler_s9_ack_vv0 },

    // State S10 transitions
    { S10, CLOSECONNECTION, handler_s10_closeconnection },
    { S10, ACK_PSH_VV1, handler_s10_ack_psh_vv1 },
    { S10, SYN_ACK_VV0, handler_s10_syn_ack_vv0 },
    { S10, RST_VV0, handler_s10_rst_vv0 },
    { S10, ACCEPT, handler_s10_accept },
    { S10, FIN_ACK_VV0, handler_s10_fin_ack_vv0 },
    { S10, LISTEN, handler_s10_listen },
    { S10, SYN_VV0, handler_s10_syn_vv0 },
    { S10, RCV, handler_s10_rcv },
    { S10, ACK_RST_VV0, handler_s10_ack_rst_vv0 },
    { S10, CLOSE, handler_s10_close },
    { S10, SEND, handler_s10_send },
    { S10, ACK_VV0, handler_s10_ack_vv0 },

    // State S11 transitions
    { S11, CLOSECONNECTION, handler_s11_closeconnection },
    { S11, ACK_PSH_VV1, handler_s11_ack_psh_vv1 },
    { S11, SYN_ACK_VV0, handler_s11_syn_ack_vv0 },
    { S11, RST_VV0, handler_s11_rst_vv0 },
    { S11, ACCEPT, handler_s11_accept },
    { S11, FIN_ACK_VV0, handler_s11_fin_ack_vv0 },
    { S11, LISTEN, handler_s11_listen },
    { S11, SYN_VV0, handler_s11_syn_vv0 },
    { S11, RCV, handler_s11_rcv },
    { S11, ACK_RST_VV0, handler_s11_ack_rst_vv0 },
    { S11, CLOSE, handler_s11_close },
    { S11, SEND, handler_s11_send },
    { S11, ACK_VV0, handler_s11_ack_vv0 },

    // State S12 transitions
    { S12, CLOSECONNECTION, handler_s12_closeconnection },
    { S12, ACK_PSH_VV1, handler_s12_ack_psh_vv1 },
    { S12, SYN_ACK_VV0, handler_s12_syn_ack_vv0 },
    { S12, RST_VV0, handler_s12_rst_vv0 },
    { S12, ACCEPT, handler_s12_accept },
    { S12, FIN_ACK_VV0, handler_s12_fin_ack_vv0 },
    { S12, LISTEN, handler_s12_listen },
    { S12, SYN_VV0, handler_s12_syn_vv0 },
    { S12, RCV, handler_s12_rcv },
    { S12, ACK_RST_VV0, handler_s12_ack_rst_vv0 },
    { S12, CLOSE, handler_s12_close },
    { S12, SEND, handler_s12_send },
    { S12, ACK_VV0, handler_s12_ack_vv0 },

    // State S13 transitions
    { S13, CLOSECONNECTION, handler_s13_closeconnection },
    { S13, ACK_PSH_VV1, handler_s13_ack_psh_vv1 },
    { S13, SYN_ACK_VV0, handler_s13_syn_ack_vv0 },
    { S13, RST_VV0, handler_s13_rst_vv0 },
    { S13, ACCEPT, handler_s13_accept },
    { S13, FIN_ACK_VV0, handler_s13_fin_ack_vv0 },
    { S13, LISTEN, handler_s13_listen },
    { S13, SYN_VV0, handler_s13_syn_vv0 },
    { S13, RCV, handler_s13_rcv },
    { S13, ACK_RST_VV0, handler_s13_ack_rst_vv0 },
    { S13, CLOSE, handler_s13_close },
    { S13, SEND, handler_s13_send },
    { S13, ACK_VV0, handler_s13_ack_vv0 },

    // State S14 transitions
    { S14, CLOSECONNECTION, handler_s14_closeconnection },
    { S14, ACK_PSH_VV1, handler_s14_ack_psh_vv1 },
    { S14, SYN_ACK_VV0, handler_s14_syn_ack_vv0 },
    { S14, RST_VV0, handler_s14_rst_vv0 },
    { S14, ACCEPT, handler_s14_accept },
    { S14, FIN_ACK_VV0, handler_s14_fin_ack_vv0 },
    { S14, LISTEN, handler_s14_listen },
    { S14, SYN_VV0, handler_s14_syn_vv0 },
    { S14, RCV, handler_s14_rcv },
    { S14, ACK_RST_VV0, handler_s14_ack_rst_vv0 },
    { S14, CLOSE, handler_s14_close },
    { S14, SEND, handler_s14_send },
    { S14, ACK_VV0, handler_s14_ack_vv0 },

    // State S15 transitions
    { S15, CLOSECONNECTION, handler_s15_closeconnection },
    { S15, ACK_PSH_VV1, handler_s15_ack_psh_vv1 },
    { S15, SYN_ACK_VV0, handler_s15_syn_ack_vv0 },
    { S15, RST_VV0, handler_s15_rst_vv0 },
    { S15, ACCEPT, handler_s15_accept },
    { S15, FIN_ACK_VV0, handler_s15_fin_ack_vv0 },
    { S15, LISTEN, handler_s15_listen },
    { S15, SYN_VV0, handler_s15_syn_vv0 },
    { S15, RCV, handler_s15_rcv },
    { S15, ACK_RST_VV0, handler_s15_ack_rst_vv0 },
    { S15, CLOSE, handler_s15_close },
    { S15, SEND, handler_s15_send },
    { S15, ACK_VV0, handler_s15_ack_vv0 },

    // State S16 transitions
    { S16, CLOSECONNECTION, handler_s16_closeconnection },
    { S16, ACK_PSH_VV1, handler_s16_ack_psh_vv1 },
    { S16, SYN_ACK_VV0, handler_s16_syn_ack_vv0 },
    { S16, RST_VV0, handler_s16_rst_vv0 },
    { S16, ACCEPT, handler_s16_accept },
    { S16, FIN_ACK_VV0, handler_s16_fin_ack_vv0 },
    { S16, LISTEN, handler_s16_listen },
    { S16, SYN_VV0, handler_s16_syn_vv0 },
    { S16, RCV, handler_s16_rcv },
    { S16, ACK_RST_VV0, handler_s16_ack_rst_vv0 },
    { S16, CLOSE, handler_s16_close },
    { S16, SEND, handler_s16_send },
    { S16, ACK_VV0, handler_s16_ack_vv0 },

    // State S17 transitions
    { S17, CLOSECONNECTION, handler_s17_closeconnection },
    { S17, ACK_PSH_VV1, handler_s17_ack_psh_vv1 },
    { S17, SYN_ACK_VV0, handler_s17_syn_ack_vv0 },
    { S17, RST_VV0, handler_s17_rst_vv0 },
    { S17, ACCEPT, handler_s17_accept },
    { S17, FIN_ACK_VV0, handler_s17_fin_ack_vv0 },
    { S17, LISTEN, handler_s17_listen },
    { S17, SYN_VV0, handler_s17_syn_vv0 },
    { S17, RCV, handler_s17_rcv },
    { S17, ACK_RST_VV0, handler_s17_ack_rst_vv0 },
    { S17, CLOSE, handler_s17_close },
    { S17, SEND, handler_s17_send },
    { S17, ACK_VV0, handler_s17_ack_vv0 },

    // State S18 transitions
    { S18, CLOSECONNECTION, handler_s18_closeconnection },
    { S18, ACK_PSH_VV1, handler_s18_ack_psh_vv1 },
    { S18, SYN_ACK_VV0, handler_s18_syn_ack_vv0 },
    { S18, RST_VV0, handler_s18_rst_vv0 },
    { S18, ACCEPT, handler_s18_accept },
    { S18, FIN_ACK_VV0, handler_s18_fin_ack_vv0 },
    { S18, LISTEN, handler_s18_listen },
    { S18, SYN_VV0, handler_s18_syn_vv0 },
    { S18, RCV, handler_s18_rcv },
    { S18, ACK_RST_VV0, handler_s18_ack_rst_vv0 },
    { S18, CLOSE, handler_s18_close },
    { S18, SEND, handler_s18_send },
    { S18, ACK_VV0, handler_s18_ack_vv0 },

    // State S19 transitions
    { S19, CLOSECONNECTION, handler_s19_closeconnection },
    { S19, ACK_PSH_VV1, handler_s19_ack_psh_vv1 },
    { S19, SYN_ACK_VV0, handler_s19_syn_ack_vv0 },
    { S19, RST_VV0, handler_s19_rst_vv0 },
    { S19, ACCEPT, handler_s19_accept },
    { S19, FIN_ACK_VV0, handler_s19_fin_ack_vv0 },
    { S19, LISTEN, handler_s19_listen },
    { S19, SYN_VV0, handler_s19_syn_vv0 },
    { S19, RCV, handler_s19_rcv },
    { S19, ACK_RST_VV0, handler_s19_ack_rst_vv0 },
    { S19, CLOSE, handler_s19_close },
    { S19, SEND, handler_s19_send },
    { S19, ACK_VV0, handler_s19_ack_vv0 },

    // State S20 transitions
    { S20, CLOSECONNECTION, handler_s20_closeconnection },
    { S20, ACK_PSH_VV1, handler_s20_ack_psh_vv1 },
    { S20, SYN_ACK_VV0, handler_s20_syn_ack_vv0 },
    { S20, RST_VV0, handler_s20_rst_vv0 },
    { S20, ACCEPT, handler_s20_accept },
    { S20, FIN_ACK_VV0, handler_s20_fin_ack_vv0 },
    { S20, LISTEN, handler_s20_listen },
    { S20, SYN_VV0, handler_s20_syn_vv0 },
    { S20, RCV, handler_s20_rcv },
    { S20, ACK_RST_VV0, handler_s20_ack_rst_vv0 },
    { S20, CLOSE, handler_s20_close },
    { S20, SEND, handler_s20_send },
    { S20, ACK_VV0, handler_s20_ack_vv0 },

    // State S21 transitions
    { S21, CLOSECONNECTION, handler_s21_closeconnection },
    { S21, ACK_PSH_VV1, handler_s21_ack_psh_vv1 },
    { S21, SYN_ACK_VV0, handler_s21_syn_ack_vv0 },
    { S21, RST_VV0, handler_s21_rst_vv0 },
    { S21, ACCEPT, handler_s21_accept },
    { S21, FIN_ACK_VV0, handler_s21_fin_ack_vv0 },
    { S21, LISTEN, handler_s21_listen },
    { S21, SYN_VV0, handler_s21_syn_vv0 },
    { S21, RCV, handler_s21_rcv },
    { S21, ACK_RST_VV0, handler_s21_ack_rst_vv0 },
    { S21, CLOSE, handler_s21_close },
    { S21, SEND, handler_s21_send },
    { S21, ACK_VV0, handler_s21_ack_vv0 },

    // State S22 transitions
    { S22, CLOSECONNECTION, handler_s22_closeconnection },
    { S22, ACK_PSH_VV1, handler_s22_ack_psh_vv1 },
    { S22, SYN_ACK_VV0, handler_s22_syn_ack_vv0 },
    { S22, RST_VV0, handler_s22_rst_vv0 },
    { S22, ACCEPT, handler_s22_accept },
    { S22, FIN_ACK_VV0, handler_s22_fin_ack_vv0 },
    { S22, LISTEN, handler_s22_listen },
    { S22, SYN_VV0, handler_s22_syn_vv0 },
    { S22, RCV, handler_s22_rcv },
    { S22, ACK_RST_VV0, handler_s22_ack_rst_vv0 },
    { S22, CLOSE, handler_s22_close },
    { S22, SEND, handler_s22_send },
    { S22, ACK_VV0, handler_s22_ack_vv0 },

    // State S23 transitions
    { S23, CLOSECONNECTION, handler_s23_closeconnection },
    { S23, ACK_PSH_VV1, handler_s23_ack_psh_vv1 },
    { S23, SYN_ACK_VV0, handler_s23_syn_ack_vv0 },
    { S23, RST_VV0, handler_s23_rst_vv0 },
    { S23, ACCEPT, handler_s23_accept },
    { S23, FIN_ACK_VV0, handler_s23_fin_ack_vv0 },
    { S23, LISTEN, handler_s23_listen },
    { S23, SYN_VV0, handler_s23_syn_vv0 },
    { S23, RCV, handler_s23_rcv },
    { S23, ACK_RST_VV0, handler_s23_ack_rst_vv0 },
    { S23, CLOSE, handler_s23_close },
    { S23, SEND, handler_s23_send },
    { S23, ACK_VV0, handler_s23_ack_vv0 },

    // State S24 transitions
    { S24, CLOSECONNECTION, handler_s24_closeconnection },
    { S24, ACK_PSH_VV1, handler_s24_ack_psh_vv1 },
    { S24, SYN_ACK_VV0, handler_s24_syn_ack_vv0 },
    { S24, RST_VV0, handler_s24_rst_vv0 },
    { S24, ACCEPT, handler_s24_accept },
    { S24, FIN_ACK_VV0, handler_s24_fin_ack_vv0 },
    { S24, LISTEN, handler_s24_listen },
    { S24, SYN_VV0, handler_s24_syn_vv0 },
    { S24, RCV, handler_s24_rcv },
    { S24, ACK_RST_VV0, handler_s24_ack_rst_vv0 },
    { S24, CLOSE, handler_s24_close },
    { S24, SEND, handler_s24_send },
    { S24, ACK_VV0, handler_s24_ack_vv0 },

    // State S25 transitions
    { S25, CLOSECONNECTION, handler_s25_closeconnection },
    { S25, ACK_PSH_VV1, handler_s25_ack_psh_vv1 },
    { S25, SYN_ACK_VV0, handler_s25_syn_ack_vv0 },
    { S25, RST_VV0, handler_s25_rst_vv0 },
    { S25, ACCEPT, handler_s25_accept },
    { S25, FIN_ACK_VV0, handler_s25_fin_ack_vv0 },
    { S25, LISTEN, handler_s25_listen },
    { S25, SYN_VV0, handler_s25_syn_vv0 },
    { S25, RCV, handler_s25_rcv },
    { S25, ACK_RST_VV0, handler_s25_ack_rst_vv0 },
    { S25, CLOSE, handler_s25_close },
    { S25, SEND, handler_s25_send },
    { S25, ACK_VV0, handler_s25_ack_vv0 },

    // State S26 transitions
    { S26, CLOSECONNECTION, handler_s26_closeconnection },
    { S26, ACK_PSH_VV1, handler_s26_ack_psh_vv1 },
    { S26, SYN_ACK_VV0, handler_s26_syn_ack_vv0 },
    { S26, RST_VV0, handler_s26_rst_vv0 },
    { S26, ACCEPT, handler_s26_accept },
    { S26, FIN_ACK_VV0, handler_s26_fin_ack_vv0 },
    { S26, LISTEN, handler_s26_listen },
    { S26, SYN_VV0, handler_s26_syn_vv0 },
    { S26, RCV, handler_s26_rcv },
    { S26, ACK_RST_VV0, handler_s26_ack_rst_vv0 },
    { S26, CLOSE, handler_s26_close },
    { S26, SEND, handler_s26_send },
    { S26, ACK_VV0, handler_s26_ack_vv0 },

    // State S27 transitions
    { S27, CLOSECONNECTION, handler_s27_closeconnection },
    { S27, ACK_PSH_VV1, handler_s27_ack_psh_vv1 },
    { S27, SYN_ACK_VV0, handler_s27_syn_ack_vv0 },
    { S27, RST_VV0, handler_s27_rst_vv0 },
    { S27, ACCEPT, handler_s27_accept },
    { S27, FIN_ACK_VV0, handler_s27_fin_ack_vv0 },
    { S27, LISTEN, handler_s27_listen },
    { S27, SYN_VV0, handler_s27_syn_vv0 },
    { S27, RCV, handler_s27_rcv },
    { S27, ACK_RST_VV0, handler_s27_ack_rst_vv0 },
    { S27, CLOSE, handler_s27_close },
    { S27, SEND, handler_s27_send },
    { S27, ACK_VV0, handler_s27_ack_vv0 },

    // State S28 transitions
    { S28, CLOSECONNECTION, handler_s28_closeconnection },
    { S28, ACK_PSH_VV1, handler_s28_ack_psh_vv1 },
    { S28, SYN_ACK_VV0, handler_s28_syn_ack_vv0 },
    { S28, RST_VV0, handler_s28_rst_vv0 },
    { S28, ACCEPT, handler_s28_accept },
    { S28, FIN_ACK_VV0, handler_s28_fin_ack_vv0 },
    { S28, LISTEN, handler_s28_listen },
    { S28, SYN_VV0, handler_s28_syn_vv0 },
    { S28, RCV, handler_s28_rcv },
    { S28, ACK_RST_VV0, handler_s28_ack_rst_vv0 },
    { S28, CLOSE, handler_s28_close },
    { S28, SEND, handler_s28_send },
    { S28, ACK_VV0, handler_s28_ack_vv0 },

    // State S29 transitions
    { S29, CLOSECONNECTION, handler_s29_closeconnection },
    { S29, ACK_PSH_VV1, handler_s29_ack_psh_vv1 },
    { S29, SYN_ACK_VV0, handler_s29_syn_ack_vv0 },
    { S29, RST_VV0, handler_s29_rst_vv0 },
    { S29, ACCEPT, handler_s29_accept },
    { S29, FIN_ACK_VV0, handler_s29_fin_ack_vv0 },
    { S29, LISTEN, handler_s29_listen },
    { S29, SYN_VV0, handler_s29_syn_vv0 },
    { S29, RCV, handler_s29_rcv },
    { S29, ACK_RST_VV0, handler_s29_ack_rst_vv0 },
    { S29, CLOSE, handler_s29_close },
    { S29, SEND, handler_s29_send },
    { S29, ACK_VV0, handler_s29_ack_vv0 },

    // State S30 transitions
    { S30, CLOSECONNECTION, handler_s30_closeconnection },
    { S30, ACK_PSH_VV1, handler_s30_ack_psh_vv1 },
    { S30, SYN_ACK_VV0, handler_s30_syn_ack_vv0 },
    { S30, RST_VV0, handler_s30_rst_vv0 },
    { S30, ACCEPT, handler_s30_accept },
    { S30, FIN_ACK_VV0, handler_s30_fin_ack_vv0 },
    { S30, LISTEN, handler_s30_listen },
    { S30, SYN_VV0, handler_s30_syn_vv0 },
    { S30, RCV, handler_s30_rcv },
    { S30, ACK_RST_VV0, handler_s30_ack_rst_vv0 },
    { S30, CLOSE, handler_s30_close },
    { S30, SEND, handler_s30_send },
    { S30, ACK_VV0, handler_s30_ack_vv0 },

    // State S31 transitions
    { S31, CLOSECONNECTION, handler_s31_closeconnection },
    { S31, ACK_PSH_VV1, handler_s31_ack_psh_vv1 },
    { S31, SYN_ACK_VV0, handler_s31_syn_ack_vv0 },
    { S31, RST_VV0, handler_s31_rst_vv0 },
    { S31, ACCEPT, handler_s31_accept },
    { S31, FIN_ACK_VV0, handler_s31_fin_ack_vv0 },
    { S31, LISTEN, handler_s31_listen },
    { S31, SYN_VV0, handler_s31_syn_vv0 },
    { S31, RCV, handler_s31_rcv },
    { S31, ACK_RST_VV0, handler_s31_ack_rst_vv0 },
    { S31, CLOSE, handler_s31_close },
    { S31, SEND, handler_s31_send },
    { S31, ACK_VV0, handler_s31_ack_vv0 },

    // State S32 transitions
    { S32, CLOSECONNECTION, handler_s32_closeconnection },
    { S32, ACK_PSH_VV1, handler_s32_ack_psh_vv1 },
    { S32, SYN_ACK_VV0, handler_s32_syn_ack_vv0 },
    { S32, RST_VV0, handler_s32_rst_vv0 },
    { S32, ACCEPT, handler_s32_accept },
    { S32, FIN_ACK_VV0, handler_s32_fin_ack_vv0 },
    { S32, LISTEN, handler_s32_listen },
    { S32, SYN_VV0, handler_s32_syn_vv0 },
    { S32, RCV, handler_s32_rcv },
    { S32, ACK_RST_VV0, handler_s32_ack_rst_vv0 },
    { S32, CLOSE, handler_s32_close },
    { S32, SEND, handler_s32_send },
    { S32, ACK_VV0, handler_s32_ack_vv0 },

    // State S33 transitions
    { S33, CLOSECONNECTION, handler_s33_closeconnection },
    { S33, ACK_PSH_VV1, handler_s33_ack_psh_vv1 },
    { S33, SYN_ACK_VV0, handler_s33_syn_ack_vv0 },
    { S33, RST_VV0, handler_s33_rst_vv0 },
    { S33, ACCEPT, handler_s33_accept },
    { S33, FIN_ACK_VV0, handler_s33_fin_ack_vv0 },
    { S33, LISTEN, handler_s33_listen },
    { S33, SYN_VV0, handler_s33_syn_vv0 },
    { S33, RCV, handler_s33_rcv },
    { S33, ACK_RST_VV0, handler_s33_ack_rst_vv0 },
    { S33, CLOSE, handler_s33_close },
    { S33, SEND, handler_s33_send },
    { S33, ACK_VV0, handler_s33_ack_vv0 },

    // State S34 transitions
    { S34, CLOSECONNECTION, handler_s34_closeconnection },
    { S34, ACK_PSH_VV1, handler_s34_ack_psh_vv1 },
    { S34, SYN_ACK_VV0, handler_s34_syn_ack_vv0 },
    { S34, RST_VV0, handler_s34_rst_vv0 },
    { S34, ACCEPT, handler_s34_accept },
    { S34, FIN_ACK_VV0, handler_s34_fin_ack_vv0 },
    { S34, LISTEN, handler_s34_listen },
    { S34, SYN_VV0, handler_s34_syn_vv0 },
    { S34, RCV, handler_s34_rcv },
    { S34, ACK_RST_VV0, handler_s34_ack_rst_vv0 },
    { S34, CLOSE, handler_s34_close },
    { S34, SEND, handler_s34_send },
    { S34, ACK_VV0, handler_s34_ack_vv0 },

    // State S35 transitions
    { S35, CLOSECONNECTION, handler_s35_closeconnection },
    { S35, ACK_PSH_VV1, handler_s35_ack_psh_vv1 },
    { S35, SYN_ACK_VV0, handler_s35_syn_ack_vv0 },
    { S35, RST_VV0, handler_s35_rst_vv0 },
    { S35, ACCEPT, handler_s35_accept },
    { S35, FIN_ACK_VV0, handler_s35_fin_ack_vv0 },
    { S35, LISTEN, handler_s35_listen },
    { S35, SYN_VV0, handler_s35_syn_vv0 },
    { S35, RCV, handler_s35_rcv },
    { S35, ACK_RST_VV0, handler_s35_ack_rst_vv0 },
    { S35, CLOSE, handler_s35_close },
    { S35, SEND, handler_s35_send },
    { S35, ACK_VV0, handler_s35_ack_vv0 },

    // State S36 transitions
    { S36, CLOSECONNECTION, handler_s36_closeconnection },
    { S36, ACK_PSH_VV1, handler_s36_ack_psh_vv1 },
    { S36, SYN_ACK_VV0, handler_s36_syn_ack_vv0 },
    { S36, RST_VV0, handler_s36_rst_vv0 },
    { S36, ACCEPT, handler_s36_accept },
    { S36, FIN_ACK_VV0, handler_s36_fin_ack_vv0 },
    { S36, LISTEN, handler_s36_listen },
    { S36, SYN_VV0, handler_s36_syn_vv0 },
    { S36, RCV, handler_s36_rcv },
    { S36, ACK_RST_VV0, handler_s36_ack_rst_vv0 },
    { S36, CLOSE, handler_s36_close },
    { S36, SEND, handler_s36_send },
    { S36, ACK_VV0, handler_s36_ack_vv0 },

    // State S37 transitions
    { S37, CLOSECONNECTION, handler_s37_closeconnection },
    { S37, ACK_PSH_VV1, handler_s37_ack_psh_vv1 },
    { S37, SYN_ACK_VV0, handler_s37_syn_ack_vv0 },
    { S37, RST_VV0, handler_s37_rst_vv0 },
    { S37, ACCEPT, handler_s37_accept },
    { S37, FIN_ACK_VV0, handler_s37_fin_ack_vv0 },
    { S37, LISTEN, handler_s37_listen },
    { S37, SYN_VV0, handler_s37_syn_vv0 },
    { S37, RCV, handler_s37_rcv },
    { S37, ACK_RST_VV0, handler_s37_ack_rst_vv0 },
    { S37, CLOSE, handler_s37_close },
    { S37, SEND, handler_s37_send },
    { S37, ACK_VV0, handler_s37_ack_vv0 },

    // State S38 transitions
    { S38, CLOSECONNECTION, handler_s38_closeconnection },
    { S38, ACK_PSH_VV1, handler_s38_ack_psh_vv1 },
    { S38, SYN_ACK_VV0, handler_s38_syn_ack_vv0 },
    { S38, RST_VV0, handler_s38_rst_vv0 },
    { S38, ACCEPT, handler_s38_accept },
    { S38, FIN_ACK_VV0, handler_s38_fin_ack_vv0 },
    { S38, LISTEN, handler_s38_listen },
    { S38, SYN_VV0, handler_s38_syn_vv0 },
    { S38, RCV, handler_s38_rcv },
    { S38, ACK_RST_VV0, handler_s38_ack_rst_vv0 },
    { S38, CLOSE, handler_s38_close },
    { S38, SEND, handler_s38_send },
    { S38, ACK_VV0, handler_s38_ack_vv0 },

    // State S39 transitions
    { S39, CLOSECONNECTION, handler_s39_closeconnection },
    { S39, ACK_PSH_VV1, handler_s39_ack_psh_vv1 },
    { S39, SYN_ACK_VV0, handler_s39_syn_ack_vv0 },
    { S39, RST_VV0, handler_s39_rst_vv0 },
    { S39, ACCEPT, handler_s39_accept },
    { S39, FIN_ACK_VV0, handler_s39_fin_ack_vv0 },
    { S39, LISTEN, handler_s39_listen },
    { S39, SYN_VV0, handler_s39_syn_vv0 },
    { S39, RCV, handler_s39_rcv },
    { S39, ACK_RST_VV0, handler_s39_ack_rst_vv0 },
    { S39, CLOSE, handler_s39_close },
    { S39, SEND, handler_s39_send },
    { S39, ACK_VV0, handler_s39_ack_vv0 },

    // State S40 transitions
    { S40, CLOSECONNECTION, handler_s40_closeconnection },
    { S40, ACK_PSH_VV1, handler_s40_ack_psh_vv1 },
    { S40, SYN_ACK_VV0, handler_s40_syn_ack_vv0 },
    { S40, RST_VV0, handler_s40_rst_vv0 },
    { S40, ACCEPT, handler_s40_accept },
    { S40, FIN_ACK_VV0, handler_s40_fin_ack_vv0 },
    { S40, LISTEN, handler_s40_listen },
    { S40, SYN_VV0, handler_s40_syn_vv0 },
    { S40, RCV, handler_s40_rcv },
    { S40, ACK_RST_VV0, handler_s40_ack_rst_vv0 },
    { S40, CLOSE, handler_s40_close },
    { S40, SEND, handler_s40_send },
    { S40, ACK_VV0, handler_s40_ack_vv0 },

    // State S41 transitions
    { S41, CLOSECONNECTION, handler_s41_closeconnection },
    { S41, ACK_PSH_VV1, handler_s41_ack_psh_vv1 },
    { S41, SYN_ACK_VV0, handler_s41_syn_ack_vv0 },
    { S41, RST_VV0, handler_s41_rst_vv0 },
    { S41, ACCEPT, handler_s41_accept },
    { S41, FIN_ACK_VV0, handler_s41_fin_ack_vv0 },
    { S41, LISTEN, handler_s41_listen },
    { S41, SYN_VV0, handler_s41_syn_vv0 },
    { S41, RCV, handler_s41_rcv },
    { S41, ACK_RST_VV0, handler_s41_ack_rst_vv0 },
    { S41, CLOSE, handler_s41_close },
    { S41, SEND, handler_s41_send },
    { S41, ACK_VV0, handler_s41_ack_vv0 },

    // State S42 transitions
    { S42, CLOSECONNECTION, handler_s42_closeconnection },
    { S42, ACK_PSH_VV1, handler_s42_ack_psh_vv1 },
    { S42, SYN_ACK_VV0, handler_s42_syn_ack_vv0 },
    { S42, RST_VV0, handler_s42_rst_vv0 },
    { S42, ACCEPT, handler_s42_accept },
    { S42, FIN_ACK_VV0, handler_s42_fin_ack_vv0 },
    { S42, LISTEN, handler_s42_listen },
    { S42, SYN_VV0, handler_s42_syn_vv0 },
    { S42, RCV, handler_s42_rcv },
    { S42, ACK_RST_VV0, handler_s42_ack_rst_vv0 },
    { S42, CLOSE, handler_s42_close },
    { S42, SEND, handler_s42_send },
    { S42, ACK_VV0, handler_s42_ack_vv0 },

    // State S43 transitions
    { S43, CLOSECONNECTION, handler_s43_closeconnection },
    { S43, ACK_PSH_VV1, handler_s43_ack_psh_vv1 },
    { S43, SYN_ACK_VV0, handler_s43_syn_ack_vv0 },
    { S43, RST_VV0, handler_s43_rst_vv0 },
    { S43, ACCEPT, handler_s43_accept },
    { S43, FIN_ACK_VV0, handler_s43_fin_ack_vv0 },
    { S43, LISTEN, handler_s43_listen },
    { S43, SYN_VV0, handler_s43_syn_vv0 },
    { S43, RCV, handler_s43_rcv },
    { S43, ACK_RST_VV0, handler_s43_ack_rst_vv0 },
    { S43, CLOSE, handler_s43_close },
    { S43, SEND, handler_s43_send },
    { S43, ACK_VV0, handler_s43_ack_vv0 },

    // State S44 transitions
    { S44, CLOSECONNECTION, handler_s44_closeconnection },
    { S44, ACK_PSH_VV1, handler_s44_ack_psh_vv1 },
    { S44, SYN_ACK_VV0, handler_s44_syn_ack_vv0 },
    { S44, RST_VV0, handler_s44_rst_vv0 },
    { S44, ACCEPT, handler_s44_accept },
    { S44, FIN_ACK_VV0, handler_s44_fin_ack_vv0 },
    { S44, LISTEN, handler_s44_listen },
    { S44, SYN_VV0, handler_s44_syn_vv0 },
    { S44, RCV, handler_s44_rcv },
    { S44, ACK_RST_VV0, handler_s44_ack_rst_vv0 },
    { S44, CLOSE, handler_s44_close },
    { S44, SEND, handler_s44_send },
    { S44, ACK_VV0, handler_s44_ack_vv0 },

    // State S45 transitions
    { S45, CLOSECONNECTION, handler_s45_closeconnection },
    { S45, ACK_PSH_VV1, handler_s45_ack_psh_vv1 },
    { S45, SYN_ACK_VV0, handler_s45_syn_ack_vv0 },
    { S45, RST_VV0, handler_s45_rst_vv0 },
    { S45, ACCEPT, handler_s45_accept },
    { S45, FIN_ACK_VV0, handler_s45_fin_ack_vv0 },
    { S45, LISTEN, handler_s45_listen },
    { S45, SYN_VV0, handler_s45_syn_vv0 },
    { S45, RCV, handler_s45_rcv },
    { S45, ACK_RST_VV0, handler_s45_ack_rst_vv0 },
    { S45, CLOSE, handler_s45_close },
    { S45, SEND, handler_s45_send },
    { S45, ACK_VV0, handler_s45_ack_vv0 },

    // State S46 transitions
    { S46, CLOSECONNECTION, handler_s46_closeconnection },
    { S46, ACK_PSH_VV1, handler_s46_ack_psh_vv1 },
    { S46, SYN_ACK_VV0, handler_s46_syn_ack_vv0 },
    { S46, RST_VV0, handler_s46_rst_vv0 },
    { S46, ACCEPT, handler_s46_accept },
    { S46, FIN_ACK_VV0, handler_s46_fin_ack_vv0 },
    { S46, LISTEN, handler_s46_listen },
    { S46, SYN_VV0, handler_s46_syn_vv0 },
    { S46, RCV, handler_s46_rcv },
    { S46, ACK_RST_VV0, handler_s46_ack_rst_vv0 },
    { S46, CLOSE, handler_s46_close },
    { S46, SEND, handler_s46_send },
    { S46, ACK_VV0, handler_s46_ack_vv0 },

    // State S47 transitions
    { S47, CLOSECONNECTION, handler_s47_closeconnection },
    { S47, ACK_PSH_VV1, handler_s47_ack_psh_vv1 },
    { S47, SYN_ACK_VV0, handler_s47_syn_ack_vv0 },
    { S47, RST_VV0, handler_s47_rst_vv0 },
    { S47, ACCEPT, handler_s47_accept },
    { S47, FIN_ACK_VV0, handler_s47_fin_ack_vv0 },
    { S47, LISTEN, handler_s47_listen },
    { S47, SYN_VV0, handler_s47_syn_vv0 },
    { S47, RCV, handler_s47_rcv },
    { S47, ACK_RST_VV0, handler_s47_ack_rst_vv0 },
    { S47, CLOSE, handler_s47_close },
    { S47, SEND, handler_s47_send },
    { S47, ACK_VV0, handler_s47_ack_vv0 },

    // State S48 transitions
    { S48, CLOSECONNECTION, handler_s48_closeconnection },
    { S48, ACK_PSH_VV1, handler_s48_ack_psh_vv1 },
    { S48, SYN_ACK_VV0, handler_s48_syn_ack_vv0 },
    { S48, RST_VV0, handler_s48_rst_vv0 },
    { S48, ACCEPT, handler_s48_accept },
    { S48, FIN_ACK_VV0, handler_s48_fin_ack_vv0 },
    { S48, LISTEN, handler_s48_listen },
    { S48, SYN_VV0, handler_s48_syn_vv0 },
    { S48, RCV, handler_s48_rcv },
    { S48, ACK_RST_VV0, handler_s48_ack_rst_vv0 },
    { S48, CLOSE, handler_s48_close },
    { S48, SEND, handler_s48_send },
    { S48, ACK_VV0, handler_s48_ack_vv0 },

    // State S49 transitions
    { S49, CLOSECONNECTION, handler_s49_closeconnection },
    { S49, ACK_PSH_VV1, handler_s49_ack_psh_vv1 },
    { S49, SYN_ACK_VV0, handler_s49_syn_ack_vv0 },
    { S49, RST_VV0, handler_s49_rst_vv0 },
    { S49, ACCEPT, handler_s49_accept },
    { S49, FIN_ACK_VV0, handler_s49_fin_ack_vv0 },
    { S49, LISTEN, handler_s49_listen },
    { S49, SYN_VV0, handler_s49_syn_vv0 },
    { S49, RCV, handler_s49_rcv },
    { S49, ACK_RST_VV0, handler_s49_ack_rst_vv0 },
    { S49, CLOSE, handler_s49_close },
    { S49, SEND, handler_s49_send },
    { S49, ACK_VV0, handler_s49_ack_vv0 },

    // State S50 transitions
    { S50, CLOSECONNECTION, handler_s50_closeconnection },
    { S50, ACK_PSH_VV1, handler_s50_ack_psh_vv1 },
    { S50, SYN_ACK_VV0, handler_s50_syn_ack_vv0 },
    { S50, RST_VV0, handler_s50_rst_vv0 },
    { S50, ACCEPT, handler_s50_accept },
    { S50, FIN_ACK_VV0, handler_s50_fin_ack_vv0 },
    { S50, LISTEN, handler_s50_listen },
    { S50, SYN_VV0, handler_s50_syn_vv0 },
    { S50, RCV, handler_s50_rcv },
    { S50, ACK_RST_VV0, handler_s50_ack_rst_vv0 },
    { S50, CLOSE, handler_s50_close },
    { S50, SEND, handler_s50_send },
    { S50, ACK_VV0, handler_s50_ack_vv0 },

    // State S51 transitions
    { S51, CLOSECONNECTION, handler_s51_closeconnection },
    { S51, ACK_PSH_VV1, handler_s51_ack_psh_vv1 },
    { S51, SYN_ACK_VV0, handler_s51_syn_ack_vv0 },
    { S51, RST_VV0, handler_s51_rst_vv0 },
    { S51, ACCEPT, handler_s51_accept },
    { S51, FIN_ACK_VV0, handler_s51_fin_ack_vv0 },
    { S51, LISTEN, handler_s51_listen },
    { S51, SYN_VV0, handler_s51_syn_vv0 },
    { S51, RCV, handler_s51_rcv },
    { S51, ACK_RST_VV0, handler_s51_ack_rst_vv0 },
    { S51, CLOSE, handler_s51_close },
    { S51, SEND, handler_s51_send },
    { S51, ACK_VV0, handler_s51_ack_vv0 },

    // State S52 transitions
    { S52, CLOSECONNECTION, handler_s52_closeconnection },
    { S52, ACK_PSH_VV1, handler_s52_ack_psh_vv1 },
    { S52, SYN_ACK_VV0, handler_s52_syn_ack_vv0 },
    { S52, RST_VV0, handler_s52_rst_vv0 },
    { S52, ACCEPT, handler_s52_accept },
    { S52, FIN_ACK_VV0, handler_s52_fin_ack_vv0 },
    { S52, LISTEN, handler_s52_listen },
    { S52, SYN_VV0, handler_s52_syn_vv0 },
    { S52, RCV, handler_s52_rcv },
    { S52, ACK_RST_VV0, handler_s52_ack_rst_vv0 },
    { S52, CLOSE, handler_s52_close },
    { S52, SEND, handler_s52_send },
    { S52, ACK_VV0, handler_s52_ack_vv0 },

    // State S53 transitions
    { S53, CLOSECONNECTION, handler_s53_closeconnection },
    { S53, ACK_PSH_VV1, handler_s53_ack_psh_vv1 },
    { S53, SYN_ACK_VV0, handler_s53_syn_ack_vv0 },
    { S53, RST_VV0, handler_s53_rst_vv0 },
    { S53, ACCEPT, handler_s53_accept },
    { S53, FIN_ACK_VV0, handler_s53_fin_ack_vv0 },
    { S53, LISTEN, handler_s53_listen },
    { S53, SYN_VV0, handler_s53_syn_vv0 },
    { S53, RCV, handler_s53_rcv },
    { S53, ACK_RST_VV0, handler_s53_ack_rst_vv0 },
    { S53, CLOSE, handler_s53_close },
    { S53, SEND, handler_s53_send },
    { S53, ACK_VV0, handler_s53_ack_vv0 },

    // State S54 transitions
    { S54, CLOSECONNECTION, handler_s54_closeconnection },
    { S54, ACK_PSH_VV1, handler_s54_ack_psh_vv1 },
    { S54, SYN_ACK_VV0, handler_s54_syn_ack_vv0 },
    { S54, RST_VV0, handler_s54_rst_vv0 },
    { S54, ACCEPT, handler_s54_accept },
    { S54, FIN_ACK_VV0, handler_s54_fin_ack_vv0 },
    { S54, LISTEN, handler_s54_listen },
    { S54, SYN_VV0, handler_s54_syn_vv0 },
    { S54, RCV, handler_s54_rcv },
    { S54, ACK_RST_VV0, handler_s54_ack_rst_vv0 },
    { S54, CLOSE, handler_s54_close },
    { S54, SEND, handler_s54_send },
    { S54, ACK_VV0, handler_s54_ack_vv0 }
};

#define TABLE_SIZE (sizeof(transition_table) / sizeof(Transition))

static uint8_t fsm_transition(uint8_t state, uint8_t event) {
    uint8_t t_event, t_state;
    EventHandler t_handler;
    for (uint8_t i = 0; i < TABLE_SIZE; i++) {
        t_event = pgm_read_byte(&transition_table[i].event);
        t_state = pgm_read_byte(&transition_table[i].state);
        if (t_state == state && t_event == event) {
            t_handler = (EventHandler) pgm_read_word(&transition_table[i].handler);
            return t_handler(); // indirektni poziv - dodatni trosak
        }
    }
    return state; // no match -> stay in current state
}

static uint16_t measured_transition(uint8_t event) {
    cli();
    cycles_start();
    uint8_t next = fsm_transition(current_state, event);
    uint16_t cycles = cycles_stop();
    sei();
    current_state = next;
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
    uart_puts("\r\nMQTT FSM - Nested Switch pattern ready.\r\n");
    uart_puts("Commands: num(dec)=STATE_NUM b=BENCHMARK\r\n");
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

        if (event < 0 && event >= NUM_EVENTS)
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

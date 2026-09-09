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
#define NUM_STATES 55

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

// 2D niz POKAZIVACA NA FUNKCIJE - svaka (state,event) kombinacija je popunjena
static const EventHandler transition_table[NUM_STATES][NUM_EVENTS] PROGMEM = {
    /* S0  */ { handler_s0_closeconnection, handler_s0_ack_psh_vv1, handler_s0_syn_ack_vv0, handler_s0_rst_vv0, handler_s0_accept, handler_s0_fin_ack_vv0, handler_s0_listen, handler_s0_syn_vv0, handler_s0_rcv, handler_s0_ack_rst_vv0, handler_s0_close, handler_s0_send, handler_s0_ack_vv0 },
    /* S1  */ { handler_s1_closeconnection, handler_s1_ack_psh_vv1, handler_s1_syn_ack_vv0, handler_s1_rst_vv0, handler_s1_accept, handler_s1_fin_ack_vv0, handler_s1_listen, handler_s1_syn_vv0, handler_s1_rcv, handler_s1_ack_rst_vv0, handler_s1_close, handler_s1_send, handler_s1_ack_vv0 },
    /* S2  */ { handler_s2_closeconnection, handler_s2_ack_psh_vv1, handler_s2_syn_ack_vv0, handler_s2_rst_vv0, handler_s2_accept, handler_s2_fin_ack_vv0, handler_s2_listen, handler_s2_syn_vv0, handler_s2_rcv, handler_s2_ack_rst_vv0, handler_s2_close, handler_s2_send, handler_s2_ack_vv0 },
    /* S3  */ { handler_s3_closeconnection, handler_s3_ack_psh_vv1, handler_s3_syn_ack_vv0, handler_s3_rst_vv0, handler_s3_accept, handler_s3_fin_ack_vv0, handler_s3_listen, handler_s3_syn_vv0, handler_s3_rcv, handler_s3_ack_rst_vv0, handler_s3_close, handler_s3_send, handler_s3_ack_vv0 },
    /* S4  */ { handler_s4_closeconnection, handler_s4_ack_psh_vv1, handler_s4_syn_ack_vv0, handler_s4_rst_vv0, handler_s4_accept, handler_s4_fin_ack_vv0, handler_s4_listen, handler_s4_syn_vv0, handler_s4_rcv, handler_s4_ack_rst_vv0, handler_s4_close, handler_s4_send, handler_s4_ack_vv0 },
    /* S5  */ { handler_s5_closeconnection, handler_s5_ack_psh_vv1, handler_s5_syn_ack_vv0, handler_s5_rst_vv0, handler_s5_accept, handler_s5_fin_ack_vv0, handler_s5_listen, handler_s5_syn_vv0, handler_s5_rcv, handler_s5_ack_rst_vv0, handler_s5_close, handler_s5_send, handler_s5_ack_vv0 },
    /* S6  */ { handler_s6_closeconnection, handler_s6_ack_psh_vv1, handler_s6_syn_ack_vv0, handler_s6_rst_vv0, handler_s6_accept, handler_s6_fin_ack_vv0, handler_s6_listen, handler_s6_syn_vv0, handler_s6_rcv, handler_s6_ack_rst_vv0, handler_s6_close, handler_s6_send, handler_s6_ack_vv0 },
    /* S7  */ { handler_s7_closeconnection, handler_s7_ack_psh_vv1, handler_s7_syn_ack_vv0, handler_s7_rst_vv0, handler_s7_accept, handler_s7_fin_ack_vv0, handler_s7_listen, handler_s7_syn_vv0, handler_s7_rcv, handler_s7_ack_rst_vv0, handler_s7_close, handler_s7_send, handler_s7_ack_vv0 },
    /* S8  */ { handler_s8_closeconnection, handler_s8_ack_psh_vv1, handler_s8_syn_ack_vv0, handler_s8_rst_vv0, handler_s8_accept, handler_s8_fin_ack_vv0, handler_s8_listen, handler_s8_syn_vv0, handler_s8_rcv, handler_s8_ack_rst_vv0, handler_s8_close, handler_s8_send, handler_s8_ack_vv0 },
    /* S9  */ { handler_s9_closeconnection, handler_s9_ack_psh_vv1, handler_s9_syn_ack_vv0, handler_s9_rst_vv0, handler_s9_accept, handler_s9_fin_ack_vv0, handler_s9_listen, handler_s9_syn_vv0, handler_s9_rcv, handler_s9_ack_rst_vv0, handler_s9_close, handler_s9_send, handler_s9_ack_vv0 },
    /* S10 */ { handler_s10_closeconnection, handler_s10_ack_psh_vv1, handler_s10_syn_ack_vv0, handler_s10_rst_vv0, handler_s10_accept, handler_s10_fin_ack_vv0, handler_s10_listen, handler_s10_syn_vv0, handler_s10_rcv, handler_s10_ack_rst_vv0, handler_s10_close, handler_s10_send, handler_s10_ack_vv0 },
    /* S11 */ { handler_s11_closeconnection, handler_s11_ack_psh_vv1, handler_s11_syn_ack_vv0, handler_s11_rst_vv0, handler_s11_accept, handler_s11_fin_ack_vv0, handler_s11_listen, handler_s11_syn_vv0, handler_s11_rcv, handler_s11_ack_rst_vv0, handler_s11_close, handler_s11_send, handler_s11_ack_vv0 },
    /* S12 */ { handler_s12_closeconnection, handler_s12_ack_psh_vv1, handler_s12_syn_ack_vv0, handler_s12_rst_vv0, handler_s12_accept, handler_s12_fin_ack_vv0, handler_s12_listen, handler_s12_syn_vv0, handler_s12_rcv, handler_s12_ack_rst_vv0, handler_s12_close, handler_s12_send, handler_s12_ack_vv0 },
    /* S13 */ { handler_s13_closeconnection, handler_s13_ack_psh_vv1, handler_s13_syn_ack_vv0, handler_s13_rst_vv0, handler_s13_accept, handler_s13_fin_ack_vv0, handler_s13_listen, handler_s13_syn_vv0, handler_s13_rcv, handler_s13_ack_rst_vv0, handler_s13_close, handler_s13_send, handler_s13_ack_vv0 },
    /* S14 */ { handler_s14_closeconnection, handler_s14_ack_psh_vv1, handler_s14_syn_ack_vv0, handler_s14_rst_vv0, handler_s14_accept, handler_s14_fin_ack_vv0, handler_s14_listen, handler_s14_syn_vv0, handler_s14_rcv, handler_s14_ack_rst_vv0, handler_s14_close, handler_s14_send, handler_s14_ack_vv0 },
    /* S15 */ { handler_s15_closeconnection, handler_s15_ack_psh_vv1, handler_s15_syn_ack_vv0, handler_s15_rst_vv0, handler_s15_accept, handler_s15_fin_ack_vv0, handler_s15_listen, handler_s15_syn_vv0, handler_s15_rcv, handler_s15_ack_rst_vv0, handler_s15_close, handler_s15_send, handler_s15_ack_vv0 },
    /* S16 */ { handler_s16_closeconnection, handler_s16_ack_psh_vv1, handler_s16_syn_ack_vv0, handler_s16_rst_vv0, handler_s16_accept, handler_s16_fin_ack_vv0, handler_s16_listen, handler_s16_syn_vv0, handler_s16_rcv, handler_s16_ack_rst_vv0, handler_s16_close, handler_s16_send, handler_s16_ack_vv0 },
    /* S17 */ { handler_s17_closeconnection, handler_s17_ack_psh_vv1, handler_s17_syn_ack_vv0, handler_s17_rst_vv0, handler_s17_accept, handler_s17_fin_ack_vv0, handler_s17_listen, handler_s17_syn_vv0, handler_s17_rcv, handler_s17_ack_rst_vv0, handler_s17_close, handler_s17_send, handler_s17_ack_vv0 },
    /* S18 */ { handler_s18_closeconnection, handler_s18_ack_psh_vv1, handler_s18_syn_ack_vv0, handler_s18_rst_vv0, handler_s18_accept, handler_s18_fin_ack_vv0, handler_s18_listen, handler_s18_syn_vv0, handler_s18_rcv, handler_s18_ack_rst_vv0, handler_s18_close, handler_s18_send, handler_s18_ack_vv0 },
    /* S19 */ { handler_s19_closeconnection, handler_s19_ack_psh_vv1, handler_s19_syn_ack_vv0, handler_s19_rst_vv0, handler_s19_accept, handler_s19_fin_ack_vv0, handler_s19_listen, handler_s19_syn_vv0, handler_s19_rcv, handler_s19_ack_rst_vv0, handler_s19_close, handler_s19_send, handler_s19_ack_vv0 },
    /* S20 */ { handler_s20_closeconnection, handler_s20_ack_psh_vv1, handler_s20_syn_ack_vv0, handler_s20_rst_vv0, handler_s20_accept, handler_s20_fin_ack_vv0, handler_s20_listen, handler_s20_syn_vv0, handler_s20_rcv, handler_s20_ack_rst_vv0, handler_s20_close, handler_s20_send, handler_s20_ack_vv0 },
    /* S21 */ { handler_s21_closeconnection, handler_s21_ack_psh_vv1, handler_s21_syn_ack_vv0, handler_s21_rst_vv0, handler_s21_accept, handler_s21_fin_ack_vv0, handler_s21_listen, handler_s21_syn_vv0, handler_s21_rcv, handler_s21_ack_rst_vv0, handler_s21_close, handler_s21_send, handler_s21_ack_vv0 },
    /* S22 */ { handler_s22_closeconnection, handler_s22_ack_psh_vv1, handler_s22_syn_ack_vv0, handler_s22_rst_vv0, handler_s22_accept, handler_s22_fin_ack_vv0, handler_s22_listen, handler_s22_syn_vv0, handler_s22_rcv, handler_s22_ack_rst_vv0, handler_s22_close, handler_s22_send, handler_s22_ack_vv0 },
    /* S23 */ { handler_s23_closeconnection, handler_s23_ack_psh_vv1, handler_s23_syn_ack_vv0, handler_s23_rst_vv0, handler_s23_accept, handler_s23_fin_ack_vv0, handler_s23_listen, handler_s23_syn_vv0, handler_s23_rcv, handler_s23_ack_rst_vv0, handler_s23_close, handler_s23_send, handler_s23_ack_vv0 },
    /* S24 */ { handler_s24_closeconnection, handler_s24_ack_psh_vv1, handler_s24_syn_ack_vv0, handler_s24_rst_vv0, handler_s24_accept, handler_s24_fin_ack_vv0, handler_s24_listen, handler_s24_syn_vv0, handler_s24_rcv, handler_s24_ack_rst_vv0, handler_s24_close, handler_s24_send, handler_s24_ack_vv0 },
    /* S25 */ { handler_s25_closeconnection, handler_s25_ack_psh_vv1, handler_s25_syn_ack_vv0, handler_s25_rst_vv0, handler_s25_accept, handler_s25_fin_ack_vv0, handler_s25_listen, handler_s25_syn_vv0, handler_s25_rcv, handler_s25_ack_rst_vv0, handler_s25_close, handler_s25_send, handler_s25_ack_vv0 },
    /* S26 */ { handler_s26_closeconnection, handler_s26_ack_psh_vv1, handler_s26_syn_ack_vv0, handler_s26_rst_vv0, handler_s26_accept, handler_s26_fin_ack_vv0, handler_s26_listen, handler_s26_syn_vv0, handler_s26_rcv, handler_s26_ack_rst_vv0, handler_s26_close, handler_s26_send, handler_s26_ack_vv0 },
    /* S27 */ { handler_s27_closeconnection, handler_s27_ack_psh_vv1, handler_s27_syn_ack_vv0, handler_s27_rst_vv0, handler_s27_accept, handler_s27_fin_ack_vv0, handler_s27_listen, handler_s27_syn_vv0, handler_s27_rcv, handler_s27_ack_rst_vv0, handler_s27_close, handler_s27_send, handler_s27_ack_vv0 },
    /* S28 */ { handler_s28_closeconnection, handler_s28_ack_psh_vv1, handler_s28_syn_ack_vv0, handler_s28_rst_vv0, handler_s28_accept, handler_s28_fin_ack_vv0, handler_s28_listen, handler_s28_syn_vv0, handler_s28_rcv, handler_s28_ack_rst_vv0, handler_s28_close, handler_s28_send, handler_s28_ack_vv0 },
    /* S29 */ { handler_s29_closeconnection, handler_s29_ack_psh_vv1, handler_s29_syn_ack_vv0, handler_s29_rst_vv0, handler_s29_accept, handler_s29_fin_ack_vv0, handler_s29_listen, handler_s29_syn_vv0, handler_s29_rcv, handler_s29_ack_rst_vv0, handler_s29_close, handler_s29_send, handler_s29_ack_vv0 },
    /* S30 */ { handler_s30_closeconnection, handler_s30_ack_psh_vv1, handler_s30_syn_ack_vv0, handler_s30_rst_vv0, handler_s30_accept, handler_s30_fin_ack_vv0, handler_s30_listen, handler_s30_syn_vv0, handler_s30_rcv, handler_s30_ack_rst_vv0, handler_s30_close, handler_s30_send, handler_s30_ack_vv0 },
    /* S31 */ { handler_s31_closeconnection, handler_s31_ack_psh_vv1, handler_s31_syn_ack_vv0, handler_s31_rst_vv0, handler_s31_accept, handler_s31_fin_ack_vv0, handler_s31_listen, handler_s31_syn_vv0, handler_s31_rcv, handler_s31_ack_rst_vv0, handler_s31_close, handler_s31_send, handler_s31_ack_vv0 },
    /* S32 */ { handler_s32_closeconnection, handler_s32_ack_psh_vv1, handler_s32_syn_ack_vv0, handler_s32_rst_vv0, handler_s32_accept, handler_s32_fin_ack_vv0, handler_s32_listen, handler_s32_syn_vv0, handler_s32_rcv, handler_s32_ack_rst_vv0, handler_s32_close, handler_s32_send, handler_s32_ack_vv0 },
    /* S33 */ { handler_s33_closeconnection, handler_s33_ack_psh_vv1, handler_s33_syn_ack_vv0, handler_s33_rst_vv0, handler_s33_accept, handler_s33_fin_ack_vv0, handler_s33_listen, handler_s33_syn_vv0, handler_s33_rcv, handler_s33_ack_rst_vv0, handler_s33_close, handler_s33_send, handler_s33_ack_vv0 },
    /* S34 */ { handler_s34_closeconnection, handler_s34_ack_psh_vv1, handler_s34_syn_ack_vv0, handler_s34_rst_vv0, handler_s34_accept, handler_s34_fin_ack_vv0, handler_s34_listen, handler_s34_syn_vv0, handler_s34_rcv, handler_s34_ack_rst_vv0, handler_s34_close, handler_s34_send, handler_s34_ack_vv0 },
    /* S35 */ { handler_s35_closeconnection, handler_s35_ack_psh_vv1, handler_s35_syn_ack_vv0, handler_s35_rst_vv0, handler_s35_accept, handler_s35_fin_ack_vv0, handler_s35_listen, handler_s35_syn_vv0, handler_s35_rcv, handler_s35_ack_rst_vv0, handler_s35_close, handler_s35_send, handler_s35_ack_vv0 },
    /* S36 */ { handler_s36_closeconnection, handler_s36_ack_psh_vv1, handler_s36_syn_ack_vv0, handler_s36_rst_vv0, handler_s36_accept, handler_s36_fin_ack_vv0, handler_s36_listen, handler_s36_syn_vv0, handler_s36_rcv, handler_s36_ack_rst_vv0, handler_s36_close, handler_s36_send, handler_s36_ack_vv0 },
    /* S37 */ { handler_s37_closeconnection, handler_s37_ack_psh_vv1, handler_s37_syn_ack_vv0, handler_s37_rst_vv0, handler_s37_accept, handler_s37_fin_ack_vv0, handler_s37_listen, handler_s37_syn_vv0, handler_s37_rcv, handler_s37_ack_rst_vv0, handler_s37_close, handler_s37_send, handler_s37_ack_vv0 },
    /* S38 */ { handler_s38_closeconnection, handler_s38_ack_psh_vv1, handler_s38_syn_ack_vv0, handler_s38_rst_vv0, handler_s38_accept, handler_s38_fin_ack_vv0, handler_s38_listen, handler_s38_syn_vv0, handler_s38_rcv, handler_s38_ack_rst_vv0, handler_s38_close, handler_s38_send, handler_s38_ack_vv0 },
    /* S39 */ { handler_s39_closeconnection, handler_s39_ack_psh_vv1, handler_s39_syn_ack_vv0, handler_s39_rst_vv0, handler_s39_accept, handler_s39_fin_ack_vv0, handler_s39_listen, handler_s39_syn_vv0, handler_s39_rcv, handler_s39_ack_rst_vv0, handler_s39_close, handler_s39_send, handler_s39_ack_vv0 },
    /* S40 */ { handler_s40_closeconnection, handler_s40_ack_psh_vv1, handler_s40_syn_ack_vv0, handler_s40_rst_vv0, handler_s40_accept, handler_s40_fin_ack_vv0, handler_s40_listen, handler_s40_syn_vv0, handler_s40_rcv, handler_s40_ack_rst_vv0, handler_s40_close, handler_s40_send, handler_s40_ack_vv0 },
    /* S41 */ { handler_s41_closeconnection, handler_s41_ack_psh_vv1, handler_s41_syn_ack_vv0, handler_s41_rst_vv0, handler_s41_accept, handler_s41_fin_ack_vv0, handler_s41_listen, handler_s41_syn_vv0, handler_s41_rcv, handler_s41_ack_rst_vv0, handler_s41_close, handler_s41_send, handler_s41_ack_vv0 },
    /* S42 */ { handler_s42_closeconnection, handler_s42_ack_psh_vv1, handler_s42_syn_ack_vv0, handler_s42_rst_vv0, handler_s42_accept, handler_s42_fin_ack_vv0, handler_s42_listen, handler_s42_syn_vv0, handler_s42_rcv, handler_s42_ack_rst_vv0, handler_s42_close, handler_s42_send, handler_s42_ack_vv0 },
    /* S43 */ { handler_s43_closeconnection, handler_s43_ack_psh_vv1, handler_s43_syn_ack_vv0, handler_s43_rst_vv0, handler_s43_accept, handler_s43_fin_ack_vv0, handler_s43_listen, handler_s43_syn_vv0, handler_s43_rcv, handler_s43_ack_rst_vv0, handler_s43_close, handler_s43_send, handler_s43_ack_vv0 },
    /* S44 */ { handler_s44_closeconnection, handler_s44_ack_psh_vv1, handler_s44_syn_ack_vv0, handler_s44_rst_vv0, handler_s44_accept, handler_s44_fin_ack_vv0, handler_s44_listen, handler_s44_syn_vv0, handler_s44_rcv, handler_s44_ack_rst_vv0, handler_s44_close, handler_s44_send, handler_s44_ack_vv0 },
    /* S45 */ { handler_s45_closeconnection, handler_s45_ack_psh_vv1, handler_s45_syn_ack_vv0, handler_s45_rst_vv0, handler_s45_accept, handler_s45_fin_ack_vv0, handler_s45_listen, handler_s45_syn_vv0, handler_s45_rcv, handler_s45_ack_rst_vv0, handler_s45_close, handler_s45_send, handler_s45_ack_vv0 },
    /* S46 */ { handler_s46_closeconnection, handler_s46_ack_psh_vv1, handler_s46_syn_ack_vv0, handler_s46_rst_vv0, handler_s46_accept, handler_s46_fin_ack_vv0, handler_s46_listen, handler_s46_syn_vv0, handler_s46_rcv, handler_s46_ack_rst_vv0, handler_s46_close, handler_s46_send, handler_s46_ack_vv0 },
    /* S47 */ { handler_s47_closeconnection, handler_s47_ack_psh_vv1, handler_s47_syn_ack_vv0, handler_s47_rst_vv0, handler_s47_accept, handler_s47_fin_ack_vv0, handler_s47_listen, handler_s47_syn_vv0, handler_s47_rcv, handler_s47_ack_rst_vv0, handler_s47_close, handler_s47_send, handler_s47_ack_vv0 },
    /* S48 */ { handler_s48_closeconnection, handler_s48_ack_psh_vv1, handler_s48_syn_ack_vv0, handler_s48_rst_vv0, handler_s48_accept, handler_s48_fin_ack_vv0, handler_s48_listen, handler_s48_syn_vv0, handler_s48_rcv, handler_s48_ack_rst_vv0, handler_s48_close, handler_s48_send, handler_s48_ack_vv0 },
    /* S49 */ { handler_s49_closeconnection, handler_s49_ack_psh_vv1, handler_s49_syn_ack_vv0, handler_s49_rst_vv0, handler_s49_accept, handler_s49_fin_ack_vv0, handler_s49_listen, handler_s49_syn_vv0, handler_s49_rcv, handler_s49_ack_rst_vv0, handler_s49_close, handler_s49_send, handler_s49_ack_vv0 },
    /* S50 */ { handler_s50_closeconnection, handler_s50_ack_psh_vv1, handler_s50_syn_ack_vv0, handler_s50_rst_vv0, handler_s50_accept, handler_s50_fin_ack_vv0, handler_s50_listen, handler_s50_syn_vv0, handler_s50_rcv, handler_s50_ack_rst_vv0, handler_s50_close, handler_s50_send, handler_s50_ack_vv0 },
    /* S51 */ { handler_s51_closeconnection, handler_s51_ack_psh_vv1, handler_s51_syn_ack_vv0, handler_s51_rst_vv0, handler_s51_accept, handler_s51_fin_ack_vv0, handler_s51_listen, handler_s51_syn_vv0, handler_s51_rcv, handler_s51_ack_rst_vv0, handler_s51_close, handler_s51_send, handler_s51_ack_vv0 },
    /* S52 */ { handler_s52_closeconnection, handler_s52_ack_psh_vv1, handler_s52_syn_ack_vv0, handler_s52_rst_vv0, handler_s52_accept, handler_s52_fin_ack_vv0, handler_s52_listen, handler_s52_syn_vv0, handler_s52_rcv, handler_s52_ack_rst_vv0, handler_s52_close, handler_s52_send, handler_s52_ack_vv0 },
    /* S53 */ { handler_s53_closeconnection, handler_s53_ack_psh_vv1, handler_s53_syn_ack_vv0, handler_s53_rst_vv0, handler_s53_accept, handler_s53_fin_ack_vv0, handler_s53_listen, handler_s53_syn_vv0, handler_s53_rcv, handler_s53_ack_rst_vv0, handler_s53_close, handler_s53_send, handler_s53_ack_vv0 },
    /* S54 */ { handler_s54_closeconnection, handler_s54_ack_psh_vv1, handler_s54_syn_ack_vv0, handler_s54_rst_vv0, handler_s54_accept, handler_s54_fin_ack_vv0, handler_s54_listen, handler_s54_syn_vv0, handler_s54_rcv, handler_s54_ack_rst_vv0, handler_s54_close, handler_s54_send, handler_s54_ack_vv0 }
};

static inline uint8_t fsm_transition(uint8_t state, uint8_t event) {
    EventHandler t_handler = (EventHandler) pgm_read_word(&transition_table[state][event]);
    return t_handler(); // indeksiranje + indirektni poziv
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
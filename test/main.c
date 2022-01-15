#include "uart.h"
#include "twi.h"
#include "BMP280.h"

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>

typedef struct USART_RXC_cb_ctx_t {
    uint8_t rx;
    int cnt;
} USART_RXC_cb_ctx;

/**
 * USART RX interrupt callback handle
 * @param[inout] ctx user data for interrupt callback
 * When ISR occurs USART_RXC_cb will be called with ctx as parameter
 * UART RX data (UDR) should be saved in this function
 */
static void USART_RXC_cb_handle(void* ctx) {
    USART_RXC_cb_ctx* t_ctx = (USART_RXC_cb_ctx*)ctx;

    t_ctx->rx = UDR;
    t_ctx->cnt++;
}

typedef struct TWI_cb_ctx_t {
    int cnt;
}TWI_cb_ctx;

/**
 * TWI interrupt callback handle
 * @param[inout] ctx user data for interrupt callback
 */
static void TWI_cb_handle(void* ctx) {
    TWI_cb_ctx* t_ctx = (TWI_cb_ctx*)ctx;

    t_ctx->cnt++;
}

int main(void) {

    // UART INIT
    //-------------------------------
    const uint16_t baud_rate = 38400;

    uint8_t sts = 0;
    struct USART_RXC_cb_ctx_t USART_RXC_ctx = {};

    regiter_USART_RXC_cb(USART_RXC_cb_handle, &USART_RXC_ctx);

    USART_init(baud_rate);

    printf("Init Done UART baud: %u\n", (uint16_t)baud_rate);

    // TWI init
    //-------------------------------
    uint8_t slave_addr;
    TWI_cb_ctx twi_ctx = {};
    regiter_TWI_isr_cb(TWI_cb_handle, &twi_ctx);

    TWI_init(TWI_PS_1, 2);

    printf("TWI init done\n");

    sts = TWI_sniff(0x00, 0x7F, &slave_addr);
    if (sts == 0xFF) printf("TWI error\n");

    printf("TWI sniff: slaves:%d fist slave addr:0x%02x\n", sts, slave_addr);
    printf("total number of TWI interrupts: %d\n", twi_ctx.cnt);

    // BMP280 init
    //-------------------------------
    register_BMP_cb(TWI_write_reg, TWI_read_reg_burst);

    sts = BMP280_init();
    if (sts) return sts;

    printf("BMP280 init done\n");

    BMP280_final bmp_data = {};

    while(1) {
        BMP280_get_data(&bmp_data);
        printf("%li, %li\n",bmp_data.temp, bmp_data.baro);
        _delay_ms(300);
    }

    return sts;
}

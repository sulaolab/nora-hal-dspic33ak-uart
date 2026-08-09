/*
 * Minimal async TX/RX integration example for dspic33ak-hal-uart.
 *
 * Board code must configure the UART clock source, PPS, GPIO direction, and
 * analog-disable settings before app_uart_init().
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "nora_uart.h"
#include "nora_uart_dspic33ak_rx_isr_ring.h"

#define APP_UART_INST NORA_UART_INST_1

static uint8_t s_uart_rx_ring[256u];
static volatile uint32_t s_uart_events;

static const uint8_t s_tx_message[] =
    "async TX through dspic33ak-hal-uart\r\n";
static uint8_t s_rx_buffer[4u];

static void app_uart_callback(
    nora_uart_instance_t inst,
    uint32_t events,
    void *user_data)
{
    (void)user_data;

    if (inst == APP_UART_INST) {
        s_uart_events |= events;
    }
}

void app_uart_init(void)
{
    nora_uart_config_t cfg;

    memset(&cfg, 0, sizeof(cfg));

    cfg.uart_clk_hz = 100000000u;  /* CLKGEN8 frequency supplied by board code */
    cfg.baudrate    = 115200u;
    cfg.timeout_ms  = 0u;
    cfg.get_ms      = NULL;

    cfg.data_bits   = 8u;
    cfg.stop_bits   = 1u;
    cfg.parity      = NORA_UART_PARITY_NONE;

    cfg.enable_tx   = true;
    cfg.enable_rx   = true;

    cfg.rx_mode             = NORA_UART_RX_MODE_ISR_RING;
    cfg.rx_ring_buffer      = s_uart_rx_ring;
    cfg.rx_ring_buffer_size = sizeof(s_uart_rx_ring);

    cfg.rx_irq_priority = 3u;
    cfg.tx_irq_priority = 3u;

    /*
     * Board-level CLKGEN8 / PPS / GPIO setup must be completed before this call.
     * nora_uart_init() clears callback state, so register the callback after
     * init succeeds.
     */
    if (nora_uart_init(APP_UART_INST, &cfg) == NORA_UART_OK) {
        (void)nora_uart_set_callback(
            APP_UART_INST,
            app_uart_callback,
            NULL);
    }
}

bool app_uart_async_tx_start(void)
{
    s_uart_events &= ~NORA_UART_EVENT_SEND_COMPLETE;

    return (nora_uart_tx_start(
                APP_UART_INST,
                s_tx_message,
                sizeof(s_tx_message) - 1u) == NORA_UART_OK);
}

bool app_uart_async_tx_submitted(void)
{
    return ((s_uart_events & NORA_UART_EVENT_SEND_COMPLETE) != 0u);
}

bool app_uart_async_tx_line_idle(void)
{
    /*
     * SEND_COMPLETE means all bytes were submitted to the hardware. Confirm
     * tx_done() before returning to blocking output such as printf().
     */
    return app_uart_async_tx_submitted() &&
           nora_uart_tx_done(APP_UART_INST);
}

bool app_uart_async_rx_start_clean(void)
{
    s_uart_events &= ~NORA_UART_EVENT_RX_COMPLETE;
    memset(s_rx_buffer, 0, sizeof(s_rx_buffer));

    return (nora_uart_rx_start_clean(
                APP_UART_INST,
                s_rx_buffer,
                sizeof(s_rx_buffer)) == NORA_UART_OK);
}

bool app_uart_async_rx_complete(void)
{
    return ((s_uart_events & NORA_UART_EVENT_RX_COMPLETE) != 0u);
}

bool app_uart_async_rx_matches_rxok(void)
{
    static const uint8_t expected[] = { 'R', 'X', 'O', 'K' };

    return app_uart_async_rx_complete() &&
           (nora_uart_rx_count_get(APP_UART_INST) == sizeof(expected)) &&
           (memcmp(s_rx_buffer, expected, sizeof(expected)) == 0);
}

void app_uart_async_abort_all(void)
{
    (void)nora_uart_tx_abort(APP_UART_INST);
    (void)nora_uart_rx_abort(APP_UART_INST);
}

size_t app_uart_async_tx_count(void)
{
    return nora_uart_tx_count_get(APP_UART_INST);
}

size_t app_uart_async_rx_count(void)
{
    return nora_uart_rx_count_get(APP_UART_INST);
}

/*
 * Application-owned interrupt vector wrappers.
 *
 * Adjust the interrupt attribute and vector names for your project, selected
 * UART instance, device, and compiler settings.
 */
void __attribute__((interrupt, context)) _U1RXInterrupt(void)
{
    nora_uart_rx_irq_handler(APP_UART_INST);
}

void __attribute__((interrupt, context)) _U1TXInterrupt(void)
{
    nora_uart_tx_irq_handler(APP_UART_INST);
}

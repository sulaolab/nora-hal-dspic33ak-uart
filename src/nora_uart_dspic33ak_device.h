#ifndef DSPIC33AK_UART_DEVICE_H
#define DSPIC33AK_UART_DEVICE_H

/* ========================================================================== */
/* Includes                                                                   */
/* ========================================================================== */

#include <stdbool.h>
#include "dspic33ak_uart.h"
#include "dspic33ak_uart_reg.h"

/* ========================================================================== */
/* C++ Linkage                                                                */
/* ========================================================================== */

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/* Internal Types                                                             */
/* ========================================================================== */

typedef struct {
    bool present;
    dspic33ak_uart_regs_t regs;
} dspic33ak_uart_device_t;

/* ========================================================================== */
/* Internal API                                                               */
/* ========================================================================== */

const dspic33ak_uart_device_t *dspic33ak_uart_get_device(
    dspic33ak_uart_instance_t inst);

bool dspic33ak_uart_instance_is_present(
    dspic33ak_uart_instance_t inst);

bool dspic33ak_uart_device_set_rx_irq_priority(
    dspic33ak_uart_instance_t inst,
    uint8_t priority);

bool dspic33ak_uart_device_set_tx_irq_priority(
    dspic33ak_uart_instance_t inst,
    uint8_t priority);

/* ========================================================================== */
/* C++ Linkage                                                                */
/* ========================================================================== */

#ifdef __cplusplus
}
#endif

#endif /* DSPIC33AK_UART_DEVICE_H */

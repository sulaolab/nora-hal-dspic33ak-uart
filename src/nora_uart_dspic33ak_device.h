#ifndef NORA_UART_DSPIC33AK_DEVICE_H
#define NORA_UART_DSPIC33AK_DEVICE_H

/* ========================================================================== */
/* Includes                                                                   */
/* ========================================================================== */

#include <stdbool.h>
#include "nora_uart.h"
#include "nora_uart_dspic33ak_reg.h"

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
    nora_uart_dspic33ak_regs_t regs;
} nora_uart_dspic33ak_device_t;

/* ========================================================================== */
/* Internal API                                                               */
/* ========================================================================== */

const nora_uart_dspic33ak_device_t *nora_uart_dspic33ak_get_device(
    nora_uart_instance_t inst);

bool nora_uart_dspic33ak_instance_is_present(
    nora_uart_instance_t inst);

bool nora_uart_dspic33ak_device_set_rx_irq_priority(
    nora_uart_instance_t inst,
    uint8_t priority);

bool nora_uart_dspic33ak_device_set_tx_irq_priority(
    nora_uart_instance_t inst,
    uint8_t priority);

/*
 * Per-instance interrupt flag / enable access.
 *
 * These replace the old { &IFSn, &IECn, mask } descriptor plus generic pointer
 * helpers.  Each one resolves to a DFP bit alias (_UxRXIF, _UxTXIE, ...) written
 * with a literal, so the compiler emits one atomic bit operation on a register
 * that is shared with every other peripheral.  All return false for an absent or
 * out-of-range instance; _is_enabled reports through *enabled.
 */
bool nora_uart_dspic33ak_device_rx_irq_clear_flag(nora_uart_instance_t inst);
bool nora_uart_dspic33ak_device_rx_irq_enable(nora_uart_instance_t inst,
                                                  bool enable);
bool nora_uart_dspic33ak_device_rx_irq_is_enabled(nora_uart_instance_t inst,
                                                      bool *enabled);
bool nora_uart_dspic33ak_device_tx_irq_clear_flag(nora_uart_instance_t inst);
bool nora_uart_dspic33ak_device_tx_irq_raise_flag(nora_uart_instance_t inst);
bool nora_uart_dspic33ak_device_tx_irq_enable(nora_uart_instance_t inst,
                                                  bool enable);

/* ========================================================================== */
/* C++ Linkage                                                                */
/* ========================================================================== */

#ifdef __cplusplus
}
#endif

#endif /* NORA_UART_DSPIC33AK_DEVICE_H */

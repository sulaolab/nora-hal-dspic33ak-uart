#ifndef NORA_UART_DSPIC33AK_RX_ISR_RING_H
#define NORA_UART_DSPIC33AK_RX_ISR_RING_H

/* ========================================================================== */
/* Includes                                                                   */
/* ========================================================================== */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "nora_uart.h"

/* ========================================================================== */
/* C++ Linkage                                                                */
/* ========================================================================== */

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/* Module Overview                                                            */
/* ========================================================================== */

/*
 * UART RX interrupt-driven ring HAL (dsPIC33AK).
 *
 * Provides the RX ISR ring core: a single-producer (ISR) / single-consumer
 * (reader) software ring fed by draining the RX FIFO, plus the RX ISR ring
 * runtime status counters.
 *
 * Design policy:
 *   - No printf / halt / blocking calls.
 *   - No application or board-specific dependencies.
 *   - No dynamic allocation; RX ring buffer storage is caller-provided.
 *   - No project-specific compile-time macros: the ring buffer storage, its size,
 *     interrupt priority, and RX backend selection stay in the
 *     board/application/integration layer.
 *   - The scattered RX interrupt Flag/Enable/Priority bits (_UxRXIF/IE/IP) are
 *     isolated inside small per-instance switch helpers in
 *     nora_uart_dspic33ak_rx_isr_ring.c.
 *
 * Interrupt vector ownership:
 *   This HAL does NOT define the _UxRXInterrupt vector. The application/integration
 *   layer owns the thin vector wrapper and calls nora_uart_rx_irq_handler()
 *   from it. nora_uart_rx_irq_handler() is an ordinary function, not an
 *   interrupt vector declaration.
 *
 * Ring buffer ownership:
 *   The ring buffer storage is caller-provided (passed in via config). This HAL
 *   only holds the buffer pointer, its size and the read/write indices, so it
 *   consumes no implicit per-instance RAM for buffers that are never used.
 */

/* ========================================================================== */
/* Public Types                                                               */
/* ========================================================================== */

typedef struct
{
    uint8_t *buffer;        /* caller-provided ring storage (not owned)        */
    uint16_t buffer_size;   /* size of buffer in bytes; must be >= 2           */
    uint8_t irq_priority;   /* CPU interrupt priority for the RX interrupt     */
} nora_uart_dspic33ak_rx_isr_config_t;

/*
 * RX ISR ring runtime status snapshot.
 *
 * This is different from nora_uart_status_t:
 *   - nora_uart_status_t          is a function return code.
 *   - nora_uart_dspic33ak_rx_isr_status_t   is accumulated runtime state / counters.
 */
typedef struct
{
    uint32_t rx_isr_count;
    uint32_t rx_byte_count;
    uint32_t rx_fifo_overflow_count;
    uint32_t framing_error_count;
    uint32_t parity_error_count;
    uint32_t autobaud_overflow_count;
    uint32_t tx_collision_count;
    uint32_t rx_ring_overflow_count;
    uint32_t rx_max_drain_count;
} nora_uart_dspic33ak_rx_isr_status_t;

/* ========================================================================== */
/* Public API                                                                 */
/* ========================================================================== */

/*
 * Configure the RX ISR ring for an instance.
 *
 * Validates the instance/config, stores the caller-provided buffer pointer,
 * resets the ring indices and status counters, sets the RX FIFO watermark to
 * interrupt on >= 1 byte, and programs the RX interrupt priority. The RX
 * interrupt is left DISABLED; call nora_uart_dspic33ak_rx_isr_enable() to start it.
 *
 * Returns:
 *   NORA_UART_ERR_INVALID_ARG     config/buffer NULL or buffer_size < 2
 *   NORA_UART_ERR_NOT_PRESENT     instance not present on this device
 *   NORA_UART_ERR_NOT_INITIALIZED UART not initialized yet
 *   NORA_UART_ERR_UNSUPPORTED     no RX interrupt mapping for this instance
 *   NORA_UART_OK                  configured
 */
nora_uart_status_t nora_uart_dspic33ak_rx_isr_config(
    nora_uart_instance_t inst,
    const nora_uart_dspic33ak_rx_isr_config_t *config);

/* Enable the RX interrupt (instance must be configured and initialized). */
nora_uart_status_t nora_uart_dspic33ak_rx_isr_enable(
    nora_uart_instance_t inst);

/* Disable the RX interrupt (safe direction; allowed even if not configured). */
nora_uart_status_t nora_uart_dspic33ak_rx_isr_disable(
    nora_uart_instance_t inst);

/* True when the ring holds at least one buffered byte. */
bool nora_uart_dspic33ak_rx_isr_ready(
    nora_uart_instance_t inst);

/*
 * Pop one byte from the ring.
 *   NORA_UART_ERR_INVALID_ARG  data == NULL
 *   NORA_UART_ERR_RX_EMPTY     ring empty
 *   NORA_UART_OK               one byte written to *data
 */
nora_uart_status_t nora_uart_dspic33ak_rx_isr_read_byte(
    nora_uart_instance_t inst,
    uint8_t *data);

/* Drop buffered ring contents and drain the hardware RX FIFO. */
void nora_uart_dspic33ak_rx_isr_flush(
    nora_uart_instance_t inst);

/* Snapshot the RX ISR ring runtime status counters (atomic vs the ISR). */
void nora_uart_dspic33ak_rx_isr_status_get(
    nora_uart_instance_t inst,
    nora_uart_dspic33ak_rx_isr_status_t *status);

/* Zero the RX ISR ring runtime status counters (atomic vs the ISR). */
void nora_uart_dspic33ak_rx_isr_status_clear(
    nora_uart_instance_t inst);

/* ========================================================================== */
/* Internal HAL hooks                                                         */
/*                                                                            */
/* Glue between the RX ISR ring core (nora_uart_dspic33ak_rx_isr_ring.c) and the    */
/* asynchronous transfer engine and its callback state (nora_uart_dspic33ak.c).     */
/* The RX ISR handler above calls these; they are implemented in               */
/* nora_uart_dspic33ak.c. Not part of the application-facing API.                   */
/* ========================================================================== */

/*
 * Internal HAL hook. Do NOT call from application code.
 *
 * Offer one freshly received byte to an active async RX transfer. Returns true
 * when the byte was consumed by the transfer (and the caller must NOT also push
 * it to the RX ISR ring); returns false when no async RX transfer is active.
 * Reports NORA_UART_EVENT_RX_COMPLETE when the requested length is reached.
 */
bool nora_uart_dspic33ak_async_rx_feed(
    nora_uart_instance_t inst,
    uint8_t byte);

/*
 * Internal HAL hook. Do NOT call from application code.
 *
 * Forward RX-side event bits (errors / RX_READY) to the registered callback.
 */
void nora_uart_dspic33ak_async_notify_events(
    nora_uart_instance_t inst,
    uint32_t events);

/* ========================================================================== */
/* C++ Linkage                                                                */
/* ========================================================================== */

#ifdef __cplusplus
}
#endif

#endif /* NORA_UART_DSPIC33AK_RX_ISR_RING_H */

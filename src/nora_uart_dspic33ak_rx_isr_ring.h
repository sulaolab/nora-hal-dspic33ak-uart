#ifndef DSPIC33AK_UART_RX_ISR_RING_H
#define DSPIC33AK_UART_RX_ISR_RING_H

/* ========================================================================== */
/* Includes                                                                   */
/* ========================================================================== */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "dspic33ak_uart.h"

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
 *     dspic33ak_uart_rx_isr_ring.c.
 *
 * Interrupt vector ownership:
 *   This HAL does NOT define the _UxRXInterrupt vector. The application/integration
 *   layer owns the thin vector wrapper and calls dspic33ak_uart_rx_irq_handler()
 *   from it. dspic33ak_uart_rx_irq_handler() is an ordinary function, not an
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
} dspic33ak_uart_rx_isr_config_t;

/*
 * RX ISR ring runtime status snapshot.
 *
 * This is different from dspic33ak_uart_status_t:
 *   - dspic33ak_uart_status_t          is a function return code.
 *   - dspic33ak_uart_rx_isr_status_t   is accumulated runtime state / counters.
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
} dspic33ak_uart_rx_isr_status_t;

/* ========================================================================== */
/* Public API                                                                 */
/* ========================================================================== */

/*
 * Configure the RX ISR ring for an instance.
 *
 * Validates the instance/config, stores the caller-provided buffer pointer,
 * resets the ring indices and status counters, sets the RX FIFO watermark to
 * interrupt on >= 1 byte, and programs the RX interrupt priority. The RX
 * interrupt is left DISABLED; call dspic33ak_uart_rx_isr_enable() to start it.
 *
 * Returns:
 *   DSPIC33AK_UART_ERR_INVALID_ARG     config/buffer NULL or buffer_size < 2
 *   DSPIC33AK_UART_ERR_NOT_PRESENT     instance not present on this device
 *   DSPIC33AK_UART_ERR_NOT_INITIALIZED UART not initialized yet
 *   DSPIC33AK_UART_ERR_UNSUPPORTED     no RX interrupt mapping for this instance
 *   DSPIC33AK_UART_OK                  configured
 */
dspic33ak_uart_status_t dspic33ak_uart_rx_isr_config(
    dspic33ak_uart_instance_t inst,
    const dspic33ak_uart_rx_isr_config_t *config);

/* Enable the RX interrupt (instance must be configured and initialized). */
dspic33ak_uart_status_t dspic33ak_uart_rx_isr_enable(
    dspic33ak_uart_instance_t inst);

/* Disable the RX interrupt (safe direction; allowed even if not configured). */
dspic33ak_uart_status_t dspic33ak_uart_rx_isr_disable(
    dspic33ak_uart_instance_t inst);

/* True when the ring holds at least one buffered byte. */
bool dspic33ak_uart_rx_isr_ready(
    dspic33ak_uart_instance_t inst);

/*
 * Pop one byte from the ring.
 *   DSPIC33AK_UART_ERR_INVALID_ARG  data == NULL
 *   DSPIC33AK_UART_ERR_RX_EMPTY     ring empty
 *   DSPIC33AK_UART_OK               one byte written to *data
 */
dspic33ak_uart_status_t dspic33ak_uart_rx_isr_read_byte(
    dspic33ak_uart_instance_t inst,
    uint8_t *data);

/* Drop buffered ring contents and drain the hardware RX FIFO. */
void dspic33ak_uart_rx_isr_flush(
    dspic33ak_uart_instance_t inst);

/* Snapshot the RX ISR ring runtime status counters (atomic vs the ISR). */
void dspic33ak_uart_rx_isr_status_get(
    dspic33ak_uart_instance_t inst,
    dspic33ak_uart_rx_isr_status_t *status);

/* Zero the RX ISR ring runtime status counters (atomic vs the ISR). */
void dspic33ak_uart_rx_isr_status_clear(
    dspic33ak_uart_instance_t inst);

/*
 * RX interrupt service routine body. Called from the application-owned
 * _UxRXInterrupt vector. This is an ordinary function, NOT an interrupt vector:
 * it does not carry the interrupt/context attribute and does not declare the
 * vector name. It clears the RX interrupt flag, drains the RX FIFO into the
 * ring, and counts/clears the latched RX error flags. No printf / blocking.
 */
void dspic33ak_uart_rx_irq_handler(
    dspic33ak_uart_instance_t inst);

/* ========================================================================== */
/* Internal HAL hooks                                                         */
/*                                                                            */
/* Glue between the RX ISR ring core (dspic33ak_uart_rx_isr_ring.c) and the    */
/* asynchronous transfer engine and its callback state (dspic33ak_uart.c).     */
/* The RX ISR handler above calls these; they are implemented in               */
/* dspic33ak_uart.c. Not part of the application-facing API.                   */
/* ========================================================================== */

/*
 * Internal HAL hook. Do NOT call from application code.
 *
 * Offer one freshly received byte to an active async RX transfer. Returns true
 * when the byte was consumed by the transfer (and the caller must NOT also push
 * it to the RX ISR ring); returns false when no async RX transfer is active.
 * Reports DSPIC33AK_UART_EVENT_RX_COMPLETE when the requested length is reached.
 */
bool dspic33ak_uart_async_rx_feed(
    dspic33ak_uart_instance_t inst,
    uint8_t byte);

/*
 * Internal HAL hook. Do NOT call from application code.
 *
 * Forward RX-side event bits (errors / RX_READY) to the registered callback.
 */
void dspic33ak_uart_async_notify_events(
    dspic33ak_uart_instance_t inst,
    uint32_t events);

/* ========================================================================== */
/* C++ Linkage                                                                */
/* ========================================================================== */

#ifdef __cplusplus
}
#endif

#endif /* DSPIC33AK_UART_RX_ISR_RING_H */

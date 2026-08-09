#ifndef DSPIC33AK_UART_H
#define DSPIC33AK_UART_H

/* ========================================================================== */
/* Includes                                                                   */
/* ========================================================================== */

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

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
 * Small, readable dsPIC33AK CLKGEN8 UART byte-stream HAL.
 *
 * Clock assumption:
 *   - This HAL selects the UART clock source as CLKGEN8 (UxCONbits.CLKSEL) and
 *     uses fractional baud mode. The caller passes the resulting CLKGEN8 frequency
 *     as config.uart_clk_hz; the HAL computes the baud divisor from it.
 *   - Board/application code must configure (route/enable) CLKGEN8 appropriately
 *     before calling dspic33ak_uart_init(). Clock generator bring-up, PPS, and
 *     GPIO routing remain outside this HAL.
 *
 * Design policy:
 *   - Public API does not expose XC-DSC / DFP bitfield types.
 *   - Device-specific register names are isolated in dspic33ak_uart_device.c.
 *   - Register pointer and bit-mask helpers are isolated in dspic33ak_uart_reg.h.
 *   - Board-specific clock / PPS / GPIO routing stays outside this HAL.
 *   - stdio retarget, console parsing, and application command handling stay
 *     outside this HAL.
 *
 * Functional model:
 *   - init / deinit
 *   - presence and initialization queries
 *   - TX/RX status queries
 *   - blocking byte write with optional timeout
 *   - non-blocking byte read
 *   - buffer read/write helpers
 *   - RX FIFO flush
 */

/* ========================================================================== */
/* Public Types                                                               */
/* ========================================================================== */

typedef enum {
    DSPIC33AK_UART_INST_1 = 0,
    DSPIC33AK_UART_INST_2,
    DSPIC33AK_UART_INST_3,
    DSPIC33AK_UART_INST_4,
    DSPIC33AK_UART_INST_COUNT
} dspic33ak_uart_instance_t;

typedef enum {
    DSPIC33AK_UART_OK = 0,
    DSPIC33AK_UART_ERR_INVALID_ARG,
    DSPIC33AK_UART_ERR_NOT_PRESENT,
    DSPIC33AK_UART_ERR_NOT_INITIALIZED,
    DSPIC33AK_UART_ERR_BUSY,
    DSPIC33AK_UART_ERR_TIMEOUT,
    DSPIC33AK_UART_ERR_RX_EMPTY,
    DSPIC33AK_UART_ERR_TX_FULL,
    DSPIC33AK_UART_ERR_OVERRUN,
    DSPIC33AK_UART_ERR_FRAMING,
    DSPIC33AK_UART_ERR_PARITY,
    DSPIC33AK_UART_ERR_UNSUPPORTED
} dspic33ak_uart_status_t;

typedef uint32_t (*dspic33ak_uart_get_ms_fn)(void);

typedef enum {
    DSPIC33AK_UART_PARITY_NONE = 0,
    DSPIC33AK_UART_PARITY_EVEN,
    DSPIC33AK_UART_PARITY_ODD
} dspic33ak_uart_parity_t;

/*
 * RX backend selection (per instance).
 *
 *   POLLING  - RX is read directly from the hardware RX FIFO; no RX interrupt is
 *              enabled. The rx_ring_* / rx_irq_priority config fields are ignored.
 *   ISR_RING - dspic33ak_uart_init() sets up the interrupt-driven RX ring (see
 *              dspic33ak_uart_rx_isr_ring.h): the RX ISR drains the FIFO into a caller-
 *              provided software ring, and rx_ready/read_byte/rx_flush operate on
 *              that ring instead of the FIFO. Requires rx_ring_buffer != NULL,
 *              rx_ring_buffer_size >= 2, and uses rx_irq_priority.
 *
 * Selecting the backend per instance (rather than one global compile-time switch)
 * lets a build mix modes, e.g. UART1 console = ISR ring, UART2 log = polling.
 *
 * Only POLLING and ISR_RING are valid; dspic33ak_uart_init() rejects any other
 * rx_mode value with DSPIC33AK_UART_ERR_INVALID_ARG.
 */
typedef enum {
    DSPIC33AK_UART_RX_MODE_POLLING = 0,
    DSPIC33AK_UART_RX_MODE_ISR_RING
} dspic33ak_uart_rx_mode_t;

typedef struct {
    uint32_t uart_clk_hz;
    uint32_t baudrate;
    uint32_t timeout_ms;

    /*
     * Optional millisecond tick callback for timeout handling.
     * If get_ms is NULL, timeout handling is disabled.
     * If timeout_ms is 0, timeout handling is also disabled.
     */
    dspic33ak_uart_get_ms_fn get_ms;

    uint8_t data_bits;
    uint8_t stop_bits;
    dspic33ak_uart_parity_t parity;
    bool enable_tx;
    bool enable_rx;

    /* RX backend (see dspic33ak_uart_rx_mode_t). The rx_ring_* / rx_irq_priority
     * fields are used only when rx_mode == DSPIC33AK_UART_RX_MODE_ISR_RING. The
     * ring buffer storage is caller-provided so the HAL holds no implicit RAM. */
    dspic33ak_uart_rx_mode_t rx_mode;
    uint8_t  *rx_ring_buffer;
    uint16_t  rx_ring_buffer_size;
    uint8_t   rx_irq_priority;

    /*
     * CPU interrupt priority for the TX interrupt, used only by the non-blocking
     * TX transfer engine (dspic33ak_uart_tx_start). It is programmed at init and
     * is independent of rx_irq_priority. Builds that never call the async TX API
     * may leave this at any value; the TX interrupt stays disabled until a
     * transfer starts. A value of 0 disables the TX interrupt on this CPU, so an
     * async-TX user must set a non-zero priority here.
     */
    uint8_t   tx_irq_priority;
} dspic33ak_uart_config_t;

/*
 * RX runtime status snapshot (backend-aware).
 *
 * This is different from dspic33ak_uart_status_t:
 *   - dspic33ak_uart_status_t      is a function return code.
 *   - dspic33ak_uart_rx_status_t   is runtime RX state / counters.
 *
 * In ISR ring mode the counters are copied from the RX ISR ring backend. In
 * polling mode rx_mode is POLLING and the backend-specific counters are zero
 * (the polling path keeps no counters). Lets callers read RX diagnostics without
 * knowing or branching on the configured backend.
 */
typedef struct {
    dspic33ak_uart_rx_mode_t rx_mode;

    uint32_t rx_isr_count;
    uint32_t rx_byte_count;
    uint32_t rx_fifo_overflow_count;
    uint32_t framing_error_count;
    uint32_t parity_error_count;
    uint32_t autobaud_overflow_count;
    uint32_t tx_collision_count;
    uint32_t rx_ring_overflow_count;
    uint32_t rx_max_drain_count;
} dspic33ak_uart_rx_status_t;

/* ========================================================================== */
/* Public API                                                                 */
/* ========================================================================== */

dspic33ak_uart_status_t dspic33ak_uart_init(
    dspic33ak_uart_instance_t inst,
    const dspic33ak_uart_config_t *config);

dspic33ak_uart_status_t dspic33ak_uart_deinit(
    dspic33ak_uart_instance_t inst);

bool dspic33ak_uart_is_present(
    dspic33ak_uart_instance_t inst);

bool dspic33ak_uart_is_initialized(
    dspic33ak_uart_instance_t inst);

bool dspic33ak_uart_rx_ready(
    dspic33ak_uart_instance_t inst);

bool dspic33ak_uart_tx_ready(
    dspic33ak_uart_instance_t inst);

bool dspic33ak_uart_tx_done(
    dspic33ak_uart_instance_t inst);

dspic33ak_uart_status_t dspic33ak_uart_write_byte(
    dspic33ak_uart_instance_t inst,
    uint8_t data);

dspic33ak_uart_status_t dspic33ak_uart_read_byte(
    dspic33ak_uart_instance_t inst,
    uint8_t *data);

size_t dspic33ak_uart_write(
    dspic33ak_uart_instance_t inst,
    const void *data,
    size_t len);

size_t dspic33ak_uart_read(
    dspic33ak_uart_instance_t inst,
    void *data,
    size_t len);

void dspic33ak_uart_rx_flush(
    dspic33ak_uart_instance_t inst);

/*
 * Backend-aware RX status snapshot / clear.
 *
 * ISR ring mode reports/clears the RX ISR ring counters; polling mode reports a
 * zeroed snapshot (rx_mode = POLLING) and clear is a no-op. Callers use these
 * instead of the backend-specific dspic33ak_uart_rx_isr_status_* API so they
 * stay backend-agnostic.
 *
 * Returns DSPIC33AK_UART_ERR_INVALID_ARG (status NULL), _ERR_NOT_PRESENT,
 * _ERR_NOT_INITIALIZED, or DSPIC33AK_UART_OK.
 */
dspic33ak_uart_status_t dspic33ak_uart_rx_status_get(
    dspic33ak_uart_instance_t inst,
    dspic33ak_uart_rx_status_t *status);

dspic33ak_uart_status_t dspic33ak_uart_rx_status_clear(
    dspic33ak_uart_instance_t inst);

/* ========================================================================== */
/* Asynchronous Transfer Model (event-driven, non-blocking)                   */
/* ========================================================================== */

/*
 * Optional asynchronous transfer layer for upper layers that want a non-blocking
 * Send/Receive model with completion/error events (for example a CMSIS-style
 * USART wrapper built on top of this HAL).
 *
 * This layer is purely additive and does NOT replace or change the byte-stream
 * API above. The blocking write byte path, the non-blocking read byte path and
 * the RX ISR ring keep working exactly as before; the async transfer engine is
 * inert until dspic33ak_uart_tx_start(), dspic33ak_uart_rx_start(), or
 * dspic33ak_uart_rx_start_clean() is called.
 *
 * Intentionally generic: the events and the API below describe a UART, not any
 * specific middleware. No ARM_USART_* / ARM_DRIVER_* names appear here.
 *
 * Backend requirements:
 *   - Async TX requires TX enabled and a non-zero tx_irq_priority; otherwise
 *     dspic33ak_uart_tx_start() returns DSPIC33AK_UART_ERR_UNSUPPORTED (a transfer
 *     with no servicing interrupt would never complete). It also requires the
 *     application to route the device TX interrupt vector to
 *     dspic33ak_uart_tx_irq_handler(), as the RX vector forwards to
 *     dspic33ak_uart_rx_irq_handler().
 *   - Async RX requires RX enabled and DSPIC33AK_UART_RX_MODE_ISR_RING (the RX ISR
 *     feeds the async buffer); otherwise dspic33ak_uart_rx_start() returns
 *     DSPIC33AK_UART_ERR_UNSUPPORTED.
 *   - dspic33ak_uart_tx_enable(false) / dspic33ak_uart_rx_enable(false) return
 *     DSPIC33AK_UART_ERR_BUSY while an async transfer is active, so a transfer is
 *     never stranded by disabling its line mid-flight.
 */

/* Event bit-flags reported through the registered callback. Multiple bits may be
 * OR'd together in a single notification.
 *
 * SEND_COMPLETE means the driver has submitted every TX byte to the hardware
 * (FIFO/register) - the CMSIS ARM_USART_EVENT_SEND_COMPLETE sense, NOT physical
 * shift-register-empty. It is intentionally NOT named TX_COMPLETE to avoid being
 * read as the CMSIS ARM_USART_EVENT_TX_COMPLETE (line idle / shifter empty). Use
 * the existing dspic33ak_uart_tx_done() to confirm physical transmit completion. */
#define DSPIC33AK_UART_EVENT_SEND_COMPLETE     (1u << 0)  /* all TX data submitted (SEND_COMPLETE) */
#define DSPIC33AK_UART_EVENT_RX_COMPLETE       (1u << 1)  /* requested RX length got */
#define DSPIC33AK_UART_EVENT_RX_READY          (1u << 2)  /* unsolicited RX -> ring  */
#define DSPIC33AK_UART_EVENT_RX_OVERFLOW       (1u << 3)  /* software RX ring overflow */
#define DSPIC33AK_UART_EVENT_RX_FRAMING_ERROR  (1u << 4)  /* UxSTAT FERIF            */
#define DSPIC33AK_UART_EVENT_RX_PARITY_ERROR   (1u << 5)  /* UxSTAT PERIF            */
#define DSPIC33AK_UART_EVENT_RX_OVERRUN_ERROR  (1u << 6)  /* hardware RX FIFO overrun */

/*
 * Event callback. Invoked with the OR'd event bits for the instance and the
 * user_data pointer registered alongside it.
 *
 * NOTE (initial version): the callback may be invoked from interrupt context
 * (TX/RX ISR). Keep it short and non-blocking; do not call back into a blocking
 * HAL API from inside it.
 */
typedef void (*dspic33ak_uart_event_callback_t)(
    dspic33ak_uart_instance_t inst,
    uint32_t events,
    void *user_data);

/*
 * Register (or clear, with callback == NULL) the event callback for an instance.
 * Valid before or after init; dspic33ak_uart_init()/deinit() clear it, so call
 * this after init. Returns _ERR_INVALID_ARG / _ERR_NOT_PRESENT or _OK.
 *
 * The registered callback is invoked from interrupt context (TX/RX ISR). It must
 * be short and non-blocking, and must not call back into a blocking HAL API
 * (for example dspic33ak_uart_write_byte() with a timeout).
 */
dspic33ak_uart_status_t dspic33ak_uart_set_callback(
    dspic33ak_uart_instance_t inst,
    dspic33ak_uart_event_callback_t callback,
    void *user_data);

/* ----- TX / RX line enable control ---------------------------------------- */

/*
 * Enable or disable TX.
 *
 * Disabling TX while an async TX transfer is active returns
 * DSPIC33AK_UART_ERR_BUSY; abort or wait for completion first.
 */
dspic33ak_uart_status_t dspic33ak_uart_tx_enable(
    dspic33ak_uart_instance_t inst,
    bool enable);

/*
 * Enable or disable RX.
 *
 * Disabling RX while an async RX transfer is active returns
 * DSPIC33AK_UART_ERR_BUSY; abort or wait for completion first.
 */
dspic33ak_uart_status_t dspic33ak_uart_rx_enable(
    dspic33ak_uart_instance_t inst,
    bool enable);

bool dspic33ak_uart_tx_is_enabled(
    dspic33ak_uart_instance_t inst);

bool dspic33ak_uart_rx_is_enabled(
    dspic33ak_uart_instance_t inst);

/* ----- Baud rate (re)configuration ---------------------------------------- */

/*
 * Recompute and apply the baud divisor from uart_clk_hz / baudrate and remember
 * both in the instance context. Rejected with DSPIC33AK_UART_ERR_BUSY while a
 * byte is in flight or an async TX/RX transfer is active, so an in-progress
 * transfer is never silently reconfigured. _ERR_INVALID_ARG on a zero/invalid
 * clock or baud.
 */
dspic33ak_uart_status_t dspic33ak_uart_set_baudrate(
    dspic33ak_uart_instance_t inst,
    uint32_t uart_clk_hz,
    uint32_t baudrate);

/* Last baud rate applied (0 if the instance is not initialized). */
uint32_t dspic33ak_uart_get_baudrate(
    dspic33ak_uart_instance_t inst);

/* ----- Non-blocking TX transfer ------------------------------------------- */

/*
 * Start a non-blocking TX transfer of length bytes from data. Returns
 * immediately; the bytes are pushed to the TX FIFO from the TX interrupt. When
 * the last byte has been submitted to the hardware, DSPIC33AK_UART_EVENT_SEND_COMPLETE
 * is reported via the callback (CMSIS SEND_COMPLETE sense, not physical shift-out;
 * use dspic33ak_uart_tx_done() for that). data must remain valid until completion
 * or abort.
 *
 *   _ERR_INVALID_ARG  data == NULL or length == 0
 *   _ERR_BUSY         a TX transfer is already active
 *   _ERR_UNSUPPORTED  TX disabled or tx_irq_priority == 0
 *   _ERR_NOT_INITIALIZED / _ERR_NOT_PRESENT as usual
 *
 * This is independent of dspic33ak_uart_write()/_write_byte(); do not mix a
 * blocking write with an active async TX transfer on the same instance.
 */
dspic33ak_uart_status_t dspic33ak_uart_tx_start(
    dspic33ak_uart_instance_t inst,
    const uint8_t *data,
    size_t length);

/* Abort an active TX transfer. Already-submitted bytes still go out; no
 * TX_COMPLETE event is reported. Safe to call when idle. */
dspic33ak_uart_status_t dspic33ak_uart_tx_abort(
    dspic33ak_uart_instance_t inst);

/* Number of bytes submitted by the current/last TX transfer. */
size_t dspic33ak_uart_tx_count_get(
    dspic33ak_uart_instance_t inst);

bool dspic33ak_uart_tx_is_busy(
    dspic33ak_uart_instance_t inst);

/* ----- Non-blocking RX transfer ------------------------------------------- */

/*
 * Register a non-blocking RX transfer: up to length bytes are stored into data
 * as they arrive (fed from the RX ISR, ISR ring mode only). Returns immediately.
 * When length bytes have been received, DSPIC33AK_UART_EVENT_RX_COMPLETE is
 * reported via the callback. data must remain valid until completion or abort.
 *
 * While a transfer is active, incoming bytes go to the async buffer instead of
 * the RX ISR ring (so dspic33ak_uart_read_byte() will not see them).
 *
 * BY DESIGN, the transfer captures only bytes that arrive AFTER this call: bytes
 * already buffered in the RX ISR ring before rx_start() are NOT drained into the
 * async buffer and stay readable via the byte-stream API. A caller that wants the
 * async transfer to start from a clean slate should use
 * dspic33ak_uart_rx_start_clean(), which avoids a flush/start race window.
 *
 *   _ERR_INVALID_ARG  data == NULL or length == 0
 *   _ERR_BUSY         an RX transfer is already active
 *   _ERR_UNSUPPORTED  instance is not in ISR ring RX mode, or RX is disabled
 *   _ERR_NOT_INITIALIZED / _ERR_NOT_PRESENT as usual
 */
dspic33ak_uart_status_t dspic33ak_uart_rx_start(
    dspic33ak_uart_instance_t inst,
    uint8_t *data,
    size_t length);

/*
 * Start a clean non-blocking RX transfer. Bytes already buffered in the RX ISR
 * ring or hardware FIFO are discarded, then the async RX descriptor is armed
 * while the RX interrupt is held disabled.
 *
 * Bytes that arrive after the clean arm are captured by the new async transfer.
 * The exact boundary is the end of the FIFO drain / descriptor publication, not
 * the function entry point.
 *
 * This is intended for APIs such as CMSIS USART Receive(), where each receive
 * operation should observe only bytes that arrive for that operation.
 *
 * Return codes match dspic33ak_uart_rx_start(); _ERR_UNSUPPORTED is also
 * returned if the RX ISR is not currently enabled.
 */
dspic33ak_uart_status_t dspic33ak_uart_rx_start_clean(
    dspic33ak_uart_instance_t inst,
    uint8_t *data,
    size_t length);

/* Abort an active RX transfer. No RX_COMPLETE event is reported. Already-stored
 * bytes stay in the caller buffer (count readable via _rx_count_get). Safe when
 * idle. Subsequent incoming bytes resume going to the RX ISR ring. */
dspic33ak_uart_status_t dspic33ak_uart_rx_abort(
    dspic33ak_uart_instance_t inst);

/* Number of bytes stored by the current/last RX transfer. */
size_t dspic33ak_uart_rx_count_get(
    dspic33ak_uart_instance_t inst);

bool dspic33ak_uart_rx_is_busy(
    dspic33ak_uart_instance_t inst);

/* ----- Interrupt entry points --------------------------------------------- */

/*
 * TX interrupt service routine body, the TX counterpart of
 * dspic33ak_uart_rx_irq_handler(). Called from the application-owned
 * _UxTXInterrupt vector. This is an ordinary function, NOT an interrupt vector.
 * It refills the TX FIFO from the active transfer and, on the last byte,
 * disables the TX interrupt and reports DSPIC33AK_UART_EVENT_SEND_COMPLETE.
 */
void dspic33ak_uart_tx_irq_handler(
    dspic33ak_uart_instance_t inst);

/* NOTE: the HAL-internal glue between the RX ISR ring core and the async transfer
 * engine (dspic33ak_uart_async_rx_feed / dspic33ak_uart_async_notify_events) is
 * declared in dspic33ak_uart_rx_isr_ring.h, not here. It is not part of the public
 * API and must not be called from application code. */

/* ========================================================================== */
/* C++ Linkage                                                                */
/* ========================================================================== */

#ifdef __cplusplus
}
#endif

#endif /* DSPIC33AK_UART_H */

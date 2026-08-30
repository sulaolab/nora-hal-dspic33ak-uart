#ifndef NORA_UART_H
#define NORA_UART_H

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
 * Small, readable NORA UART byte-stream HAL.
 *
 * Clock assumption:
 *   - The caller configures the selected UART hardware clock and passes its resulting
 *     frequency as config.uart_clk_hz; the HAL computes and applies the baud divisor.
 *   - Board/application code owns clock-generator setup, pin routing and GPIO
 *     configuration before calling nora_uart_init().
 *
 * Design policy:
 *   - Public API does not expose XC-DSC / DFP bitfield types.
 *   - Backend register names, interrupt mappings and device conditionals stay in
 *     the selected implementation backend.
 *   - Board-specific clock / PPS / GPIO routing stays outside this HAL.
 *   - stdio retarget, console parsing, and application command handling stay
 *     outside this HAL.
 *   - The application/board selects nora_uart_instance_t. The HAL does not assert
 *     that a given logical UART instance has the same physical routing on every
 *     NORA-supported device.
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
    NORA_UART_INST_1 = 0,
    NORA_UART_INST_2,
    NORA_UART_INST_3,
    NORA_UART_INST_4,
    NORA_UART_INST_COUNT
} nora_uart_instance_t;

/* ========================================================================== */
/* Per-instance state width (project-configurable)                            */
/* ========================================================================== */

/*
 * NORA_UART_HW_INST_MAX is the enum above, as a preprocessor literal. It has to
 * be a macro and not NORA_UART_INST_COUNT: an enumerator is invisible to #if,
 * which evaluates the unknown identifier as 0, so a range check written against
 * the enum silently compares against zero. The _Static_assert below is what
 * keeps the two from drifting apart.
 */
#define NORA_UART_HW_INST_MAX   4

/*
 * Project-supplied compile-time config, optional. The HAL ships no conf.h, so a
 * project that vendors hal_uart without one keeps the documented default below
 * (state for every instance the enum has -- the behaviour this driver has always
 * had) instead of failing to compile on a missing include.
 *
 * NORA_UART_INST_SUPPORTED_COUNT narrows only the SIZE of the driver's
 * per-instance arrays. The enum, the API and every prototype are unchanged;
 * instances at or above the count report themselves absent
 * (nora_uart_is_present() == false) rather than being silently accepted, so the
 * narrowing cannot turn into an out-of-range write. See
 * board/uart/nora_uart_conf.h for this product's value and the reasoning.
 */
#if !defined( NORA_UART_INST_SUPPORTED_COUNT )
#  if defined( __has_include )
#    if __has_include( "nora_uart_conf.h" )
#      include "nora_uart_conf.h"
#    endif
#  endif
#endif

#ifndef NORA_UART_INST_SUPPORTED_COUNT
#define NORA_UART_INST_SUPPORTED_COUNT   NORA_UART_HW_INST_MAX
#endif

#if (NORA_UART_INST_SUPPORTED_COUNT < 1) || \
    (NORA_UART_INST_SUPPORTED_COUNT > NORA_UART_HW_INST_MAX)
#error "NORA_UART_INST_SUPPORTED_COUNT must be 1..NORA_UART_HW_INST_MAX."
#endif

_Static_assert( (int)NORA_UART_INST_COUNT == NORA_UART_HW_INST_MAX,
                "NORA_UART_HW_INST_MAX must match nora_uart_instance_t" );

typedef enum {
    NORA_UART_OK = 0,
    NORA_UART_ERR_INVALID_ARG,
    NORA_UART_ERR_NOT_PRESENT,
    NORA_UART_ERR_NOT_INITIALIZED,
    NORA_UART_ERR_BUSY,
    NORA_UART_ERR_TIMEOUT,
    NORA_UART_ERR_RX_EMPTY,
    NORA_UART_ERR_TX_FULL,
    NORA_UART_ERR_OVERRUN,
    NORA_UART_ERR_FRAMING,
    NORA_UART_ERR_PARITY,
    NORA_UART_ERR_UNSUPPORTED
} nora_uart_status_t;

typedef uint32_t (*nora_uart_get_ms_fn)(void);

typedef enum {
    NORA_UART_PARITY_NONE = 0,
    NORA_UART_PARITY_EVEN,
    NORA_UART_PARITY_ODD
} nora_uart_parity_t;

/*
 * RX backend selection (per instance).
 *
 *   POLLING  - RX is read directly from the hardware RX FIFO; no RX interrupt is
 *              enabled. The rx_ring_* / rx_irq_priority config fields are ignored.
 *   ISR_RING - nora_uart_init() sets up the interrupt-driven RX ring: the RX ISR
 *              drains the FIFO into a caller-provided software ring, and
 *              rx_ready/read_byte/rx_flush operate on that ring instead of the
 *              FIFO. Requires rx_ring_buffer != NULL,
 *              rx_ring_buffer_size >= 2, and uses rx_irq_priority.
 *
 * Selecting the backend per instance (rather than one global compile-time switch)
 * lets a build mix modes, e.g. UART1 console = ISR ring, UART2 log = polling.
 *
 * Only POLLING and ISR_RING are valid; nora_uart_init() rejects any other
 * rx_mode value with NORA_UART_ERR_INVALID_ARG.
 */
typedef enum {
    NORA_UART_RX_MODE_POLLING = 0,
    NORA_UART_RX_MODE_ISR_RING
} nora_uart_rx_mode_t;

typedef struct {
    uint32_t uart_clk_hz;
    uint32_t baudrate;
    uint32_t timeout_ms;

    /*
     * Optional millisecond tick callback for timeout handling.
     * If get_ms is NULL, timeout handling is disabled.
     * If timeout_ms is 0, timeout handling is also disabled.
     */
    nora_uart_get_ms_fn get_ms;

    uint8_t data_bits;
    uint8_t stop_bits;
    nora_uart_parity_t parity;
    bool enable_tx;
    bool enable_rx;

    /* RX backend (see nora_uart_rx_mode_t). The rx_ring_* / rx_irq_priority
     * fields are used only when rx_mode == NORA_UART_RX_MODE_ISR_RING. The
     * ring buffer storage is caller-provided so the HAL holds no implicit RAM. */
    nora_uart_rx_mode_t rx_mode;
    uint8_t  *rx_ring_buffer;
    uint16_t  rx_ring_buffer_size;
    uint8_t   rx_irq_priority;

    /*
     * CPU interrupt priority for the TX interrupt, used only by the non-blocking
     * TX transfer engine (nora_uart_tx_start). It is programmed at init and
     * is independent of rx_irq_priority. Builds that never call the async TX API
     * may leave this at any value; the TX interrupt stays disabled until a
     * transfer starts. A value of 0 disables the TX interrupt on this CPU, so an
     * async-TX user must set a non-zero priority here.
     */
    uint8_t   tx_irq_priority;
} nora_uart_config_t;

/*
 * RX runtime status snapshot (backend-aware).
 *
 * This is different from nora_uart_status_t:
 *   - nora_uart_status_t      is a function return code.
 *   - nora_uart_rx_status_t   is runtime RX state / counters.
 *
 * In ISR ring mode the counters are copied from the RX ISR ring backend. In
 * polling mode rx_mode is POLLING and the backend-specific counters are zero
 * (the polling path keeps no counters). Lets callers read RX diagnostics without
 * knowing or branching on the configured backend.
 */
typedef struct {
    nora_uart_rx_mode_t rx_mode;

    uint32_t rx_isr_count;
    uint32_t rx_byte_count;
    uint32_t rx_fifo_overflow_count;
    uint32_t framing_error_count;
    uint32_t parity_error_count;
    uint32_t autobaud_overflow_count;
    uint32_t tx_collision_count;
    uint32_t rx_ring_overflow_count;
    uint32_t rx_max_drain_count;

    /*
     * Cross-silicon RX recovery diagnostics. The dsPIC33CK backend records
     * these recovery paths; dsPIC33AK has no corresponding paths and reports
     * zero for all three counters.
     */
    uint32_t rx_stall_recovery_count;
    uint32_t rx_ie_lost_count;
    uint32_t rx_overrun_recovered_count;
} nora_uart_rx_status_t;

/* ========================================================================== */
/* Public API                                                                 */
/* ========================================================================== */

nora_uart_status_t nora_uart_init(
    nora_uart_instance_t inst,
    const nora_uart_config_t *config);

nora_uart_status_t nora_uart_deinit(
    nora_uart_instance_t inst);

bool nora_uart_is_present(
    nora_uart_instance_t inst);

bool nora_uart_is_initialized(
    nora_uart_instance_t inst);

bool nora_uart_rx_ready(
    nora_uart_instance_t inst);

bool nora_uart_tx_ready(
    nora_uart_instance_t inst);

bool nora_uart_tx_done(
    nora_uart_instance_t inst);

nora_uart_status_t nora_uart_write_byte(
    nora_uart_instance_t inst,
    uint8_t data);

nora_uart_status_t nora_uart_read_byte(
    nora_uart_instance_t inst,
    uint8_t *data);

size_t nora_uart_write(
    nora_uart_instance_t inst,
    const void *data,
    size_t len);

size_t nora_uart_read(
    nora_uart_instance_t inst,
    void *data,
    size_t len);

void nora_uart_rx_flush(
    nora_uart_instance_t inst);

/*
 * Backend-aware RX status snapshot / clear.
 *
 * ISR ring mode reports/clears the RX ISR ring counters; polling mode reports a
 * zeroed snapshot (rx_mode = POLLING) and clear is a no-op. Callers use these
 * instead of backend-specific diagnostic helpers so they stay backend-agnostic.
 *
 * Returns NORA_UART_ERR_INVALID_ARG (status NULL), _ERR_NOT_PRESENT,
 * _ERR_NOT_INITIALIZED, or NORA_UART_OK.
 */
nora_uart_status_t nora_uart_rx_status_get(
    nora_uart_instance_t inst,
    nora_uart_rx_status_t *status);

nora_uart_status_t nora_uart_rx_status_clear(
    nora_uart_instance_t inst);

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
 * inert until nora_uart_tx_start(), nora_uart_rx_start(), or
 * nora_uart_rx_start_clean() is called.
 *
 * Intentionally generic: the events and the API below describe a UART, not any
 * specific middleware. No ARM_USART_* / ARM_DRIVER_* names appear here.
 *
 * Backend requirements:
 *   - Async TX requires TX enabled and a non-zero tx_irq_priority; otherwise
 *     nora_uart_tx_start() returns NORA_UART_ERR_UNSUPPORTED (a transfer
 *     with no servicing interrupt would never complete). It also requires the
 *     application to route the device TX interrupt vector to
 *     nora_uart_tx_irq_handler(), as the RX vector forwards to
 *     nora_uart_rx_irq_handler().
 *   - Async RX requires RX enabled and NORA_UART_RX_MODE_ISR_RING (the RX ISR
 *     feeds the async buffer); otherwise nora_uart_rx_start() returns
 *     NORA_UART_ERR_UNSUPPORTED.
 *   - nora_uart_tx_enable(false) / nora_uart_rx_enable(false) return
 *     NORA_UART_ERR_BUSY while an async transfer is active, so a transfer is
 *     never stranded by disabling its line mid-flight.
 */

/* Event bit-flags reported through the registered callback. Multiple bits may be
 * OR'd together in a single notification.
 *
 * SEND_COMPLETE means the driver has submitted every TX byte to the hardware
 * (FIFO/register) - the CMSIS ARM_USART_EVENT_SEND_COMPLETE sense, NOT physical
 * shift-register-empty. It is intentionally NOT named TX_COMPLETE to avoid being
 * read as the CMSIS ARM_USART_EVENT_TX_COMPLETE (line idle / shifter empty). Use
 * the existing nora_uart_tx_done() to confirm physical transmit completion. */
#define NORA_UART_EVENT_SEND_COMPLETE     (1u << 0)  /* all TX data submitted (SEND_COMPLETE) */
#define NORA_UART_EVENT_RX_COMPLETE       (1u << 1)  /* requested RX length got */
#define NORA_UART_EVENT_RX_READY          (1u << 2)  /* unsolicited RX -> ring  */
#define NORA_UART_EVENT_RX_OVERFLOW       (1u << 3)  /* software RX ring overflow */
#define NORA_UART_EVENT_RX_FRAMING_ERROR  (1u << 4)  /* UxSTAT FERIF            */
#define NORA_UART_EVENT_RX_PARITY_ERROR   (1u << 5)  /* UxSTAT PERIF            */
#define NORA_UART_EVENT_RX_OVERRUN_ERROR  (1u << 6)  /* hardware RX FIFO overrun */

/*
 * Event callback. Invoked with the OR'd event bits for the instance and the
 * user_data pointer registered alongside it.
 *
 * NOTE (initial version): the callback may be invoked from interrupt context
 * (TX/RX ISR). Keep it short and non-blocking; do not call back into a blocking
 * HAL API from inside it.
 */
typedef void (*nora_uart_event_callback_t)(
    nora_uart_instance_t inst,
    uint32_t events,
    void *user_data);

/*
 * Register (or clear, with callback == NULL) the event callback for an instance.
 * Valid before or after init; nora_uart_init()/deinit() clear it, so call
 * this after init. Returns _ERR_INVALID_ARG / _ERR_NOT_PRESENT or _OK.
 *
 * The registered callback is invoked from interrupt context (TX/RX ISR). It must
 * be short and non-blocking, and must not call back into a blocking HAL API
 * (for example nora_uart_write_byte() with a timeout).
 */
nora_uart_status_t nora_uart_set_callback(
    nora_uart_instance_t inst,
    nora_uart_event_callback_t callback,
    void *user_data);

/* ----- TX / RX line enable control ---------------------------------------- */

/*
 * Enable or disable TX.
 *
 * Disabling TX while an async TX transfer is active returns
 * NORA_UART_ERR_BUSY; abort or wait for completion first.
 */
nora_uart_status_t nora_uart_tx_enable(
    nora_uart_instance_t inst,
    bool enable);

/*
 * Enable or disable RX.
 *
 * Disabling RX while an async RX transfer is active returns
 * NORA_UART_ERR_BUSY; abort or wait for completion first.
 */
nora_uart_status_t nora_uart_rx_enable(
    nora_uart_instance_t inst,
    bool enable);

bool nora_uart_tx_is_enabled(
    nora_uart_instance_t inst);

bool nora_uart_rx_is_enabled(
    nora_uart_instance_t inst);

/* ----- Baud rate (re)configuration ---------------------------------------- */

/*
 * Recompute and apply the baud divisor from uart_clk_hz / baudrate and remember
 * both in the instance context. Rejected with NORA_UART_ERR_BUSY while a
 * byte is in flight or an async TX/RX transfer is active, so an in-progress
 * transfer is never silently reconfigured. _ERR_INVALID_ARG on a zero/invalid
 * clock or baud.
 */
nora_uart_status_t nora_uart_set_baudrate(
    nora_uart_instance_t inst,
    uint32_t uart_clk_hz,
    uint32_t baudrate);

/* Last baud rate applied (0 if the instance is not initialized). */
uint32_t nora_uart_get_baudrate(
    nora_uart_instance_t inst);

/* ----- Non-blocking TX transfer ------------------------------------------- */

/*
 * Start a non-blocking TX transfer of length bytes from data. Returns
 * immediately; the bytes are pushed to the TX FIFO from the TX interrupt. When
 * the last byte has been submitted to the hardware, NORA_UART_EVENT_SEND_COMPLETE
 * is reported via the callback (CMSIS SEND_COMPLETE sense, not physical shift-out;
 * use nora_uart_tx_done() for that). data must remain valid until completion
 * or abort.
 *
 *   _ERR_INVALID_ARG  data == NULL or length == 0
 *   _ERR_BUSY         a TX transfer is already active
 *   _ERR_UNSUPPORTED  TX disabled or tx_irq_priority == 0
 *   _ERR_NOT_INITIALIZED / _ERR_NOT_PRESENT as usual
 *
 * This is independent of nora_uart_write()/_write_byte(); do not mix a
 * blocking write with an active async TX transfer on the same instance.
 */
nora_uart_status_t nora_uart_tx_start(
    nora_uart_instance_t inst,
    const uint8_t *data,
    size_t length);

/* Abort an active TX transfer. Already-submitted bytes still go out; no
 * TX_COMPLETE event is reported. Safe to call when idle. */
nora_uart_status_t nora_uart_tx_abort(
    nora_uart_instance_t inst);

/* Number of bytes submitted by the current/last TX transfer. */
size_t nora_uart_tx_count_get(
    nora_uart_instance_t inst);

bool nora_uart_tx_is_busy(
    nora_uart_instance_t inst);

/* ----- Non-blocking RX transfer ------------------------------------------- */

/*
 * Register a non-blocking RX transfer: up to length bytes are stored into data
 * as they arrive (fed from the RX ISR, ISR ring mode only). Returns immediately.
 * When length bytes have been received, NORA_UART_EVENT_RX_COMPLETE is
 * reported via the callback. data must remain valid until completion or abort.
 *
 * While a transfer is active, incoming bytes go to the async buffer instead of
 * the RX ISR ring (so nora_uart_read_byte() will not see them).
 *
 * BY DESIGN, the transfer captures only bytes that arrive AFTER this call: bytes
 * already buffered in the RX ISR ring before rx_start() are NOT drained into the
 * async buffer and stay readable via the byte-stream API. A caller that wants the
 * async transfer to start from a clean slate should use
 * nora_uart_rx_start_clean(), which avoids a flush/start race window.
 *
 *   _ERR_INVALID_ARG  data == NULL or length == 0
 *   _ERR_BUSY         an RX transfer is already active
 *   _ERR_UNSUPPORTED  instance is not in ISR ring RX mode, or RX is disabled
 *   _ERR_NOT_INITIALIZED / _ERR_NOT_PRESENT as usual
 */
nora_uart_status_t nora_uart_rx_start(
    nora_uart_instance_t inst,
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
 * Return codes match nora_uart_rx_start(); _ERR_UNSUPPORTED is also
 * returned if the RX ISR is not currently enabled.
 */
nora_uart_status_t nora_uart_rx_start_clean(
    nora_uart_instance_t inst,
    uint8_t *data,
    size_t length);

/* Abort an active RX transfer. No RX_COMPLETE event is reported. Already-stored
 * bytes stay in the caller buffer (count readable via _rx_count_get). Safe when
 * idle. Subsequent incoming bytes resume going to the RX ISR ring. */
nora_uart_status_t nora_uart_rx_abort(
    nora_uart_instance_t inst);

/* Number of bytes stored by the current/last RX transfer. */
size_t nora_uart_rx_count_get(
    nora_uart_instance_t inst);

bool nora_uart_rx_is_busy(
    nora_uart_instance_t inst);

/* ----- Interrupt entry points --------------------------------------------- */

/*
 * RX interrupt service routine body. Called from the board/application-owned
 * RX interrupt vector. This is an ordinary function, NOT an interrupt vector.
 */
void nora_uart_rx_irq_handler(
    nora_uart_instance_t inst);

/*
 * TX interrupt service routine body. Called from the board/application-owned
 * TX interrupt vector. This is an ordinary function, NOT an interrupt vector.
 * It refills the TX FIFO from the active transfer and, on the last byte,
 * disables the TX interrupt and reports NORA_UART_EVENT_SEND_COMPLETE.
 */
void nora_uart_tx_irq_handler(
    nora_uart_instance_t inst);

/* HAL-internal RX-ring and asynchronous-transfer glue is intentionally not part of
 * this public API. */

/* ========================================================================== */
/* C++ Linkage                                                                */
/* ========================================================================== */

#ifdef __cplusplus
}
#endif

#endif /* NORA_UART_H */

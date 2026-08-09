/* ========================================================================== */
/* Includes                                                                   */
/* ========================================================================== */

#include <string.h>

#include "dspic33ak_uart.h"
#include "dspic33ak_uart_rx_isr_ring.h"
#include "dspic33ak_uart_device.h"
#include "dspic33ak_uart_reg.h"

/* ========================================================================== */
/* Module Variables                                                           */
/* ========================================================================== */

static uint32_t g_timeout_ms[DSPIC33AK_UART_INST_COUNT];
static dspic33ak_uart_get_ms_fn g_get_ms[DSPIC33AK_UART_INST_COUNT];
static bool g_initialized[DSPIC33AK_UART_INST_COUNT];

/* Per-instance RX backend (defaults to polling; set from config at init). The
 * RX query/read/flush API consults this to pick the FIFO or the ISR ring. */
static dspic33ak_uart_rx_mode_t g_rx_mode[DSPIC33AK_UART_INST_COUNT];

/* ---- Asynchronous transfer model state (per instance) -------------------- *
 * All of this is inert until dspic33ak_uart_tx_start(), dspic33ak_uart_rx_start(),
 * or dspic33ak_uart_rx_start_clean() is used, so it does not affect the
 * byte-stream API or the RX ISR ring.
 *
 * Sharing with interrupt context: the TX ISR updates g_tx_count/g_tx_busy and
 * the RX ISR updates g_rx_count/g_rx_busy, so only those counters and flags are
 * volatile. The buffer pointer and length are written once (before the busy
 * flag is set and the interrupt is enabled) and only read afterwards, so they
 * do not need to be volatile. */
static dspic33ak_uart_event_callback_t g_callback[DSPIC33AK_UART_INST_COUNT];
static void *g_callback_user_data[DSPIC33AK_UART_INST_COUNT];

static uint32_t g_uart_clk_hz[DSPIC33AK_UART_INST_COUNT];
static uint32_t g_baudrate[DSPIC33AK_UART_INST_COUNT];
static bool     g_tx_enabled[DSPIC33AK_UART_INST_COUNT];
static bool     g_rx_enabled[DSPIC33AK_UART_INST_COUNT];
static uint8_t  g_tx_irq_priority[DSPIC33AK_UART_INST_COUNT];

static const uint8_t   *g_tx_buf[DSPIC33AK_UART_INST_COUNT];
static size_t           g_tx_len[DSPIC33AK_UART_INST_COUNT];
static volatile size_t  g_tx_count[DSPIC33AK_UART_INST_COUNT];
static volatile bool    g_tx_busy[DSPIC33AK_UART_INST_COUNT];

static uint8_t         *g_rx_buf[DSPIC33AK_UART_INST_COUNT];
static size_t           g_rx_len[DSPIC33AK_UART_INST_COUNT];
static volatile size_t  g_rx_count[DSPIC33AK_UART_INST_COUNT];
static volatile bool    g_rx_busy[DSPIC33AK_UART_INST_COUNT];

/* ========================================================================== */
/* Local Function Prototypes                                                  */
/* ========================================================================== */

static bool uart_inst_is_valid(dspic33ak_uart_instance_t inst);
static dspic33ak_uart_status_t uart_get_regs(
    dspic33ak_uart_instance_t inst,
    const dspic33ak_uart_regs_t **regs);
static dspic33ak_uart_status_t uart_require_initialized(
    dspic33ak_uart_instance_t inst,
    const dspic33ak_uart_regs_t **regs);
static dspic33ak_uart_status_t uart_check_initialized(
    dspic33ak_uart_instance_t inst);
static uint32_t uart_calc_brg(
    uint32_t uart_clk_hz,
    uint32_t baudrate);
static bool uart_timeout_enabled(
    dspic33ak_uart_instance_t inst);
static uint32_t uart_timeout_start_ms(
    dspic33ak_uart_instance_t inst);
static bool uart_timeout_expired(
    dspic33ak_uart_instance_t inst,
    uint32_t start_ms);

static void uart_async_reset(dspic33ak_uart_instance_t inst);
static void uart_rx_arm(
    dspic33ak_uart_instance_t inst,
    uint8_t *data,
    size_t length);
static void uart_notify(dspic33ak_uart_instance_t inst, uint32_t events);

static bool uart_tx_irq_set_priority(dspic33ak_uart_instance_t inst, uint8_t prio);
static bool uart_tx_irq_clear_flag(dspic33ak_uart_instance_t inst);
static bool uart_tx_irq_raise_flag(dspic33ak_uart_instance_t inst);
static bool uart_tx_irq_enable(dspic33ak_uart_instance_t inst, bool enable);
static bool uart_rx_irq_enable(dspic33ak_uart_instance_t inst, bool enable);
static uint8_t uart_rx_irq_get_enable(dspic33ak_uart_instance_t inst);

/* ========================================================================== */
/* Public Functions                                                           */
/* ========================================================================== */

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_init                                                        */
/* -------------------------------------------------------------------------- */
dspic33ak_uart_status_t dspic33ak_uart_init(
    dspic33ak_uart_instance_t inst,
    const dspic33ak_uart_config_t *config)
{
    const dspic33ak_uart_regs_t *r;
    dspic33ak_uart_status_t st;
    uint32_t brg;

    if (!uart_inst_is_valid(inst)) {
        return DSPIC33AK_UART_ERR_INVALID_ARG;
    }

    if (config == 0 || config->uart_clk_hz == 0u || config->baudrate == 0u) {
        return DSPIC33AK_UART_ERR_INVALID_ARG;
    }

    /* Current implementation supports 8N1 (UxCON MODE=0 reset default). */
    if (config->data_bits != 8u ||
        config->stop_bits != 1u ||
        config->parity != DSPIC33AK_UART_PARITY_NONE) {
        return DSPIC33AK_UART_ERR_UNSUPPORTED;
    }

    /* RX backend must be a known mode. Reject unknown/uninitialized rx_mode here,
     * before touching any register, so a bad config never silently falls through
     * to polling and never leaves the UART peripheral half-configured. */
    if ((config->rx_mode != DSPIC33AK_UART_RX_MODE_POLLING) &&
        (config->rx_mode != DSPIC33AK_UART_RX_MODE_ISR_RING)) {
        return DSPIC33AK_UART_ERR_INVALID_ARG;
    }

    st = uart_get_regs(inst, &r);
    if (st != DSPIC33AK_UART_OK) {
        return st;
    }

    brg = uart_calc_brg(config->uart_clk_hz, config->baudrate);
    if (brg == 0u) {
        return DSPIC33AK_UART_ERR_INVALID_ARG;
    }

    /* Turn the module off and start from a known 8N1 state. */
    *r->CON = 0u;
    *r->STAT = 0u;

    /* Clock: select CLKGEN8 (CLKSEL) and fractional baud mode (CLKMOD; BRGS stays
     * 0). This HAL assumes CLKGEN8 as the UART clock source; the board/application
     * must have brought CLKGEN8 up before init, and config.uart_clk_hz must be the
     * CLKGEN8 frequency used for the baud-divisor calculation. */
    dspic33ak_uart_reg_set(r->CON, DSPIC33AK_UART_CON_CLKSEL);
    dspic33ak_uart_reg_set(r->CON, DSPIC33AK_UART_CON_CLKMOD);

    *r->BRG = brg;

    /* TX FIFO write enable. */
    dspic33ak_uart_reg_set(r->STAT, DSPIC33AK_UART_STAT_TXWRE);

    if (config->enable_tx) {
        dspic33ak_uart_reg_set(r->CON, DSPIC33AK_UART_CON_TXEN);
    }
    if (config->enable_rx) {
        dspic33ak_uart_reg_set(r->CON, DSPIC33AK_UART_CON_RXEN);
    }

    /* Enable the module last. */
    dspic33ak_uart_reg_set(r->CON, DSPIC33AK_UART_CON_ON);

    g_timeout_ms[inst] = config->timeout_ms;
    g_get_ms[inst] = config->get_ms;
    g_initialized[inst] = true;
    g_rx_mode[inst] = config->rx_mode;

    /* Async transfer model context: remember clock/baud and the TX/RX enable
     * state, start with a cleared callback and no active transfer, and program
     * the (disabled) TX interrupt priority for the async TX engine. */
    g_uart_clk_hz[inst]     = config->uart_clk_hz;
    g_baudrate[inst]        = config->baudrate;
    g_tx_enabled[inst]      = config->enable_tx;
    g_rx_enabled[inst]      = config->enable_rx;
    g_tx_irq_priority[inst] = config->tx_irq_priority;
    g_callback[inst]           = 0;
    g_callback_user_data[inst] = 0;
    uart_async_reset(inst);

    (void)uart_tx_irq_enable(inst, false);
    (void)uart_tx_irq_clear_flag(inst);
    (void)uart_tx_irq_set_priority(inst, config->tx_irq_priority);

    /*
     * RX backend setup. ISR ring mode configures and enables the interrupt-driven
     * RX ring now. dspic33ak_uart_rx_isr_config() requires the instance to be
     * initialized, so this runs after g_initialized = true. On failure, unwind via
     * deinit so a half-initialized instance is never left behind.
     */
    if (config->rx_mode == DSPIC33AK_UART_RX_MODE_ISR_RING) {
        dspic33ak_uart_rx_isr_config_t rx_cfg;

        rx_cfg.buffer       = config->rx_ring_buffer;
        rx_cfg.buffer_size  = config->rx_ring_buffer_size;
        rx_cfg.irq_priority = config->rx_irq_priority;

        st = dspic33ak_uart_rx_isr_config(inst, &rx_cfg);
        if (st != DSPIC33AK_UART_OK) {
            (void)dspic33ak_uart_deinit(inst);
            return st;
        }

        st = dspic33ak_uart_rx_isr_enable(inst);
        if (st != DSPIC33AK_UART_OK) {
            (void)dspic33ak_uart_deinit(inst);
            return st;
        }
    }

    return DSPIC33AK_UART_OK;
}

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_deinit                                                      */
/* -------------------------------------------------------------------------- */
dspic33ak_uart_status_t dspic33ak_uart_deinit(
    dspic33ak_uart_instance_t inst)
{
    const dspic33ak_uart_regs_t *r;
    dspic33ak_uart_status_t st;

    if (!uart_inst_is_valid(inst)) {
        return DSPIC33AK_UART_ERR_INVALID_ARG;
    }

    st = uart_get_regs(inst, &r);
    if (st != DSPIC33AK_UART_OK) {
        return st;
    }

    /* Stop the RX ISR first (if this instance ran the ring) so it cannot touch
     * the FIFO mid-teardown. Safe/no-op in polling mode. */
    if (g_rx_mode[inst] == DSPIC33AK_UART_RX_MODE_ISR_RING) {
        (void)dspic33ak_uart_rx_isr_disable(inst);
    }

    /* Stop the async TX engine too (no-op if it was never used). */
    (void)uart_tx_irq_enable(inst, false);
    (void)uart_tx_irq_clear_flag(inst);

    dspic33ak_uart_reg_clear(r->CON,
                             DSPIC33AK_UART_CON_TXEN |
                             DSPIC33AK_UART_CON_RXEN |
                             DSPIC33AK_UART_CON_ON);

    g_timeout_ms[inst] = 0u;
    g_get_ms[inst] = 0;
    g_initialized[inst] = false;
    g_rx_mode[inst] = DSPIC33AK_UART_RX_MODE_POLLING;

    /* Drop async transfer model state. */
    g_uart_clk_hz[inst]        = 0u;
    g_baudrate[inst]           = 0u;
    g_tx_enabled[inst]         = false;
    g_rx_enabled[inst]         = false;
    g_tx_irq_priority[inst]    = 0u;
    g_callback[inst]           = 0;
    g_callback_user_data[inst] = 0;
    uart_async_reset(inst);

    return DSPIC33AK_UART_OK;
}

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_is_present                                                  */
/* -------------------------------------------------------------------------- */
bool dspic33ak_uart_is_present(dspic33ak_uart_instance_t inst)
{
    return dspic33ak_uart_instance_is_present(inst);
}

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_is_initialized                                              */
/* -------------------------------------------------------------------------- */
bool dspic33ak_uart_is_initialized(dspic33ak_uart_instance_t inst)
{
    if (!uart_inst_is_valid(inst)) {
        return false;
    }

    return g_initialized[inst];
}

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_rx_ready                                                    */
/* -------------------------------------------------------------------------- */
bool dspic33ak_uart_rx_ready(dspic33ak_uart_instance_t inst)
{
    const dspic33ak_uart_regs_t *r;

    if (uart_require_initialized(inst, &r) != DSPIC33AK_UART_OK) {
        return false;
    }

    /* ISR ring backend: readiness reflects buffered ring contents. */
    if (g_rx_mode[inst] == DSPIC33AK_UART_RX_MODE_ISR_RING) {
        return dspic33ak_uart_rx_isr_ready(inst);
    }

    /* Polling backend: RX has data when the RX FIFO is NOT empty. */
    return !dspic33ak_uart_reg_is_set(r->STAT, DSPIC33AK_UART_STAT_RXBE);
}

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_tx_ready                                                    */
/* -------------------------------------------------------------------------- */
bool dspic33ak_uart_tx_ready(dspic33ak_uart_instance_t inst)
{
    const dspic33ak_uart_regs_t *r;

    if (uart_require_initialized(inst, &r) != DSPIC33AK_UART_OK) {
        return false;
    }

    /* TX can accept a byte when the TX buffer is NOT full. */
    return !dspic33ak_uart_reg_is_set(r->STAT, DSPIC33AK_UART_STAT_TXBF);
}

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_tx_done                                                     */
/* -------------------------------------------------------------------------- */
bool dspic33ak_uart_tx_done(dspic33ak_uart_instance_t inst)
{
    const dspic33ak_uart_regs_t *r;

    if (uart_require_initialized(inst, &r) != DSPIC33AK_UART_OK) {
        return false;
    }

    return dspic33ak_uart_reg_is_set(r->STAT, DSPIC33AK_UART_STAT_TXMTIF) &&
           dspic33ak_uart_reg_is_set(r->STAT, DSPIC33AK_UART_STAT_TXBE);
}

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_write_byte                                                  */
/* -------------------------------------------------------------------------- */
dspic33ak_uart_status_t dspic33ak_uart_write_byte(
    dspic33ak_uart_instance_t inst,
    uint8_t data)
{
    const dspic33ak_uart_regs_t *r;
    dspic33ak_uart_status_t st;
    uint32_t start_ms;

    st = uart_require_initialized(inst, &r);
    if (st != DSPIC33AK_UART_OK) {
        return st;
    }

    start_ms = uart_timeout_start_ms(inst);
    while (dspic33ak_uart_reg_is_set(r->STAT, DSPIC33AK_UART_STAT_TXBF)) {
        if (uart_timeout_enabled(inst) && uart_timeout_expired(inst, start_ms)) {
            return DSPIC33AK_UART_ERR_TIMEOUT;
        }
    }

    *r->TXB = (uint32_t)data;
    return DSPIC33AK_UART_OK;
}

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_read_byte                                                   */
/* -------------------------------------------------------------------------- */
dspic33ak_uart_status_t dspic33ak_uart_read_byte(
    dspic33ak_uart_instance_t inst,
    uint8_t *data)
{
    const dspic33ak_uart_regs_t *r;
    dspic33ak_uart_status_t st;

    if (data == 0) {
        return DSPIC33AK_UART_ERR_INVALID_ARG;
    }

    st = uart_require_initialized(inst, &r);
    if (st != DSPIC33AK_UART_OK) {
        return st;
    }

    /* ISR ring backend: pop from the software ring (the ISR owns the FIFO). */
    if (g_rx_mode[inst] == DSPIC33AK_UART_RX_MODE_ISR_RING) {
        return dspic33ak_uart_rx_isr_read_byte(inst, data);
    }

    /* Polling backend: read directly from the RX FIFO. */
    if (dspic33ak_uart_reg_is_set(r->STAT, DSPIC33AK_UART_STAT_RXBE)) {
        return DSPIC33AK_UART_ERR_RX_EMPTY;
    }

    *data = (uint8_t)(*r->RXB);
    return DSPIC33AK_UART_OK;
}

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_write                                                       */
/* -------------------------------------------------------------------------- */
size_t dspic33ak_uart_write(
    dspic33ak_uart_instance_t inst,
    const void *data,
    size_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    size_t i;

    if (len != 0u && data == 0) {
        return 0u;
    }

    if (uart_check_initialized(inst) != DSPIC33AK_UART_OK) {
        return 0u;
    }

    for (i = 0u; i < len; i++) {
        if (dspic33ak_uart_write_byte(inst, p[i]) != DSPIC33AK_UART_OK) {
            break;
        }
    }

    return i;
}

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_read                                                        */
/* -------------------------------------------------------------------------- */
size_t dspic33ak_uart_read(
    dspic33ak_uart_instance_t inst,
    void *data,
    size_t len)
{
    uint8_t *p = (uint8_t *)data;
    size_t i;

    if (len != 0u && data == 0) {
        return 0u;
    }

    if (uart_check_initialized(inst) != DSPIC33AK_UART_OK) {
        return 0u;
    }

    for (i = 0u; i < len; i++) {
        if (dspic33ak_uart_read_byte(inst, &p[i]) != DSPIC33AK_UART_OK) {
            break;
        }
    }

    return i;
}

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_rx_flush                                                    */
/* -------------------------------------------------------------------------- */
void dspic33ak_uart_rx_flush(dspic33ak_uart_instance_t inst)
{
    const dspic33ak_uart_regs_t *r;

    if (uart_require_initialized(inst, &r) != DSPIC33AK_UART_OK) {
        return;
    }

    /* ISR ring backend: flush the ring (it also drains the hardware FIFO). */
    if (g_rx_mode[inst] == DSPIC33AK_UART_RX_MODE_ISR_RING) {
        dspic33ak_uart_rx_isr_flush(inst);
        return;
    }

    /* Polling backend: drain the hardware RX FIFO and clear the overflow flag. */
    while (!dspic33ak_uart_reg_is_set(r->STAT, DSPIC33AK_UART_STAT_RXBE)) {
        (void)(*r->RXB);
    }

    if (dspic33ak_uart_reg_is_set(r->STAT, DSPIC33AK_UART_STAT_RXFOIF)) {
        dspic33ak_uart_reg_clear(r->STAT, DSPIC33AK_UART_STAT_RXFOIF);
    }
}

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_rx_status_get                                               */
/* -------------------------------------------------------------------------- */
dspic33ak_uart_status_t dspic33ak_uart_rx_status_get(
    dspic33ak_uart_instance_t inst,
    dspic33ak_uart_rx_status_t *status)
{
    dspic33ak_uart_status_t st;

    if (status == 0) {
        return DSPIC33AK_UART_ERR_INVALID_ARG;
    }

    st = uart_check_initialized(inst);
    if (st != DSPIC33AK_UART_OK) {
        return st;
    }

    memset(status, 0, sizeof(*status));
    status->rx_mode = g_rx_mode[inst];

    /* ISR ring backend: copy the ring counters. Polling backend keeps no
     * counters, so the zeroed snapshot above is the result. */
    if (g_rx_mode[inst] == DSPIC33AK_UART_RX_MODE_ISR_RING) {
        dspic33ak_uart_rx_isr_status_t isr_status;

        dspic33ak_uart_rx_isr_status_get(inst, &isr_status);

        status->rx_isr_count            = isr_status.rx_isr_count;
        status->rx_byte_count           = isr_status.rx_byte_count;
        status->rx_fifo_overflow_count  = isr_status.rx_fifo_overflow_count;
        status->framing_error_count     = isr_status.framing_error_count;
        status->parity_error_count      = isr_status.parity_error_count;
        status->autobaud_overflow_count = isr_status.autobaud_overflow_count;
        status->tx_collision_count      = isr_status.tx_collision_count;
        status->rx_ring_overflow_count  = isr_status.rx_ring_overflow_count;
        status->rx_max_drain_count      = isr_status.rx_max_drain_count;
    }

    return DSPIC33AK_UART_OK;
}

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_rx_status_clear                                             */
/* -------------------------------------------------------------------------- */
dspic33ak_uart_status_t dspic33ak_uart_rx_status_clear(
    dspic33ak_uart_instance_t inst)
{
    dspic33ak_uart_status_t st;

    st = uart_check_initialized(inst);
    if (st != DSPIC33AK_UART_OK) {
        return st;
    }

    /* ISR ring backend: clear the ring counters. Polling backend has none. */
    if (g_rx_mode[inst] == DSPIC33AK_UART_RX_MODE_ISR_RING) {
        dspic33ak_uart_rx_isr_status_clear(inst);
    }

    return DSPIC33AK_UART_OK;
}

/* ========================================================================== */
/* Public Functions: Asynchronous Transfer Model                              */
/* ========================================================================== */

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_set_callback                                                */
/* -------------------------------------------------------------------------- */
dspic33ak_uart_status_t dspic33ak_uart_set_callback(
    dspic33ak_uart_instance_t inst,
    dspic33ak_uart_event_callback_t callback,
    void *user_data)
{
    if (!uart_inst_is_valid(inst)) {
        return DSPIC33AK_UART_ERR_INVALID_ARG;
    }
    if (!dspic33ak_uart_instance_is_present(inst)) {
        return DSPIC33AK_UART_ERR_NOT_PRESENT;
    }

    g_callback[inst]           = callback;
    g_callback_user_data[inst] = user_data;
    return DSPIC33AK_UART_OK;
}

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_tx_enable                                                   */
/* -------------------------------------------------------------------------- */
dspic33ak_uart_status_t dspic33ak_uart_tx_enable(
    dspic33ak_uart_instance_t inst,
    bool enable)
{
    const dspic33ak_uart_regs_t *r;
    dspic33ak_uart_status_t st;

    st = uart_require_initialized(inst, &r);
    if (st != DSPIC33AK_UART_OK) {
        return st;
    }

    /* Disabling TX under an active async transfer would strand it (no
     * SEND_COMPLETE); reject until the transfer finishes or is aborted. */
    if (!enable && g_tx_busy[inst]) {
        return DSPIC33AK_UART_ERR_BUSY;
    }

    if (enable) {
        dspic33ak_uart_reg_set(r->CON, DSPIC33AK_UART_CON_TXEN);
    } else {
        dspic33ak_uart_reg_clear(r->CON, DSPIC33AK_UART_CON_TXEN);
    }
    g_tx_enabled[inst] = enable;
    return DSPIC33AK_UART_OK;
}

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_rx_enable                                                   */
/* -------------------------------------------------------------------------- */
dspic33ak_uart_status_t dspic33ak_uart_rx_enable(
    dspic33ak_uart_instance_t inst,
    bool enable)
{
    const dspic33ak_uart_regs_t *r;
    dspic33ak_uart_status_t st;

    st = uart_require_initialized(inst, &r);
    if (st != DSPIC33AK_UART_OK) {
        return st;
    }

    /* Disabling RX under an active async transfer would strand it (no
     * RX_COMPLETE); reject until the transfer finishes or is aborted. */
    if (!enable && g_rx_busy[inst]) {
        return DSPIC33AK_UART_ERR_BUSY;
    }

    if (enable) {
        dspic33ak_uart_reg_set(r->CON, DSPIC33AK_UART_CON_RXEN);
    } else {
        dspic33ak_uart_reg_clear(r->CON, DSPIC33AK_UART_CON_RXEN);
    }
    g_rx_enabled[inst] = enable;
    return DSPIC33AK_UART_OK;
}

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_tx_is_enabled                                               */
/* -------------------------------------------------------------------------- */
bool dspic33ak_uart_tx_is_enabled(dspic33ak_uart_instance_t inst)
{
    if (uart_check_initialized(inst) != DSPIC33AK_UART_OK) {
        return false;
    }
    return g_tx_enabled[inst];
}

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_rx_is_enabled                                               */
/* -------------------------------------------------------------------------- */
bool dspic33ak_uart_rx_is_enabled(dspic33ak_uart_instance_t inst)
{
    if (uart_check_initialized(inst) != DSPIC33AK_UART_OK) {
        return false;
    }
    return g_rx_enabled[inst];
}

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_set_baudrate                                                */
/* -------------------------------------------------------------------------- */
dspic33ak_uart_status_t dspic33ak_uart_set_baudrate(
    dspic33ak_uart_instance_t inst,
    uint32_t uart_clk_hz,
    uint32_t baudrate)
{
    const dspic33ak_uart_regs_t *r;
    dspic33ak_uart_status_t st;
    uint32_t brg;

    st = uart_require_initialized(inst, &r);
    if (st != DSPIC33AK_UART_OK) {
        return st;
    }

    if (uart_clk_hz == 0u || baudrate == 0u) {
        return DSPIC33AK_UART_ERR_INVALID_ARG;
    }

    /* Never reconfigure the divisor under an active transfer or while a byte is
     * still being shifted out. */
    if (g_tx_busy[inst] || g_rx_busy[inst]) {
        return DSPIC33AK_UART_ERR_BUSY;
    }
    if (!dspic33ak_uart_reg_is_set(r->STAT, DSPIC33AK_UART_STAT_TXMTIF) ||
        !dspic33ak_uart_reg_is_set(r->STAT, DSPIC33AK_UART_STAT_TXBE)) {
        return DSPIC33AK_UART_ERR_BUSY;
    }

    brg = uart_calc_brg(uart_clk_hz, baudrate);
    if (brg == 0u) {
        return DSPIC33AK_UART_ERR_INVALID_ARG;
    }

    *r->BRG = brg;
    g_uart_clk_hz[inst] = uart_clk_hz;
    g_baudrate[inst]    = baudrate;
    return DSPIC33AK_UART_OK;
}

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_get_baudrate                                                */
/* -------------------------------------------------------------------------- */
uint32_t dspic33ak_uart_get_baudrate(dspic33ak_uart_instance_t inst)
{
    if (uart_check_initialized(inst) != DSPIC33AK_UART_OK) {
        return 0u;
    }
    return g_baudrate[inst];
}

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_tx_start                                                    */
/* -------------------------------------------------------------------------- */
dspic33ak_uart_status_t dspic33ak_uart_tx_start(
    dspic33ak_uart_instance_t inst,
    const uint8_t *data,
    size_t length)
{
    dspic33ak_uart_status_t st;

    st = uart_check_initialized(inst);
    if (st != DSPIC33AK_UART_OK) {
        return st;
    }
    if (data == 0 || length == 0u) {
        return DSPIC33AK_UART_ERR_INVALID_ARG;
    }
    if (g_tx_busy[inst]) {
        return DSPIC33AK_UART_ERR_BUSY;
    }
    /* Async TX needs TX enabled and a usable (non-zero) TX interrupt priority;
     * otherwise the transfer would never complete (no SEND_COMPLETE). Reject up
     * front instead of returning a "started" transfer that silently stalls. */
    if (!g_tx_enabled[inst]) {
        return DSPIC33AK_UART_ERR_UNSUPPORTED;
    }
    if (g_tx_irq_priority[inst] == 0u) {
        return DSPIC33AK_UART_ERR_UNSUPPORTED;
    }

    /* Publish the transfer descriptor before arming the interrupt. */
    g_tx_buf[inst]   = data;
    g_tx_len[inst]   = length;
    g_tx_count[inst] = 0u;
    g_tx_busy[inst]  = true;

    /* Arm the TX interrupt and software-trigger the first entry so the engine
     * starts even if no "FIFO has space" interrupt is latched yet. */
    (void)uart_tx_irq_clear_flag(inst);
    if (!uart_tx_irq_enable(inst, true)) {
        g_tx_busy[inst] = false;
        return DSPIC33AK_UART_ERR_UNSUPPORTED;
    }
    (void)uart_tx_irq_raise_flag(inst);

    return DSPIC33AK_UART_OK;
}

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_tx_abort                                                    */
/* -------------------------------------------------------------------------- */
dspic33ak_uart_status_t dspic33ak_uart_tx_abort(dspic33ak_uart_instance_t inst)
{
    dspic33ak_uart_status_t st;

    st = uart_check_initialized(inst);
    if (st != DSPIC33AK_UART_OK) {
        return st;
    }

    (void)uart_tx_irq_enable(inst, false);
    (void)uart_tx_irq_clear_flag(inst);
    g_tx_busy[inst] = false;
    return DSPIC33AK_UART_OK;
}

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_tx_count_get                                                */
/* -------------------------------------------------------------------------- */
size_t dspic33ak_uart_tx_count_get(dspic33ak_uart_instance_t inst)
{
    if (!uart_inst_is_valid(inst)) {
        return 0u;
    }
    return g_tx_count[inst];
}

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_tx_is_busy                                                  */
/* -------------------------------------------------------------------------- */
bool dspic33ak_uart_tx_is_busy(dspic33ak_uart_instance_t inst)
{
    if (!uart_inst_is_valid(inst)) {
        return false;
    }
    return g_tx_busy[inst];
}

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_rx_start                                                    */
/* -------------------------------------------------------------------------- */
dspic33ak_uart_status_t dspic33ak_uart_rx_start(
    dspic33ak_uart_instance_t inst,
    uint8_t *data,
    size_t length)
{
    dspic33ak_uart_status_t st;

    st = uart_check_initialized(inst);
    if (st != DSPIC33AK_UART_OK) {
        return st;
    }
    if (data == 0 || length == 0u) {
        return DSPIC33AK_UART_ERR_INVALID_ARG;
    }
    /* Async RX is fed from the RX ISR, which only runs in ISR ring mode. */
    if (g_rx_mode[inst] != DSPIC33AK_UART_RX_MODE_ISR_RING) {
        return DSPIC33AK_UART_ERR_UNSUPPORTED;
    }
    /* RX must be enabled, or no bytes will arrive and the transfer never
     * completes (no RX_COMPLETE). */
    if (!g_rx_enabled[inst]) {
        return DSPIC33AK_UART_ERR_UNSUPPORTED;
    }
    if (g_rx_busy[inst]) {
        return DSPIC33AK_UART_ERR_BUSY;
    }

    /* Publish the descriptor, then arm. The RX interrupt is already enabled by
     * the ISR ring backend; dspic33ak_uart_async_rx_feed() picks the bytes up as
     * soon as g_rx_busy becomes true. */
    uart_rx_arm(inst, data, length);

    return DSPIC33AK_UART_OK;
}

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_rx_start_clean                                              */
/* -------------------------------------------------------------------------- */
dspic33ak_uart_status_t dspic33ak_uart_rx_start_clean(
    dspic33ak_uart_instance_t inst,
    uint8_t *data,
    size_t length)
{
    dspic33ak_uart_status_t st;
    uint8_t rx_irq_enabled;

    st = uart_check_initialized(inst);
    if (st != DSPIC33AK_UART_OK) {
        return st;
    }
    if (data == 0 || length == 0u) {
        return DSPIC33AK_UART_ERR_INVALID_ARG;
    }
    if (g_rx_mode[inst] != DSPIC33AK_UART_RX_MODE_ISR_RING) {
        return DSPIC33AK_UART_ERR_UNSUPPORTED;
    }
    if (!g_rx_enabled[inst]) {
        return DSPIC33AK_UART_ERR_UNSUPPORTED;
    }
    if (g_rx_busy[inst]) {
        return DSPIC33AK_UART_ERR_BUSY;
    }

    rx_irq_enabled = uart_rx_irq_get_enable(inst);
    if (rx_irq_enabled == 0u) {
        return DSPIC33AK_UART_ERR_UNSUPPORTED;
    }
    if (!uart_rx_irq_enable(inst, false)) {
        return DSPIC33AK_UART_ERR_UNSUPPORTED;
    }

    /* Drop bytes that predate this call, then arm while the RX ISR cannot move
     * a just-arrived byte into the regular ring. */
    dspic33ak_uart_rx_isr_flush(inst);
    uart_rx_arm(inst, data, length);

    (void)uart_rx_irq_enable(inst, (rx_irq_enabled != 0u));
    return DSPIC33AK_UART_OK;
}

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_rx_abort                                                    */
/* -------------------------------------------------------------------------- */
dspic33ak_uart_status_t dspic33ak_uart_rx_abort(dspic33ak_uart_instance_t inst)
{
    dspic33ak_uart_status_t st;

    st = uart_check_initialized(inst);
    if (st != DSPIC33AK_UART_OK) {
        return st;
    }

    /* Single volatile flag write; the RX ISR re-checks it before each store. */
    g_rx_busy[inst] = false;
    return DSPIC33AK_UART_OK;
}

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_rx_count_get                                                */
/* -------------------------------------------------------------------------- */
size_t dspic33ak_uart_rx_count_get(dspic33ak_uart_instance_t inst)
{
    if (!uart_inst_is_valid(inst)) {
        return 0u;
    }
    return g_rx_count[inst];
}

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_rx_is_busy                                                  */
/* -------------------------------------------------------------------------- */
bool dspic33ak_uart_rx_is_busy(dspic33ak_uart_instance_t inst)
{
    if (!uart_inst_is_valid(inst)) {
        return false;
    }
    return g_rx_busy[inst];
}

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_tx_irq_handler                                              */
/* -------------------------------------------------------------------------- */
void dspic33ak_uart_tx_irq_handler(dspic33ak_uart_instance_t inst)
{
    const dspic33ak_uart_regs_t *r;

    if (uart_get_regs(inst, &r) != DSPIC33AK_UART_OK) {
        return;
    }

    (void)uart_tx_irq_clear_flag(inst);

    /* Spurious / aborted: nothing to do but make sure the interrupt is off. */
    if (!g_tx_busy[inst]) {
        (void)uart_tx_irq_enable(inst, false);
        return;
    }

    /* Push bytes while the TX FIFO can accept them and data remains. */
    while ((g_tx_count[inst] < g_tx_len[inst]) &&
           !dspic33ak_uart_reg_is_set(r->STAT, DSPIC33AK_UART_STAT_TXBF)) {
        *r->TXB = (uint32_t)g_tx_buf[inst][g_tx_count[inst]];
        g_tx_count[inst]++;
    }

    /* All bytes submitted to the FIFO: stop the interrupt and report complete.
     * This is the CMSIS SEND_COMPLETE sense ("driver accepted and submitted all
     * data"); physical shift-register-empty is still observable via
     * dspic33ak_uart_tx_done(). */
    if (g_tx_count[inst] >= g_tx_len[inst]) {
        (void)uart_tx_irq_enable(inst, false);
        g_tx_busy[inst] = false;
        uart_notify(inst, DSPIC33AK_UART_EVENT_SEND_COMPLETE);
    }
}

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_async_rx_feed (internal ISR hook)                           */
/* -------------------------------------------------------------------------- */
bool dspic33ak_uart_async_rx_feed(dspic33ak_uart_instance_t inst, uint8_t byte)
{
    if (!uart_inst_is_valid(inst) || !g_rx_busy[inst]) {
        return false;
    }

    g_rx_buf[inst][g_rx_count[inst]] = byte;
    g_rx_count[inst]++;

    if (g_rx_count[inst] >= g_rx_len[inst]) {
        g_rx_busy[inst] = false;
        uart_notify(inst, DSPIC33AK_UART_EVENT_RX_COMPLETE);
    }

    return true;
}

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_async_notify_events (internal ISR hook)                     */
/* -------------------------------------------------------------------------- */
void dspic33ak_uart_async_notify_events(
    dspic33ak_uart_instance_t inst,
    uint32_t events)
{
    if (!uart_inst_is_valid(inst) || events == 0u) {
        return;
    }
    uart_notify(inst, events);
}

/* ========================================================================== */
/* Local Functions                                                            */
/* ========================================================================== */

/* -------------------------------------------------------------------------- */
/* uart_inst_is_valid                                                         */
/* -------------------------------------------------------------------------- */
static bool uart_inst_is_valid(dspic33ak_uart_instance_t inst)
{
    return ((unsigned)inst < (unsigned)DSPIC33AK_UART_INST_COUNT);
}

/* -------------------------------------------------------------------------- */
/* uart_get_regs                                                              */
/* -------------------------------------------------------------------------- */
static dspic33ak_uart_status_t uart_get_regs(
    dspic33ak_uart_instance_t inst,
    const dspic33ak_uart_regs_t **regs)
{
    const dspic33ak_uart_device_t *dev;

    if (regs == 0) {
        return DSPIC33AK_UART_ERR_INVALID_ARG;
    }

    if (!uart_inst_is_valid(inst)) {
        return DSPIC33AK_UART_ERR_INVALID_ARG;
    }

    dev = dspic33ak_uart_get_device(inst);
    if (dev == 0) {
        return DSPIC33AK_UART_ERR_NOT_PRESENT;
    }

    *regs = &dev->regs;
    return DSPIC33AK_UART_OK;
}

/* -------------------------------------------------------------------------- */
/* uart_require_initialized                                                   */
/* -------------------------------------------------------------------------- */
static dspic33ak_uart_status_t uart_require_initialized(
    dspic33ak_uart_instance_t inst,
    const dspic33ak_uart_regs_t **regs)
{
    dspic33ak_uart_status_t st;

    if (!uart_inst_is_valid(inst)) {
        return DSPIC33AK_UART_ERR_INVALID_ARG;
    }

    st = uart_get_regs(inst, regs);
    if (st != DSPIC33AK_UART_OK) {
        return st;
    }

    if (!g_initialized[inst]) {
        return DSPIC33AK_UART_ERR_NOT_INITIALIZED;
    }

    return DSPIC33AK_UART_OK;
}

/* -------------------------------------------------------------------------- */
/* uart_check_initialized                                                     */
/* -------------------------------------------------------------------------- */
static dspic33ak_uart_status_t uart_check_initialized(
    dspic33ak_uart_instance_t inst)
{
    const dspic33ak_uart_regs_t *r;
    return uart_require_initialized(inst, &r);
}

/* -------------------------------------------------------------------------- */
/* uart_calc_brg                                                              */
/* -------------------------------------------------------------------------- */
static uint32_t uart_calc_brg(
    uint32_t uart_clk_hz,
    uint32_t baudrate)
{
    uint64_t div;

    if (baudrate == 0u) {
        return 0u;
    }

    /*
     * Fractional mode: BRG = round(uart_clk_hz / baudrate) - 1.
     * 64-bit math avoids overflow on faster clocks.
     */
    div = ((uint64_t)uart_clk_hz + ((uint64_t)baudrate / 2u)) /
          (uint64_t)baudrate;
    if (div == 0u) {
        return 0u;
    }

    return (uint32_t)(div - 1u);
}

/* -------------------------------------------------------------------------- */
/* uart_timeout_enabled                                                       */
/* -------------------------------------------------------------------------- */
static bool uart_timeout_enabled(dspic33ak_uart_instance_t inst)
{
    return uart_inst_is_valid(inst) &&
           (g_get_ms[inst] != 0) &&
           (g_timeout_ms[inst] != 0u);
}

/* -------------------------------------------------------------------------- */
/* uart_timeout_start_ms                                                      */
/* -------------------------------------------------------------------------- */
static uint32_t uart_timeout_start_ms(dspic33ak_uart_instance_t inst)
{
    if (!uart_timeout_enabled(inst)) {
        return 0u;
    }

    return g_get_ms[inst]();
}

/* -------------------------------------------------------------------------- */
/* uart_timeout_expired                                                       */
/* -------------------------------------------------------------------------- */
static bool uart_timeout_expired(
    dspic33ak_uart_instance_t inst,
    uint32_t start_ms)
{
    uint32_t now;

    if (!uart_timeout_enabled(inst)) {
        return false;
    }

    now = g_get_ms[inst]();
    return ((uint32_t)(now - start_ms) >= g_timeout_ms[inst]);
}

/* -------------------------------------------------------------------------- */
/* uart_async_reset                                                           */
/* -------------------------------------------------------------------------- */
static void uart_async_reset(dspic33ak_uart_instance_t inst)
{
    g_tx_buf[inst]   = 0;
    g_tx_len[inst]   = 0u;
    g_tx_count[inst] = 0u;
    g_tx_busy[inst]  = false;

    g_rx_buf[inst]   = 0;
    g_rx_len[inst]   = 0u;
    g_rx_count[inst] = 0u;
    g_rx_busy[inst]  = false;
}

/* -------------------------------------------------------------------------- */
/* uart_rx_arm                                                                */
/* -------------------------------------------------------------------------- */
static void uart_rx_arm(
    dspic33ak_uart_instance_t inst,
    uint8_t *data,
    size_t length)
{
    g_rx_buf[inst]   = data;
    g_rx_len[inst]   = length;
    g_rx_count[inst] = 0u;
    g_rx_busy[inst]  = true;
}

/* -------------------------------------------------------------------------- */
/* uart_notify                                                                */
/* -------------------------------------------------------------------------- */
static void uart_notify(dspic33ak_uart_instance_t inst, uint32_t events)
{
    dspic33ak_uart_event_callback_t cb = g_callback[inst];

    if (cb != 0) {
        cb(inst, events, g_callback_user_data[inst]);
    }
}

/* ========================================================================== */
/* Local Functions: TX interrupt operations                                   */
/* ========================================================================== */

/* -------------------------------------------------------------------------- */
/* uart_tx_irq_set_priority                                                   */
/* -------------------------------------------------------------------------- */
static bool uart_tx_irq_set_priority(dspic33ak_uart_instance_t inst, uint8_t prio)
{
    return dspic33ak_uart_device_set_tx_irq_priority(inst, prio);
}

/* -------------------------------------------------------------------------- */
/* uart_tx_irq_clear_flag                                                      */
/* -------------------------------------------------------------------------- */
static bool uart_tx_irq_clear_flag(dspic33ak_uart_instance_t inst)
{
    const dspic33ak_uart_device_t *dev = dspic33ak_uart_get_device(inst);

    return (dev != 0) && dspic33ak_uart_reg_irq_clear(&dev->regs.irq_tx);
}

/* -------------------------------------------------------------------------- */
/* uart_tx_irq_raise_flag                                                      */
/*                                                                            */
/* Software-set the TX interrupt flag to force the first ISR entry after the   */
/* engine is armed (the FIFO-space condition may not latch a flag on enable).  */
/* -------------------------------------------------------------------------- */
static bool uart_tx_irq_raise_flag(dspic33ak_uart_instance_t inst)
{
    const dspic33ak_uart_device_t *dev = dspic33ak_uart_get_device(inst);

    return (dev != 0) && dspic33ak_uart_reg_irq_raise(&dev->regs.irq_tx);
}

/* -------------------------------------------------------------------------- */
/* uart_tx_irq_enable                                                         */
/* -------------------------------------------------------------------------- */
static bool uart_tx_irq_enable(dspic33ak_uart_instance_t inst, bool enable)
{
    const dspic33ak_uart_device_t *dev = dspic33ak_uart_get_device(inst);

    if (dev == 0) {
        return false;
    }

    return enable ?
        dspic33ak_uart_reg_irq_enable(&dev->regs.irq_tx) :
        dspic33ak_uart_reg_irq_disable(&dev->regs.irq_tx);
}

/* ========================================================================== */
/* Local Functions: RX interrupt operations                                   */
/* ========================================================================== */

/* -------------------------------------------------------------------------- */
/* uart_rx_irq_enable                                                         */
/* -------------------------------------------------------------------------- */
static bool uart_rx_irq_enable(dspic33ak_uart_instance_t inst, bool enable)
{
    const dspic33ak_uart_device_t *dev = dspic33ak_uart_get_device(inst);

    if (dev == 0) {
        return false;
    }

    return enable ?
        dspic33ak_uart_reg_irq_enable(&dev->regs.irq_rx) :
        dspic33ak_uart_reg_irq_disable(&dev->regs.irq_rx);
}

/* -------------------------------------------------------------------------- */
/* uart_rx_irq_get_enable                                                     */
/* -------------------------------------------------------------------------- */
static uint8_t uart_rx_irq_get_enable(dspic33ak_uart_instance_t inst)
{
    const dspic33ak_uart_device_t *dev = dspic33ak_uart_get_device(inst);

    return (dev != 0) ?
        dspic33ak_uart_reg_irq_is_enabled(&dev->regs.irq_rx) :
        0u;
}

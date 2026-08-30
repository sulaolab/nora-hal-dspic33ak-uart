/* ========================================================================== */
/* Includes                                                                   */
/* ========================================================================== */

#include <string.h>
#include <stddef.h>

#include "nora_uart_dspic33ak_rx_isr_ring.h"
#include "nora_uart_dspic33ak_device.h"
#include "nora_uart_dspic33ak_reg.h"

/* ========================================================================== */
/* Module Overview                                                            */
/* ========================================================================== */

/*
 * UART RX interrupt-driven ring HAL - implementation.
 *
 * The RX FIFO -> software-ring drain logic, the RX error-flag accounting, and
 * the RX ISR ring runtime status counters live here. The ring buffer storage is
 * caller-provided; this module only keeps a pointer to it plus the read/write
 * indices.
 *
 * Register and IRQ access goes through the device register-pointer table
 * (nora_uart_dspic33ak_get_device) and the nora_uart_dspic33ak_reg.h bit-mask helpers, so
 * no UxSTATbits / UxRXB / _UxRXIF / _UxRXIE / _UxRXIP symbol names appear here.
 *
 * No printf / halt / blocking calls and no application dependencies. The
 * _UxRXInterrupt vector is NOT defined here; the application owns the vector and
 * calls nora_uart_rx_irq_handler() from it.
 */

/* ========================================================================== */
/* Module Variables                                                           */
/* ========================================================================== */

/*
 * Single-producer (ISR) / single-consumer (reader) ring, per instance.
 *   - g_rx_write_idx is advanced only by nora_uart_rx_irq_handler().
 *   - g_rx_read_idx is advanced only by nora_uart_dspic33ak_rx_isr_read_byte().
 * The buffer itself is owned by the caller (passed in via config).
 */
static uint8_t *g_rx_ring[NORA_UART_INST_SUPPORTED_COUNT];
static uint16_t g_rx_ring_size[NORA_UART_INST_SUPPORTED_COUNT];
static volatile uint16_t g_rx_read_idx[NORA_UART_INST_SUPPORTED_COUNT];
static volatile uint16_t g_rx_write_idx[NORA_UART_INST_SUPPORTED_COUNT];
static volatile nora_uart_dspic33ak_rx_isr_status_t g_rx_status[NORA_UART_INST_SUPPORTED_COUNT];
static bool g_rx_isr_configured[NORA_UART_INST_SUPPORTED_COUNT];

/* ========================================================================== */
/* Local Function Prototypes                                                  */
/* ========================================================================== */

static bool uart_inst_is_valid(nora_uart_instance_t inst);
static const nora_uart_dspic33ak_regs_t *uart_regs(nora_uart_instance_t inst);

static bool uart_rx_irq_set_priority(nora_uart_instance_t inst, uint8_t prio);
static bool uart_rx_irq_clear_flag(nora_uart_instance_t inst);
static bool uart_rx_irq_enable(nora_uart_instance_t inst, bool enable);
static uint8_t uart_rx_irq_get_enable(nora_uart_instance_t inst);

static void uart_rx_ring_push(nora_uart_instance_t inst, uint8_t b);

/* ========================================================================== */
/* Public Functions                                                           */
/* ========================================================================== */

/* -------------------------------------------------------------------------- */
/* nora_uart_dspic33ak_rx_isr_config                                               */
/* -------------------------------------------------------------------------- */
nora_uart_status_t nora_uart_dspic33ak_rx_isr_config(
    nora_uart_instance_t inst,
    const nora_uart_dspic33ak_rx_isr_config_t *config)
{
    const nora_uart_dspic33ak_regs_t *r;

    if (!uart_inst_is_valid(inst)) {
        return NORA_UART_ERR_INVALID_ARG;
    }
    if (config == NULL || config->buffer == NULL || config->buffer_size < 2u) {
        return NORA_UART_ERR_INVALID_ARG;
    }

    r = uart_regs(inst);
    if (r == NULL) {
        return NORA_UART_ERR_NOT_PRESENT;
    }
    if (!nora_uart_is_initialized(inst)) {
        return NORA_UART_ERR_NOT_INITIALIZED;
    }

    /* Reject instances with no RX interrupt mapping before changing any state. */
    if (!uart_rx_irq_set_priority(inst, config->irq_priority)) {
        return NORA_UART_ERR_UNSUPPORTED;
    }

    /* Bind the caller's buffer and reset ring + status counters. */
    g_rx_ring[inst]      = config->buffer;
    g_rx_ring_size[inst] = config->buffer_size;
    g_rx_read_idx[inst]  = 0u;
    g_rx_write_idx[inst] = 0u;
    memset(config->buffer, 0x00, config->buffer_size);
    memset((void *)&g_rx_status[inst], 0x00, sizeof(g_rx_status[inst]));

    /* Interrupt when >= 1 char is in the RX FIFO (RXWM = 0). */
    nora_uart_dspic33ak_reg_clear(r->STAT, NORA_UART_DSPIC33AK_STAT_RXWM);

    /* Priority already set above; clear any stale flag, leave disabled. */
    (void)uart_rx_irq_clear_flag(inst);

    g_rx_isr_configured[inst] = true;

    return NORA_UART_OK;
}

/* -------------------------------------------------------------------------- */
/* nora_uart_dspic33ak_rx_isr_enable                                               */
/* -------------------------------------------------------------------------- */
nora_uart_status_t nora_uart_dspic33ak_rx_isr_enable(
    nora_uart_instance_t inst)
{
    if (!uart_inst_is_valid(inst)) {
        return NORA_UART_ERR_INVALID_ARG;
    }
    if (!g_rx_isr_configured[inst]) {
        /* No "not configured" status code exists; the ring is unusable until
         * nora_uart_dspic33ak_rx_isr_config() succeeds, so report not-initialized. */
        return NORA_UART_ERR_NOT_INITIALIZED;
    }
    if (!nora_uart_is_initialized(inst)) {
        return NORA_UART_ERR_NOT_INITIALIZED;
    }

    (void)uart_rx_irq_clear_flag(inst);
    if (!uart_rx_irq_enable(inst, true)) {
        return NORA_UART_ERR_UNSUPPORTED;
    }

    return NORA_UART_OK;
}

/* -------------------------------------------------------------------------- */
/* nora_uart_dspic33ak_rx_isr_disable                                              */
/* -------------------------------------------------------------------------- */
nora_uart_status_t nora_uart_dspic33ak_rx_isr_disable(
    nora_uart_instance_t inst)
{
    if (!uart_inst_is_valid(inst)) {
        return NORA_UART_ERR_INVALID_ARG;
    }

    /* Disable is the safe direction: allowed even when not configured. */
    if (!uart_rx_irq_enable(inst, false)) {
        return NORA_UART_ERR_UNSUPPORTED;
    }
    (void)uart_rx_irq_clear_flag(inst);

    return NORA_UART_OK;
}

/* -------------------------------------------------------------------------- */
/* nora_uart_dspic33ak_rx_isr_ready                                                */
/* -------------------------------------------------------------------------- */
bool nora_uart_dspic33ak_rx_isr_ready(nora_uart_instance_t inst)
{
    if (!uart_inst_is_valid(inst) || !g_rx_isr_configured[inst]) {
        return false;
    }

    return (g_rx_read_idx[inst] != g_rx_write_idx[inst]);
}

/* -------------------------------------------------------------------------- */
/* nora_uart_dspic33ak_rx_isr_read_byte                                            */
/* -------------------------------------------------------------------------- */
nora_uart_status_t nora_uart_dspic33ak_rx_isr_read_byte(
    nora_uart_instance_t inst,
    uint8_t *data)
{
    uint16_t read_idx;
    uint16_t next;

    if (data == NULL) {
        return NORA_UART_ERR_INVALID_ARG;
    }
    if (!uart_inst_is_valid(inst) || !g_rx_isr_configured[inst]) {
        return NORA_UART_ERR_RX_EMPTY;
    }
    if (g_rx_read_idx[inst] == g_rx_write_idx[inst]) {
        return NORA_UART_ERR_RX_EMPTY;
    }

    read_idx = g_rx_read_idx[inst];
    *data = g_rx_ring[inst][read_idx];

    next = (uint16_t)(read_idx + 1u);
    if (next >= g_rx_ring_size[inst]) {
        next = 0u;
    }
    g_rx_read_idx[inst] = next;

    return NORA_UART_OK;
}

/* -------------------------------------------------------------------------- */
/* nora_uart_dspic33ak_rx_isr_flush                                                */
/* -------------------------------------------------------------------------- */
void nora_uart_dspic33ak_rx_isr_flush(nora_uart_instance_t inst)
{
    const nora_uart_dspic33ak_regs_t *r;
    uint8_t ie;

    if (!uart_inst_is_valid(inst)) {
        return;
    }
    r = uart_regs(inst);
    if (r == NULL) {
        return;
    }

    ie = uart_rx_irq_get_enable(inst);
    (void)uart_rx_irq_enable(inst, false);   /* brief critical section vs the ISR */

    g_rx_read_idx[inst] = g_rx_write_idx[inst];   /* drop buffered ring contents */

    while (!nora_uart_dspic33ak_reg_is_set(r->STAT, NORA_UART_DSPIC33AK_STAT_RXBE)) {
        (void)(*r->RXB);                      /* drain the hardware RX FIFO too */
    }
    if (nora_uart_dspic33ak_reg_is_set(r->STAT, NORA_UART_DSPIC33AK_STAT_RXFOIF)) {
        nora_uart_dspic33ak_reg_clear(r->STAT, NORA_UART_DSPIC33AK_STAT_RXFOIF);
    }

    (void)uart_rx_irq_enable(inst, (ie != 0u));
}

/* -------------------------------------------------------------------------- */
/* nora_uart_dspic33ak_rx_isr_status_get                                           */
/* -------------------------------------------------------------------------- */
void nora_uart_dspic33ak_rx_isr_status_get(
    nora_uart_instance_t inst,
    nora_uart_dspic33ak_rx_isr_status_t *status)
{
    uint8_t ie;

    if (status == NULL || !uart_inst_is_valid(inst)) {
        return;
    }

    ie = uart_rx_irq_get_enable(inst);
    (void)uart_rx_irq_enable(inst, false);   /* atomic snapshot vs the ISR */

    status->rx_isr_count            = g_rx_status[inst].rx_isr_count;
    status->rx_byte_count           = g_rx_status[inst].rx_byte_count;
    status->rx_fifo_overflow_count  = g_rx_status[inst].rx_fifo_overflow_count;
    status->framing_error_count     = g_rx_status[inst].framing_error_count;
    status->parity_error_count      = g_rx_status[inst].parity_error_count;
    status->autobaud_overflow_count = g_rx_status[inst].autobaud_overflow_count;
    status->tx_collision_count      = g_rx_status[inst].tx_collision_count;
    status->rx_ring_overflow_count  = g_rx_status[inst].rx_ring_overflow_count;
    status->rx_max_drain_count      = g_rx_status[inst].rx_max_drain_count;

    (void)uart_rx_irq_enable(inst, (ie != 0u));
}

/* -------------------------------------------------------------------------- */
/* nora_uart_dspic33ak_rx_isr_status_clear                                         */
/* -------------------------------------------------------------------------- */
void nora_uart_dspic33ak_rx_isr_status_clear(nora_uart_instance_t inst)
{
    uint8_t ie;

    if (!uart_inst_is_valid(inst)) {
        return;
    }

    ie = uart_rx_irq_get_enable(inst);
    (void)uart_rx_irq_enable(inst, false);
    memset((void *)&g_rx_status[inst], 0x00, sizeof(g_rx_status[inst]));
    (void)uart_rx_irq_enable(inst, (ie != 0u));
}

/* -------------------------------------------------------------------------- */
/* nora_uart_rx_irq_handler                                              */
/* -------------------------------------------------------------------------- */
void nora_uart_rx_irq_handler(nora_uart_instance_t inst)
{
    const nora_uart_dspic33ak_regs_t *r;
    uint16_t drain_count = 0u;
    uint32_t events = 0u;            /* async event bits to report after drain  */
    uint32_t ring_overflow_before;  /* ring-overflow counter snapshot          */
    bool ring_got_data = false;     /* at least one byte routed to the ring     */

    if (!uart_inst_is_valid(inst)) {
        return;
    }
    r = uart_regs(inst);
    if (r == NULL) {
        return;
    }

    (void)uart_rx_irq_clear_flag(inst);   /* clear RX interrupt flag first */
    g_rx_status[inst].rx_isr_count++;
    ring_overflow_before = g_rx_status[inst].rx_ring_overflow_count;

    /*
     * Drain all available bytes from the RX FIFO (reading RXB advances it). An
     * active async RX transfer takes priority: each byte is offered to it first
     * and only pushed to the software ring when no transfer is consuming bytes.
     * This keeps the existing byte-stream ring behavior intact when no async
     * Receive is in progress.
     */
    while (!nora_uart_dspic33ak_reg_is_set(r->STAT, NORA_UART_DSPIC33AK_STAT_RXBE)) {
        uint8_t b = (uint8_t)(*r->RXB);
        if (!nora_uart_dspic33ak_async_rx_feed(inst, b)) {
            uart_rx_ring_push(inst, b);
            ring_got_data = true;
        }
        drain_count++;
    }

    g_rx_status[inst].rx_byte_count += drain_count;
    if (drain_count > g_rx_status[inst].rx_max_drain_count) {
        g_rx_status[inst].rx_max_drain_count = drain_count;
    }

    /* Count and clear the latched RX error flags, collecting the generic async
     * event bits for the ones the async layer exposes. */
    if (nora_uart_dspic33ak_reg_is_set(r->STAT, NORA_UART_DSPIC33AK_STAT_RXFOIF)) {
        g_rx_status[inst].rx_fifo_overflow_count++;
        nora_uart_dspic33ak_reg_clear(r->STAT, NORA_UART_DSPIC33AK_STAT_RXFOIF);
        events |= NORA_UART_EVENT_RX_OVERRUN_ERROR;
    }
    if (nora_uart_dspic33ak_reg_is_set(r->STAT, NORA_UART_DSPIC33AK_STAT_FERIF)) {
        g_rx_status[inst].framing_error_count++;
        nora_uart_dspic33ak_reg_clear(r->STAT, NORA_UART_DSPIC33AK_STAT_FERIF);
        events |= NORA_UART_EVENT_RX_FRAMING_ERROR;
    }
    if (nora_uart_dspic33ak_reg_is_set(r->STAT, NORA_UART_DSPIC33AK_STAT_PERIF)) {
        g_rx_status[inst].parity_error_count++;
        nora_uart_dspic33ak_reg_clear(r->STAT, NORA_UART_DSPIC33AK_STAT_PERIF);
        events |= NORA_UART_EVENT_RX_PARITY_ERROR;
    }
    if (nora_uart_dspic33ak_reg_is_set(r->STAT, NORA_UART_DSPIC33AK_STAT_ABDOVIF)) {
        g_rx_status[inst].autobaud_overflow_count++;
        nora_uart_dspic33ak_reg_clear(r->STAT, NORA_UART_DSPIC33AK_STAT_ABDOVIF);
    }
    if (nora_uart_dspic33ak_reg_is_set(r->STAT, NORA_UART_DSPIC33AK_STAT_TXCIF)) {
        g_rx_status[inst].tx_collision_count++;
        nora_uart_dspic33ak_reg_clear(r->STAT, NORA_UART_DSPIC33AK_STAT_TXCIF);
    }

    /* Software ring overflow (a byte was dropped during the drain above). */
    if (g_rx_status[inst].rx_ring_overflow_count != ring_overflow_before) {
        events |= NORA_UART_EVENT_RX_OVERFLOW;
    }

    /* Unsolicited byte-stream data landed in the ring this pass. */
    if (ring_got_data) {
        events |= NORA_UART_EVENT_RX_READY;
    }

    if (events != 0u) {
        nora_uart_dspic33ak_async_notify_events(inst, events);
    }
}

/* ========================================================================== */
/* Local Functions                                                            */
/* ========================================================================== */

/* -------------------------------------------------------------------------- */
/* uart_inst_is_valid                                                         */
/* -------------------------------------------------------------------------- */
static bool uart_inst_is_valid(nora_uart_instance_t inst)
{
    /* Narrowed by NORA_UART_INST_SUPPORTED_COUNT -- see nora_uart.h. */
    return ((unsigned)inst < (unsigned)NORA_UART_INST_SUPPORTED_COUNT);
}

/* -------------------------------------------------------------------------- */
/* uart_regs                                                                  */
/* -------------------------------------------------------------------------- */
static const nora_uart_dspic33ak_regs_t *uart_regs(nora_uart_instance_t inst)
{
    const nora_uart_dspic33ak_device_t *dev = nora_uart_dspic33ak_get_device(inst);

    if (dev == NULL) {
        return NULL;
    }
    return &dev->regs;
}

/* -------------------------------------------------------------------------- */
/* uart_rx_ring_push                                                          */
/*                                                                            */
/* Single-producer push (ISR context). If the next write slot would collide   */
/* with the read index, the byte is dropped and rx_ring_overflow_count is      */
/* incremented (safer than advancing the write index unconditionally).        */
/* -------------------------------------------------------------------------- */
static void uart_rx_ring_push(nora_uart_instance_t inst, uint8_t b)
{
    uint16_t write_idx = g_rx_write_idx[inst];
    uint16_t next = (uint16_t)(write_idx + 1u);

    if (next >= g_rx_ring_size[inst]) {
        next = 0u;
    }

    if (next == g_rx_read_idx[inst]) {
        g_rx_status[inst].rx_ring_overflow_count++;
        return;
    }

    g_rx_ring[inst][write_idx] = b;
    g_rx_write_idx[inst] = next;
}

/* ========================================================================== */
/* Local Functions: RX interrupt operations                                   */
/* ========================================================================== */

/* -------------------------------------------------------------------------- */
/* uart_rx_irq_set_priority                                                   */
/* -------------------------------------------------------------------------- */
static bool uart_rx_irq_set_priority(nora_uart_instance_t inst, uint8_t prio)
{
    return nora_uart_dspic33ak_device_set_rx_irq_priority(inst, prio);
}

/* -------------------------------------------------------------------------- */
/* uart_rx_irq_clear_flag                                                      */
/* -------------------------------------------------------------------------- */
static bool uart_rx_irq_clear_flag(nora_uart_instance_t inst)
{
    return nora_uart_dspic33ak_device_rx_irq_clear_flag(inst);
}

/* -------------------------------------------------------------------------- */
/* uart_rx_irq_enable                                                         */
/* -------------------------------------------------------------------------- */
static bool uart_rx_irq_enable(nora_uart_instance_t inst, bool enable)
{
    return nora_uart_dspic33ak_device_rx_irq_enable(inst, enable);
}

/* -------------------------------------------------------------------------- */
/* uart_rx_irq_get_enable                                                      */
/* -------------------------------------------------------------------------- */
static uint8_t uart_rx_irq_get_enable(nora_uart_instance_t inst)
{
    bool enabled = false;

    if (!nora_uart_dspic33ak_device_rx_irq_is_enabled(inst, &enabled)) {
        return 0u;
    }

    return enabled ? 1u : 0u;
}

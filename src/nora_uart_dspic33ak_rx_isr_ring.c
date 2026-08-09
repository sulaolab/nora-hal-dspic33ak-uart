/* ========================================================================== */
/* Includes                                                                   */
/* ========================================================================== */

#include <string.h>
#include <stddef.h>

#include "dspic33ak_uart_rx_isr_ring.h"
#include "dspic33ak_uart_device.h"
#include "dspic33ak_uart_reg.h"

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
 * (dspic33ak_uart_get_device) and the dspic33ak_uart_reg.h bit-mask helpers, so
 * no UxSTATbits / UxRXB / _UxRXIF / _UxRXIE / _UxRXIP symbol names appear here.
 *
 * No printf / halt / blocking calls and no application dependencies. The
 * _UxRXInterrupt vector is NOT defined here; the application owns the vector and
 * calls dspic33ak_uart_rx_irq_handler() from it.
 */

/* ========================================================================== */
/* Module Variables                                                           */
/* ========================================================================== */

/*
 * Single-producer (ISR) / single-consumer (reader) ring, per instance.
 *   - g_rx_write_idx is advanced only by dspic33ak_uart_rx_irq_handler().
 *   - g_rx_read_idx is advanced only by dspic33ak_uart_rx_isr_read_byte().
 * The buffer itself is owned by the caller (passed in via config).
 */
static uint8_t *g_rx_ring[DSPIC33AK_UART_INST_COUNT];
static uint16_t g_rx_ring_size[DSPIC33AK_UART_INST_COUNT];
static volatile uint16_t g_rx_read_idx[DSPIC33AK_UART_INST_COUNT];
static volatile uint16_t g_rx_write_idx[DSPIC33AK_UART_INST_COUNT];
static volatile dspic33ak_uart_rx_isr_status_t g_rx_status[DSPIC33AK_UART_INST_COUNT];
static bool g_rx_isr_configured[DSPIC33AK_UART_INST_COUNT];

/* ========================================================================== */
/* Local Function Prototypes                                                  */
/* ========================================================================== */

static bool uart_inst_is_valid(dspic33ak_uart_instance_t inst);
static const dspic33ak_uart_regs_t *uart_regs(dspic33ak_uart_instance_t inst);

static bool uart_rx_irq_set_priority(dspic33ak_uart_instance_t inst, uint8_t prio);
static bool uart_rx_irq_clear_flag(dspic33ak_uart_instance_t inst);
static bool uart_rx_irq_enable(dspic33ak_uart_instance_t inst, bool enable);
static uint8_t uart_rx_irq_get_enable(dspic33ak_uart_instance_t inst);

static void uart_rx_ring_push(dspic33ak_uart_instance_t inst, uint8_t b);

/* ========================================================================== */
/* Public Functions                                                           */
/* ========================================================================== */

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_rx_isr_config                                               */
/* -------------------------------------------------------------------------- */
dspic33ak_uart_status_t dspic33ak_uart_rx_isr_config(
    dspic33ak_uart_instance_t inst,
    const dspic33ak_uart_rx_isr_config_t *config)
{
    const dspic33ak_uart_regs_t *r;

    if (!uart_inst_is_valid(inst)) {
        return DSPIC33AK_UART_ERR_INVALID_ARG;
    }
    if (config == NULL || config->buffer == NULL || config->buffer_size < 2u) {
        return DSPIC33AK_UART_ERR_INVALID_ARG;
    }

    r = uart_regs(inst);
    if (r == NULL) {
        return DSPIC33AK_UART_ERR_NOT_PRESENT;
    }
    if (!dspic33ak_uart_is_initialized(inst)) {
        return DSPIC33AK_UART_ERR_NOT_INITIALIZED;
    }

    /* Reject instances with no RX interrupt mapping before changing any state. */
    if (!uart_rx_irq_set_priority(inst, config->irq_priority)) {
        return DSPIC33AK_UART_ERR_UNSUPPORTED;
    }

    /* Bind the caller's buffer and reset ring + status counters. */
    g_rx_ring[inst]      = config->buffer;
    g_rx_ring_size[inst] = config->buffer_size;
    g_rx_read_idx[inst]  = 0u;
    g_rx_write_idx[inst] = 0u;
    memset(config->buffer, 0x00, config->buffer_size);
    memset((void *)&g_rx_status[inst], 0x00, sizeof(g_rx_status[inst]));

    /* Interrupt when >= 1 char is in the RX FIFO (RXWM = 0). */
    dspic33ak_uart_reg_clear(r->STAT, DSPIC33AK_UART_STAT_RXWM);

    /* Priority already set above; clear any stale flag, leave disabled. */
    (void)uart_rx_irq_clear_flag(inst);

    g_rx_isr_configured[inst] = true;

    return DSPIC33AK_UART_OK;
}

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_rx_isr_enable                                               */
/* -------------------------------------------------------------------------- */
dspic33ak_uart_status_t dspic33ak_uart_rx_isr_enable(
    dspic33ak_uart_instance_t inst)
{
    if (!uart_inst_is_valid(inst)) {
        return DSPIC33AK_UART_ERR_INVALID_ARG;
    }
    if (!g_rx_isr_configured[inst]) {
        /* No "not configured" status code exists; the ring is unusable until
         * dspic33ak_uart_rx_isr_config() succeeds, so report not-initialized. */
        return DSPIC33AK_UART_ERR_NOT_INITIALIZED;
    }
    if (!dspic33ak_uart_is_initialized(inst)) {
        return DSPIC33AK_UART_ERR_NOT_INITIALIZED;
    }

    (void)uart_rx_irq_clear_flag(inst);
    if (!uart_rx_irq_enable(inst, true)) {
        return DSPIC33AK_UART_ERR_UNSUPPORTED;
    }

    return DSPIC33AK_UART_OK;
}

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_rx_isr_disable                                              */
/* -------------------------------------------------------------------------- */
dspic33ak_uart_status_t dspic33ak_uart_rx_isr_disable(
    dspic33ak_uart_instance_t inst)
{
    if (!uart_inst_is_valid(inst)) {
        return DSPIC33AK_UART_ERR_INVALID_ARG;
    }

    /* Disable is the safe direction: allowed even when not configured. */
    if (!uart_rx_irq_enable(inst, false)) {
        return DSPIC33AK_UART_ERR_UNSUPPORTED;
    }
    (void)uart_rx_irq_clear_flag(inst);

    return DSPIC33AK_UART_OK;
}

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_rx_isr_ready                                                */
/* -------------------------------------------------------------------------- */
bool dspic33ak_uart_rx_isr_ready(dspic33ak_uart_instance_t inst)
{
    if (!uart_inst_is_valid(inst) || !g_rx_isr_configured[inst]) {
        return false;
    }

    return (g_rx_read_idx[inst] != g_rx_write_idx[inst]);
}

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_rx_isr_read_byte                                            */
/* -------------------------------------------------------------------------- */
dspic33ak_uart_status_t dspic33ak_uart_rx_isr_read_byte(
    dspic33ak_uart_instance_t inst,
    uint8_t *data)
{
    uint16_t read_idx;
    uint16_t next;

    if (data == NULL) {
        return DSPIC33AK_UART_ERR_INVALID_ARG;
    }
    if (!uart_inst_is_valid(inst) || !g_rx_isr_configured[inst]) {
        return DSPIC33AK_UART_ERR_RX_EMPTY;
    }
    if (g_rx_read_idx[inst] == g_rx_write_idx[inst]) {
        return DSPIC33AK_UART_ERR_RX_EMPTY;
    }

    read_idx = g_rx_read_idx[inst];
    *data = g_rx_ring[inst][read_idx];

    next = (uint16_t)(read_idx + 1u);
    if (next >= g_rx_ring_size[inst]) {
        next = 0u;
    }
    g_rx_read_idx[inst] = next;

    return DSPIC33AK_UART_OK;
}

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_rx_isr_flush                                                */
/* -------------------------------------------------------------------------- */
void dspic33ak_uart_rx_isr_flush(dspic33ak_uart_instance_t inst)
{
    const dspic33ak_uart_regs_t *r;
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

    while (!dspic33ak_uart_reg_is_set(r->STAT, DSPIC33AK_UART_STAT_RXBE)) {
        (void)(*r->RXB);                      /* drain the hardware RX FIFO too */
    }
    if (dspic33ak_uart_reg_is_set(r->STAT, DSPIC33AK_UART_STAT_RXFOIF)) {
        dspic33ak_uart_reg_clear(r->STAT, DSPIC33AK_UART_STAT_RXFOIF);
    }

    (void)uart_rx_irq_enable(inst, (ie != 0u));
}

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_rx_isr_status_get                                           */
/* -------------------------------------------------------------------------- */
void dspic33ak_uart_rx_isr_status_get(
    dspic33ak_uart_instance_t inst,
    dspic33ak_uart_rx_isr_status_t *status)
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
/* dspic33ak_uart_rx_isr_status_clear                                         */
/* -------------------------------------------------------------------------- */
void dspic33ak_uart_rx_isr_status_clear(dspic33ak_uart_instance_t inst)
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
/* dspic33ak_uart_rx_irq_handler                                              */
/* -------------------------------------------------------------------------- */
void dspic33ak_uart_rx_irq_handler(dspic33ak_uart_instance_t inst)
{
    const dspic33ak_uart_regs_t *r;
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
    while (!dspic33ak_uart_reg_is_set(r->STAT, DSPIC33AK_UART_STAT_RXBE)) {
        uint8_t b = (uint8_t)(*r->RXB);
        if (!dspic33ak_uart_async_rx_feed(inst, b)) {
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
    if (dspic33ak_uart_reg_is_set(r->STAT, DSPIC33AK_UART_STAT_RXFOIF)) {
        g_rx_status[inst].rx_fifo_overflow_count++;
        dspic33ak_uart_reg_clear(r->STAT, DSPIC33AK_UART_STAT_RXFOIF);
        events |= DSPIC33AK_UART_EVENT_RX_OVERRUN_ERROR;
    }
    if (dspic33ak_uart_reg_is_set(r->STAT, DSPIC33AK_UART_STAT_FERIF)) {
        g_rx_status[inst].framing_error_count++;
        dspic33ak_uart_reg_clear(r->STAT, DSPIC33AK_UART_STAT_FERIF);
        events |= DSPIC33AK_UART_EVENT_RX_FRAMING_ERROR;
    }
    if (dspic33ak_uart_reg_is_set(r->STAT, DSPIC33AK_UART_STAT_PERIF)) {
        g_rx_status[inst].parity_error_count++;
        dspic33ak_uart_reg_clear(r->STAT, DSPIC33AK_UART_STAT_PERIF);
        events |= DSPIC33AK_UART_EVENT_RX_PARITY_ERROR;
    }
    if (dspic33ak_uart_reg_is_set(r->STAT, DSPIC33AK_UART_STAT_ABDOVIF)) {
        g_rx_status[inst].autobaud_overflow_count++;
        dspic33ak_uart_reg_clear(r->STAT, DSPIC33AK_UART_STAT_ABDOVIF);
    }
    if (dspic33ak_uart_reg_is_set(r->STAT, DSPIC33AK_UART_STAT_TXCIF)) {
        g_rx_status[inst].tx_collision_count++;
        dspic33ak_uart_reg_clear(r->STAT, DSPIC33AK_UART_STAT_TXCIF);
    }

    /* Software ring overflow (a byte was dropped during the drain above). */
    if (g_rx_status[inst].rx_ring_overflow_count != ring_overflow_before) {
        events |= DSPIC33AK_UART_EVENT_RX_OVERFLOW;
    }

    /* Unsolicited byte-stream data landed in the ring this pass. */
    if (ring_got_data) {
        events |= DSPIC33AK_UART_EVENT_RX_READY;
    }

    if (events != 0u) {
        dspic33ak_uart_async_notify_events(inst, events);
    }
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
/* uart_regs                                                                  */
/* -------------------------------------------------------------------------- */
static const dspic33ak_uart_regs_t *uart_regs(dspic33ak_uart_instance_t inst)
{
    const dspic33ak_uart_device_t *dev = dspic33ak_uart_get_device(inst);

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
static void uart_rx_ring_push(dspic33ak_uart_instance_t inst, uint8_t b)
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
static bool uart_rx_irq_set_priority(dspic33ak_uart_instance_t inst, uint8_t prio)
{
    return dspic33ak_uart_device_set_rx_irq_priority(inst, prio);
}

/* -------------------------------------------------------------------------- */
/* uart_rx_irq_clear_flag                                                      */
/* -------------------------------------------------------------------------- */
static bool uart_rx_irq_clear_flag(dspic33ak_uart_instance_t inst)
{
    const dspic33ak_uart_device_t *dev = dspic33ak_uart_get_device(inst);

    return (dev != 0) && dspic33ak_uart_reg_irq_clear(&dev->regs.irq_rx);
}

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
/* uart_rx_irq_get_enable                                                      */
/* -------------------------------------------------------------------------- */
static uint8_t uart_rx_irq_get_enable(dspic33ak_uart_instance_t inst)
{
    const dspic33ak_uart_device_t *dev = dspic33ak_uart_get_device(inst);

    return (dev != 0) ?
        dspic33ak_uart_reg_irq_is_enabled(&dev->regs.irq_rx) :
        0u;
}

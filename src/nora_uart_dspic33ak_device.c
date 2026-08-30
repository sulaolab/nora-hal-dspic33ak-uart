/* ========================================================================== */
/* Includes                                                                   */
/* ========================================================================== */

#include <xc.h>
#include "nora_uart_dspic33ak_device.h"

/* ========================================================================== */
/* Module Overview                                                            */
/* ========================================================================== */

/*
 * Device / instance mapping layer.
 *
 * This is the only place that should know about U1CON / U2CON / U3CON / U4CON
 * symbol names and the matching CPU UART RX/TX interrupt flag, enable, and
 * priority mappings. Driver logic uses only the register pointer table returned
 * from nora_uart_dspic33ak_get_device() and the internal priority setter functions.
 */

/* ========================================================================== */
/* Module Constants                                                           */
/* ========================================================================== */

/*
 * Per-instance interrupt mapping.
 *
 * The register table below carries only data/config registers.  The RX/TX
 * interrupt flag and enable bits are reached through the DFP bit aliases
 * (_UxRXIF / _UxRXIE / _UxTXIF / _UxTXIE) in the functions at the bottom of this
 * file, not through a { &IFSn, &IECn, mask } descriptor.  Two reasons:
 *
 *  - Atomicity.  IFSx/IECx are shared by every peripheral.  A pointer+mask
 *    read-modify-write can undo a bit another interrupt changed between the read
 *    and the write-back.  Writing the alias with a literal gives one bset.b /
 *    bclr.b instead.
 *  - Portability.  The bank differs per device: on dsPIC33AK512MPS512 all UART
 *    flags are in IFS3/IEC3, on dsPIC33AK128MC106 U1/U2 are in IFS2/IEC2.  The
 *    alias hides that, so no bank number appears in this code at all.
 *
 * The guards below preserve the regression guard the old #elif/#else chain gave:
 * if a UART instance exists but the DFP does not provide its flag/enable aliases
 * the build stops here, instead of shipping a UART whose RX enable silently fails
 * (as happened for AK128 U1/U2 when only the IFS3 arm existed).
 */
#if defined(U1CON) && !(defined(_U1RXIF) && defined(_U1RXIE) && \
                        defined(_U1TXIF) && defined(_U1TXIE))
#error "UART1 interrupt flag/enable aliases are not implemented"
#endif
#if defined(U2CON) && !(defined(_U2RXIF) && defined(_U2RXIE) && \
                        defined(_U2TXIF) && defined(_U2TXIE))
#error "UART2 interrupt flag/enable aliases are not implemented"
#endif
#if defined(U3CON) && !(defined(_U3RXIF) && defined(_U3RXIE) && \
                        defined(_U3TXIF) && defined(_U3TXIE))
#error "UART3 interrupt flag/enable aliases are not implemented"
#endif
#if defined(U4CON) && !(defined(_U4RXIF) && defined(_U4RXIE) && \
                        defined(_U4TXIF) && defined(_U4TXIE))
#error "UART4 interrupt flag/enable aliases are not implemented"
#endif

static const nora_uart_dspic33ak_device_t g_uart_devices[NORA_UART_INST_COUNT] = {
#if defined(U1CON)
    [NORA_UART_INST_1] = {
        .present = true,
        .regs = {
            .CON = &U1CON,
            .STAT = &U1STAT,
            .BRG = &U1BRG,
            .TXB = &U1TXB,
            .RXB = &U1RXB,
        },
    },
#else
    [NORA_UART_INST_1] = { .present = false },
#endif

#if defined(U2CON)
    [NORA_UART_INST_2] = {
        .present = true,
        .regs = {
            .CON = &U2CON,
            .STAT = &U2STAT,
            .BRG = &U2BRG,
            .TXB = &U2TXB,
            .RXB = &U2RXB,
        },
    },
#else
    [NORA_UART_INST_2] = { .present = false },
#endif

#if defined(U3CON)
    [NORA_UART_INST_3] = {
        .present = true,
        .regs = {
            .CON = &U3CON,
            .STAT = &U3STAT,
            .BRG = &U3BRG,
            .TXB = &U3TXB,
            .RXB = &U3RXB,
        },
    },
#else
    [NORA_UART_INST_3] = { .present = false },
#endif

#if defined(U4CON)
    [NORA_UART_INST_4] = {
        .present = true,
        .regs = {
            .CON = &U4CON,
            .STAT = &U4STAT,
            .BRG = &U4BRG,
            .TXB = &U4TXB,
            .RXB = &U4RXB,
        },
    },
#else
    [NORA_UART_INST_4] = { .present = false },
#endif
};

/* ========================================================================== */
/* Internal API                                                               */
/* ========================================================================== */

/* -------------------------------------------------------------------------- */
/* nora_uart_dspic33ak_get_device                                                  */
/* -------------------------------------------------------------------------- */
const nora_uart_dspic33ak_device_t *nora_uart_dspic33ak_get_device(
    nora_uart_instance_t inst)
{
    /* NORA_UART_INST_SUPPORTED_COUNT, not the enum count: an instance the
     * project narrowed away has no per-instance state, so it must report
     * absent here -- that is what makes nora_uart_is_present() honest and
     * keeps init failing with NOT_PRESENT instead of writing out of range.
     * The register table itself stays full-width (it is const, in flash). */
    if ((unsigned)inst >= (unsigned)NORA_UART_INST_SUPPORTED_COUNT) {
        return 0;
    }

    if (!g_uart_devices[inst].present) {
        return 0;
    }

    return &g_uart_devices[inst];
}

/* -------------------------------------------------------------------------- */
/* nora_uart_dspic33ak_instance_is_present                                         */
/* -------------------------------------------------------------------------- */
bool nora_uart_dspic33ak_instance_is_present(nora_uart_instance_t inst)
{
    return (nora_uart_dspic33ak_get_device(inst) != 0);
}

/* -------------------------------------------------------------------------- */
/* nora_uart_dspic33ak_device_set_rx_irq_priority                                  */
/* -------------------------------------------------------------------------- */
bool nora_uart_dspic33ak_device_set_rx_irq_priority(
    nora_uart_instance_t inst,
    uint8_t priority)
{
    switch (inst) {
#if defined(_U1RXIP)
    case NORA_UART_INST_1: _U1RXIP = priority; return true;
#endif
#if defined(_U2RXIP)
    case NORA_UART_INST_2: _U2RXIP = priority; return true;
#endif
#if defined(_U3RXIP)
    case NORA_UART_INST_3: _U3RXIP = priority; return true;
#endif
#if defined(_U4RXIP)
    case NORA_UART_INST_4: _U4RXIP = priority; return true;
#endif
    default: break;
    }

    return false;
}

/* -------------------------------------------------------------------------- */
/* nora_uart_dspic33ak_device_set_tx_irq_priority                                  */
/* -------------------------------------------------------------------------- */
bool nora_uart_dspic33ak_device_set_tx_irq_priority(
    nora_uart_instance_t inst,
    uint8_t priority)
{
    switch (inst) {
#if defined(_U1TXIP)
    case NORA_UART_INST_1: _U1TXIP = priority; return true;
#endif
#if defined(_U2TXIP)
    case NORA_UART_INST_2: _U2TXIP = priority; return true;
#endif
#if defined(_U3TXIP)
    case NORA_UART_INST_3: _U3TXIP = priority; return true;
#endif
#if defined(_U4TXIP)
    case NORA_UART_INST_4: _U4TXIP = priority; return true;
#endif
    default: break;
    }

    return false;
}

/* -------------------------------------------------------------------------- */
/* Interrupt flag / enable access (DFP bit aliases)                           */
/*                                                                            */
/* Every store below is a literal into a named bit alias, which is what makes  */
/* it a single atomic bit operation on a shared IFSx/IECx register.  The       */
/* enable/disable pair is an if/else rather than `_UxRXIE = value` on purpose:  */
/* assigning a runtime value to a bit alias compiles to a byte-wide            */
/* read-modify-write, which is exactly the hazard this layer exists to avoid.   */
/* -------------------------------------------------------------------------- */

bool nora_uart_dspic33ak_device_rx_irq_clear_flag(nora_uart_instance_t inst)
{
    switch (inst) {
#if defined(_U1RXIF)
    case NORA_UART_INST_1: _U1RXIF = 0; return true;
#endif
#if defined(_U2RXIF)
    case NORA_UART_INST_2: _U2RXIF = 0; return true;
#endif
#if defined(_U3RXIF)
    case NORA_UART_INST_3: _U3RXIF = 0; return true;
#endif
#if defined(_U4RXIF)
    case NORA_UART_INST_4: _U4RXIF = 0; return true;
#endif
    default: break;
    }

    return false;
}

bool nora_uart_dspic33ak_device_rx_irq_enable(nora_uart_instance_t inst,
                                                  bool enable)
{
    switch (inst) {
#if defined(_U1RXIE)
    case NORA_UART_INST_1:
        if (enable) { _U1RXIE = 1; } else { _U1RXIE = 0; }
        return true;
#endif
#if defined(_U2RXIE)
    case NORA_UART_INST_2:
        if (enable) { _U2RXIE = 1; } else { _U2RXIE = 0; }
        return true;
#endif
#if defined(_U3RXIE)
    case NORA_UART_INST_3:
        if (enable) { _U3RXIE = 1; } else { _U3RXIE = 0; }
        return true;
#endif
#if defined(_U4RXIE)
    case NORA_UART_INST_4:
        if (enable) { _U4RXIE = 1; } else { _U4RXIE = 0; }
        return true;
#endif
    default: break;
    }

    return false;
}

bool nora_uart_dspic33ak_device_rx_irq_is_enabled(nora_uart_instance_t inst,
                                                      bool *enabled)
{
    if (enabled == 0) {
        return false;
    }

    switch (inst) {
#if defined(_U1RXIE)
    case NORA_UART_INST_1: *enabled = (_U1RXIE != 0u); return true;
#endif
#if defined(_U2RXIE)
    case NORA_UART_INST_2: *enabled = (_U2RXIE != 0u); return true;
#endif
#if defined(_U3RXIE)
    case NORA_UART_INST_3: *enabled = (_U3RXIE != 0u); return true;
#endif
#if defined(_U4RXIE)
    case NORA_UART_INST_4: *enabled = (_U4RXIE != 0u); return true;
#endif
    default: break;
    }

    return false;
}

bool nora_uart_dspic33ak_device_tx_irq_clear_flag(nora_uart_instance_t inst)
{
    switch (inst) {
#if defined(_U1TXIF)
    case NORA_UART_INST_1: _U1TXIF = 0; return true;
#endif
#if defined(_U2TXIF)
    case NORA_UART_INST_2: _U2TXIF = 0; return true;
#endif
#if defined(_U3TXIF)
    case NORA_UART_INST_3: _U3TXIF = 0; return true;
#endif
#if defined(_U4TXIF)
    case NORA_UART_INST_4: _U4TXIF = 0; return true;
#endif
    default: break;
    }

    return false;
}

/* Software-triggered TX interrupt: the byte-stream TX path kicks the ISR by
 * setting the flag itself.  One bset.b, same reasoning as the clear above. */
bool nora_uart_dspic33ak_device_tx_irq_raise_flag(nora_uart_instance_t inst)
{
    switch (inst) {
#if defined(_U1TXIF)
    case NORA_UART_INST_1: _U1TXIF = 1; return true;
#endif
#if defined(_U2TXIF)
    case NORA_UART_INST_2: _U2TXIF = 1; return true;
#endif
#if defined(_U3TXIF)
    case NORA_UART_INST_3: _U3TXIF = 1; return true;
#endif
#if defined(_U4TXIF)
    case NORA_UART_INST_4: _U4TXIF = 1; return true;
#endif
    default: break;
    }

    return false;
}

bool nora_uart_dspic33ak_device_tx_irq_enable(nora_uart_instance_t inst,
                                                  bool enable)
{
    switch (inst) {
#if defined(_U1TXIE)
    case NORA_UART_INST_1:
        if (enable) { _U1TXIE = 1; } else { _U1TXIE = 0; }
        return true;
#endif
#if defined(_U2TXIE)
    case NORA_UART_INST_2:
        if (enable) { _U2TXIE = 1; } else { _U2TXIE = 0; }
        return true;
#endif
#if defined(_U3TXIE)
    case NORA_UART_INST_3:
        if (enable) { _U3TXIE = 1; } else { _U3TXIE = 0; }
        return true;
#endif
#if defined(_U4TXIE)
    case NORA_UART_INST_4:
        if (enable) { _U4TXIE = 1; } else { _U4TXIE = 0; }
        return true;
#endif
    default: break;
    }

    return false;
}

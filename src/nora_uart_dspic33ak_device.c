/* ========================================================================== */
/* Includes                                                                   */
/* ========================================================================== */

#include <xc.h>
#include "dspic33ak_uart_device.h"

/* ========================================================================== */
/* Module Overview                                                            */
/* ========================================================================== */

/*
 * Device / instance mapping layer.
 *
 * This is the only place that should know about U1CON / U2CON / U3CON / U4CON
 * symbol names and the matching CPU UART RX/TX interrupt flag, enable, and
 * priority mappings. Driver logic uses only the register pointer table returned
 * from dspic33ak_uart_get_device() and the internal priority setter functions.
 */

/* ========================================================================== */
/* Module Constants                                                           */
/* ========================================================================== */

/*
 * Interrupt flag/enable descriptor per UART instance.
 *
 * The UART RX/TX interrupt flags live in different IFS/IEC banks depending on
 * the device: on dsPIC33AK512MPS512 all UART flags are in IFS3/IEC3, while on
 * dsPIC33AK128MC106 the U1/U2 flags are in IFS2/IEC2 (U3 stays in IFS3). The
 * bank is therefore selected per (instance, direction) by whichever
 * _IFSn_UxyIF_MASK the DFP defines: IFS3 first, then IFS2. Exactly one is
 * defined per instance per device, so this is self-selecting; adding a device
 * that uses another bank means adding one more #elif arm here. If no arm matches
 * for a present UART (as happened for AK128 U1/U2 when only the IFS3 arm existed)
 * the #else arm fires a #error at build time -- previously the descriptor was left
 * zeroed, which made ISR-ring RX enable fail and unwound uart init, silently
 * disabling the UART. The build now stops instead of shipping a dead port.
 */
static const dspic33ak_uart_device_t g_uart_devices[DSPIC33AK_UART_INST_COUNT] = {
#if defined(U1CON)
    [DSPIC33AK_UART_INST_1] = {
        .present = true,
        .regs = {
            .CON = &U1CON,
            .STAT = &U1STAT,
            .BRG = &U1BRG,
            .TXB = &U1TXB,
            .RXB = &U1RXB,
#if defined(_U1RXIF) && defined(_IFS3_U1RXIF_MASK)
            .irq_rx = { &IFS3, &IEC3, _IFS3_U1RXIF_MASK },
#elif defined(_U1RXIF) && defined(_IFS2_U1RXIF_MASK)
            .irq_rx = { &IFS2, &IEC2, _IFS2_U1RXIF_MASK },
#else
            #error "UART1 RX interrupt mapping is not implemented"
#endif
#if defined(_U1TXIF) && defined(_IFS3_U1TXIF_MASK)
            .irq_tx = { &IFS3, &IEC3, _IFS3_U1TXIF_MASK },
#elif defined(_U1TXIF) && defined(_IFS2_U1TXIF_MASK)
            .irq_tx = { &IFS2, &IEC2, _IFS2_U1TXIF_MASK },
#else
            #error "UART1 TX interrupt mapping is not implemented"
#endif
        },
    },
#else
    [DSPIC33AK_UART_INST_1] = { .present = false },
#endif

#if defined(U2CON)
    [DSPIC33AK_UART_INST_2] = {
        .present = true,
        .regs = {
            .CON = &U2CON,
            .STAT = &U2STAT,
            .BRG = &U2BRG,
            .TXB = &U2TXB,
            .RXB = &U2RXB,
#if defined(_U2RXIF) && defined(_IFS3_U2RXIF_MASK)
            .irq_rx = { &IFS3, &IEC3, _IFS3_U2RXIF_MASK },
#elif defined(_U2RXIF) && defined(_IFS2_U2RXIF_MASK)
            .irq_rx = { &IFS2, &IEC2, _IFS2_U2RXIF_MASK },
#else
            #error "UART2 RX interrupt mapping is not implemented"
#endif
#if defined(_U2TXIF) && defined(_IFS3_U2TXIF_MASK)
            .irq_tx = { &IFS3, &IEC3, _IFS3_U2TXIF_MASK },
#elif defined(_U2TXIF) && defined(_IFS2_U2TXIF_MASK)
            .irq_tx = { &IFS2, &IEC2, _IFS2_U2TXIF_MASK },
#else
            #error "UART2 TX interrupt mapping is not implemented"
#endif
        },
    },
#else
    [DSPIC33AK_UART_INST_2] = { .present = false },
#endif

#if defined(U3CON)
    [DSPIC33AK_UART_INST_3] = {
        .present = true,
        .regs = {
            .CON = &U3CON,
            .STAT = &U3STAT,
            .BRG = &U3BRG,
            .TXB = &U3TXB,
            .RXB = &U3RXB,
#if defined(_U3RXIF) && defined(_IFS3_U3RXIF_MASK)
            .irq_rx = { &IFS3, &IEC3, _IFS3_U3RXIF_MASK },
#elif defined(_U3RXIF) && defined(_IFS2_U3RXIF_MASK)
            .irq_rx = { &IFS2, &IEC2, _IFS2_U3RXIF_MASK },
#else
            #error "UART3 RX interrupt mapping is not implemented"
#endif
#if defined(_U3TXIF) && defined(_IFS3_U3TXIF_MASK)
            .irq_tx = { &IFS3, &IEC3, _IFS3_U3TXIF_MASK },
#elif defined(_U3TXIF) && defined(_IFS2_U3TXIF_MASK)
            .irq_tx = { &IFS2, &IEC2, _IFS2_U3TXIF_MASK },
#else
            #error "UART3 TX interrupt mapping is not implemented"
#endif
        },
    },
#else
    [DSPIC33AK_UART_INST_3] = { .present = false },
#endif

#if defined(U4CON)
    [DSPIC33AK_UART_INST_4] = {
        .present = true,
        .regs = {
            .CON = &U4CON,
            .STAT = &U4STAT,
            .BRG = &U4BRG,
            .TXB = &U4TXB,
            .RXB = &U4RXB,
#if defined(_U4RXIF) && defined(_IFS3_U4RXIF_MASK)
            .irq_rx = { &IFS3, &IEC3, _IFS3_U4RXIF_MASK },
#elif defined(_U4RXIF) && defined(_IFS2_U4RXIF_MASK)
            .irq_rx = { &IFS2, &IEC2, _IFS2_U4RXIF_MASK },
#else
            #error "UART4 RX interrupt mapping is not implemented"
#endif
#if defined(_U4TXIF) && defined(_IFS3_U4TXIF_MASK)
            .irq_tx = { &IFS3, &IEC3, _IFS3_U4TXIF_MASK },
#elif defined(_U4TXIF) && defined(_IFS2_U4TXIF_MASK)
            .irq_tx = { &IFS2, &IEC2, _IFS2_U4TXIF_MASK },
#else
            #error "UART4 TX interrupt mapping is not implemented"
#endif
        },
    },
#else
    [DSPIC33AK_UART_INST_4] = { .present = false },
#endif
};

/* ========================================================================== */
/* Internal API                                                               */
/* ========================================================================== */

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_get_device                                                  */
/* -------------------------------------------------------------------------- */
const dspic33ak_uart_device_t *dspic33ak_uart_get_device(
    dspic33ak_uart_instance_t inst)
{
    if ((unsigned)inst >= (unsigned)DSPIC33AK_UART_INST_COUNT) {
        return 0;
    }

    if (!g_uart_devices[inst].present) {
        return 0;
    }

    return &g_uart_devices[inst];
}

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_instance_is_present                                         */
/* -------------------------------------------------------------------------- */
bool dspic33ak_uart_instance_is_present(dspic33ak_uart_instance_t inst)
{
    return (dspic33ak_uart_get_device(inst) != 0);
}

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_device_set_rx_irq_priority                                  */
/* -------------------------------------------------------------------------- */
bool dspic33ak_uart_device_set_rx_irq_priority(
    dspic33ak_uart_instance_t inst,
    uint8_t priority)
{
    switch (inst) {
#if defined(_U1RXIP)
    case DSPIC33AK_UART_INST_1: _U1RXIP = priority; return true;
#endif
#if defined(_U2RXIP)
    case DSPIC33AK_UART_INST_2: _U2RXIP = priority; return true;
#endif
#if defined(_U3RXIP)
    case DSPIC33AK_UART_INST_3: _U3RXIP = priority; return true;
#endif
#if defined(_U4RXIP)
    case DSPIC33AK_UART_INST_4: _U4RXIP = priority; return true;
#endif
    default: break;
    }

    return false;
}

/* -------------------------------------------------------------------------- */
/* dspic33ak_uart_device_set_tx_irq_priority                                  */
/* -------------------------------------------------------------------------- */
bool dspic33ak_uart_device_set_tx_irq_priority(
    dspic33ak_uart_instance_t inst,
    uint8_t priority)
{
    switch (inst) {
#if defined(_U1TXIP)
    case DSPIC33AK_UART_INST_1: _U1TXIP = priority; return true;
#endif
#if defined(_U2TXIP)
    case DSPIC33AK_UART_INST_2: _U2TXIP = priority; return true;
#endif
#if defined(_U3TXIP)
    case DSPIC33AK_UART_INST_3: _U3TXIP = priority; return true;
#endif
#if defined(_U4TXIP)
    case DSPIC33AK_UART_INST_4: _U4TXIP = priority; return true;
#endif
    default: break;
    }

    return false;
}

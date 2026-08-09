#ifndef NORA_UART_DSPIC33AK_REG_H
#define NORA_UART_DSPIC33AK_REG_H

/* ========================================================================== */
/* Includes                                                                   */
/* ========================================================================== */

#include <stdint.h>
#include <stdbool.h>

/* ========================================================================== */
/* Module Overview                                                            */
/* ========================================================================== */

/*
 * Internal register helper layer.
 *
 * This file intentionally uses 32-bit register pointers and bit masks instead
 * of XC-DSC bitfield structures such as U1CONbits. This keeps compiler / DFP
 * bitfield details away from the public UART API.
 *
 * Bit positions were checked against:
 *   Microchip.dsPIC33AK-MP_DFP.1.3.185
 *   xc16/support/dsPIC33A/h/p33AK512MPS512.h
 *   _U1CON_*_POSITION / _U1STAT_*_POSITION macros
 */

/* ========================================================================== */
/* Internal Types                                                             */
/* ========================================================================== */

/*
 * Only the per-UART data/config registers are described by a pointer table.
 *
 * The RX/TX interrupt flag and enable bits are deliberately NOT here.  They live
 * in IFSx/IECx, which are shared by every peripheral on the device, so they must
 * be touched one bit at a time - `*iec |= mask` with a runtime pointer and a
 * runtime mask is a 32-bit read-modify-write of a register another interrupt may
 * change in between, and that write-back silently restores the old value of every
 * other bit.  The device layer therefore exposes named per-instance entry points
 * (nora_uart_dspic33ak_device_rx_irq_enable() and friends) that write the DFP bit
 * alias `_UxRXIE` directly, where register, bit and value are all compile-time
 * constants and XC-DSC emits a single bset.b / bclr.b.  It also means the IFS/IEC
 * bank number per device no longer has to be carried in this code at all.
 */
typedef struct {
    volatile uint32_t *CON;
    volatile uint32_t *STAT;
    volatile uint32_t *BRG;
    volatile uint32_t *TXB;
    volatile uint32_t *RXB;
} nora_uart_dspic33ak_regs_t;

/* ========================================================================== */
/* Register Bit Masks                                                         */
/* ========================================================================== */

/* UxCON bits (enable + baud / clock control) */
#define NORA_UART_DSPIC33AK_CON_RXEN     (1UL << 4)   /* UxCONbits.RXEN   (pos 0x04) */
#define NORA_UART_DSPIC33AK_CON_TXEN     (1UL << 5)   /* UxCONbits.TXEN   (pos 0x05) */
#define NORA_UART_DSPIC33AK_CON_BRGS     (1UL << 7)   /* UxCONbits.BRGS   (pos 0x07) */
#define NORA_UART_DSPIC33AK_CON_ON       (1UL << 15)  /* UxCONbits.ON     (pos 0x0F) */
#define NORA_UART_DSPIC33AK_CON_CLKSEL   (1UL << 25)  /* UxCONbits.CLKSEL (pos 0x19) */
#define NORA_UART_DSPIC33AK_CON_CLKMOD   (1UL << 27)  /* UxCONbits.CLKMOD (pos 0x1B) */

/* UxSTAT bits (status used by the byte-stream API and the RX ISR ring) */
#define NORA_UART_DSPIC33AK_STAT_TXCIF   (1UL << 0)   /* UxSTATbits.TXCIF  (pos 0x00) */
#define NORA_UART_DSPIC33AK_STAT_RXFOIF  (1UL << 1)   /* UxSTATbits.RXFOIF (pos 0x01) */
#define NORA_UART_DSPIC33AK_STAT_FERIF   (1UL << 3)   /* UxSTATbits.FERIF  (pos 0x03) */
#define NORA_UART_DSPIC33AK_STAT_ABDOVIF (1UL << 5)   /* UxSTATbits.ABDOVIF(pos 0x05) */
#define NORA_UART_DSPIC33AK_STAT_PERIF   (1UL << 6)   /* UxSTATbits.PERIF  (pos 0x06) */
#define NORA_UART_DSPIC33AK_STAT_TXMTIF  (1UL << 7)   /* UxSTATbits.TXMTIF (pos 0x07) */
#define NORA_UART_DSPIC33AK_STAT_RXBE    (1UL << 17)  /* UxSTATbits.RXBE   (pos 0x11) */
#define NORA_UART_DSPIC33AK_STAT_TXBF    (1UL << 20)  /* UxSTATbits.TXBF   (pos 0x14) */
#define NORA_UART_DSPIC33AK_STAT_TXBE    (1UL << 21)  /* UxSTATbits.TXBE   (pos 0x15) */
#define NORA_UART_DSPIC33AK_STAT_TXWRE   (1UL << 23)  /* UxSTATbits.TXWRE  (pos 0x17) */
#define NORA_UART_DSPIC33AK_STAT_RXWM    (1UL << 24)  /* UxSTATbits.RXWM   (pos 0x18) */

/* ========================================================================== */
/* Internal Inline Helpers                                                    */
/* ========================================================================== */

/* -------------------------------------------------------------------------- */
/* nora_uart_dspic33ak_reg_set                                                     */
/* -------------------------------------------------------------------------- */
static inline void nora_uart_dspic33ak_reg_set(volatile uint32_t *reg, uint32_t mask)
{
    *reg |= mask;
}

/* -------------------------------------------------------------------------- */
/* nora_uart_dspic33ak_reg_clear                                                   */
/* -------------------------------------------------------------------------- */
static inline void nora_uart_dspic33ak_reg_clear(volatile uint32_t *reg, uint32_t mask)
{
    *reg &= ~mask;
}

/* -------------------------------------------------------------------------- */
/* nora_uart_dspic33ak_reg_is_set                                                  */
/* -------------------------------------------------------------------------- */
static inline bool nora_uart_dspic33ak_reg_is_set(volatile uint32_t *reg, uint32_t mask)
{
    return ((*reg & mask) != 0u);
}

/* -------------------------------------------------------------------------- */
/* nora_uart_dspic33ak_reg_write_field                                             */
/* -------------------------------------------------------------------------- */
static inline void nora_uart_dspic33ak_reg_write_field(
    volatile uint32_t *reg,
    uint32_t mask,
    uint32_t value)
{
    *reg = (*reg & ~mask) | (value & mask);
}

#endif /* NORA_UART_DSPIC33AK_REG_H */

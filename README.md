# dspic33ak-hal-uart

> Want to run it on hardware first?
> Start with [dspic33ak-hal-starter](https://github.com/sulaolab/dspic33ak-hal-starter),
> which vendors validated snapshots of the dsPIC33AK HAL repositories and
> provides a ready-to-build MPLAB X project for the dsPIC33AK Curiosity board.

Small, readable UART HAL for Microchip dsPIC33AK devices.

This repository provides a reusable byte-stream UART driver with a clean public API. The public API avoids exposing XC-DSC / DFP bitfield types, while device-specific register mapping is isolated in small internal files.

## Status

Hardware-validated release candidate.

The HAL core is hardware-validated through
[dspic33ak-hal-starter](https://github.com/sulaolab/dspic33ak-hal-starter)
on dsPIC33AK512MPS512 / UART1. It is intended to be reusable across
dsPIC33AK projects where the board/application code owns clock setup, PPS
routing, GPIO setup, interrupt vector definitions, and stdio retargeting.

This version supports:

* Polling RX mode
* Interrupt-driven RX software-ring mode
* Caller-provided RX ring buffer storage
* dsPIC33AK UART clock source through CLKGEN8
* 8N1 byte-stream operation
* Non-blocking async TX with `DSPIC33AK_UART_EVENT_SEND_COMPLETE`
* Non-blocking async RX with `DSPIC33AK_UART_EVENT_RX_COMPLETE`
* Active TX/RX abort and transfer byte counts
* Optional asynchronous (non-blocking) TX/RX transfer model with completion and
  error events through a registered callback (additive; the byte-stream API and
  RX ISR ring keep working unchanged)

Hardware validation currently covers:

* Neutral, application-owned RX vector forwarding
* Neutral, application-owned TX vector forwarding
* RX ISR ring receive path
* Async TX start, TX interrupt service, SEND_COMPLETE callback, exact TX count,
  and physical TX done check
* Async RX `start_clean`, RX interrupt service, RX_COMPLETE callback, exact RX
  count, and received data match
* Active RX abort
* Return to normal printf / RX echo operation after async self-test cleanup

The same UART core was also integration-validated in the upstream audio application
through both the legacy UART initialization route and the CMSIS USART route.
Application-level console RX remained functional while audio/TDM continued
running with `TDM miss = 0`.

## Repository layout

```text
src/
  dspic33ak_uart.h              Public UART HAL API
  dspic33ak_uart.c              UART HAL implementation
  dspic33ak_uart_device.h       Device / IRQ mapping interface
  dspic33ak_uart_device.c       Device register and RX/TX IRQ mapping
  dspic33ak_uart_reg.h          Internal register / bit-mask helper definitions
  dspic33ak_uart_rx_isr_ring.h  RX ISR ring backend API
  dspic33ak_uart_rx_isr_ring.c  RX ISR ring backend implementation
examples/
  uart_async_example.c           Minimal async TX/RX integration example
```

## Device and IRQ mapping

Device-specific UART peripheral and CPU interrupt mappings are isolated in
`dspic33ak_uart_device.c`.

For each supported UART instance, the device layer maps:

* `UxCON` / `UxSTAT` / `UxBRG` / `UxTXB` / `UxRXB`
* RX interrupt flag / enable / priority
* TX interrupt flag / enable / priority

The main UART logic and RX ISR-ring backend use the mapped register and IRQ
descriptors instead of directly referencing raw `_UxRXIF`, `_UxTXIE`, or
`_UxRXIP`-style symbols.

Actual `_UxRXInterrupt` and `_UxTXInterrupt` vector definitions remain owned by
the application or integration layer.

## Design policy

* Public API does not expose XC-DSC / DFP bitfield types.
* Board-specific CLKGEN8 setup, PPS routing, and GPIO routing stay outside this HAL.
* `printf()`, `read()`, `write()`, and other stdio retargeting stay outside this HAL.
* Application console / command parsing stays outside this HAL.
* No dynamic memory allocation is used.
* RX ISR ring buffer storage is caller-provided.
* Interrupt vector ownership stays outside this HAL.
* The HAL does not define `_UxRXInterrupt` or `_UxTXInterrupt`.
* In ISR ring mode, the application-owned RX interrupt vector calls
  `dspic33ak_uart_rx_irq_handler()`.
* When async TX is used, the application-owned TX interrupt vector calls
  `dspic33ak_uart_tx_irq_handler()`.
* Public functions are placed near the top of each source file. Static helper functions are placed near the bottom, with only prototypes near the top when needed.

## Clock assumption

This is a **dsPIC33AK CLKGEN8 UART HAL**.

`dspic33ak_uart_init()` selects the UART clock source as CLKGEN8 and uses fractional baud mode. The board/application code must configure and enable CLKGEN8 before calling `dspic33ak_uart_init()`.

The value passed as `config.uart_clk_hz` must be the CLKGEN8 frequency used by the UART baud-rate generator.

The HAL does not configure:

* Clock generator bring-up
* PPS input/output routing
* GPIO direction
* Analog-disable settings
* Board-level pin selection

## Public API overview

The main public header is:

```c
#include "dspic33ak_uart.h"
```

Core API groups:

```text
Initialization:
  dspic33ak_uart_init()
  dspic33ak_uart_deinit()
  dspic33ak_uart_is_present()
  dspic33ak_uart_is_initialized()

Status:
  dspic33ak_uart_rx_ready()
  dspic33ak_uart_tx_ready()
  dspic33ak_uart_tx_done()

Byte I/O:
  dspic33ak_uart_write_byte()
  dspic33ak_uart_read_byte()

Buffer helpers:
  dspic33ak_uart_write()
  dspic33ak_uart_read()

RX cleanup:
  dspic33ak_uart_rx_flush()

Events / callbacks:
  dspic33ak_uart_set_callback()

Async transfers:
  dspic33ak_uart_tx_start()
  dspic33ak_uart_rx_start()
  dspic33ak_uart_rx_start_clean()
  dspic33ak_uart_tx_abort()
  dspic33ak_uart_rx_abort()
  dspic33ak_uart_tx_count_get()
  dspic33ak_uart_rx_count_get()
  dspic33ak_uart_tx_is_busy()
  dspic33ak_uart_rx_is_busy()
```

RX backend selection is configured per UART instance:

```text
DSPIC33AK_UART_RX_MODE_POLLING
  RX is read directly from the hardware RX FIFO.
  No RX interrupt is enabled.

DSPIC33AK_UART_RX_MODE_ISR_RING
  RX interrupt drains the hardware RX FIFO into a caller-provided software ring.
  dspic33ak_uart_rx_ready(), dspic33ak_uart_read_byte(), and
  dspic33ak_uart_rx_flush() operate on the software ring.
```

Only these two RX modes are valid. `dspic33ak_uart_init()` rejects other `rx_mode` values with `DSPIC33AK_UART_ERR_INVALID_ARG`.

## Build notes

Add these C files to your project:

```text
src/dspic33ak_uart.c
src/dspic33ak_uart_device.c
src/dspic33ak_uart_rx_isr_ring.c
```

Add `src/` to your include path.

Application code should normally include only:

```c
#include "dspic33ak_uart.h"
```

If your application defines an RX interrupt vector for ISR ring mode, that vector file should also include:

```c
#include "dspic33ak_uart_rx_isr_ring.h"
```

The header `dspic33ak_uart_reg.h` is an internal helper header used by the HAL implementation. It is part of the source distribution, but user application code should normally not include it directly. (The asynchronous engine's internal hooks are declared in `dspic33ak_uart_rx_isr_ring.h`; they are likewise not for application use.)

If your application uses the asynchronous transfer model and starts TX transfers (`dspic33ak_uart_tx_start()`), the application-owned `_UxTXInterrupt` vector must call `dspic33ak_uart_tx_irq_handler()`, the same way the RX vector calls `dspic33ak_uart_rx_irq_handler()`.

Async transfer-state rules:

* Async TX requires TX enabled and a non-zero `tx_irq_priority`; otherwise `dspic33ak_uart_tx_start()` returns `DSPIC33AK_UART_ERR_UNSUPPORTED` (a transfer with no servicing interrupt would never complete).
* Async RX requires RX enabled and ISR ring mode; otherwise `dspic33ak_uart_rx_start()` returns `DSPIC33AK_UART_ERR_UNSUPPORTED`.
* `dspic33ak_uart_rx_start_clean()` is intended for framed/request-style receive APIs that want to discard old ring/FIFO bytes before arming a new async receive.
* `dspic33ak_uart_tx_enable(false)` / `dspic33ak_uart_rx_enable(false)` return `DSPIC33AK_UART_ERR_BUSY` while an async transfer is active, so a transfer is never stranded by disabling its line mid-flight.
* Register the callback after `dspic33ak_uart_init()`. Init and deinit clear callback state.
* The callback may run from interrupt context. Keep it short and non-blocking:
  record event bits or counters only. Do not call `printf()`, blocking I/O,
  delay routines, or HAL APIs from inside the callback.
* Async TX buffers must remain valid until the transfer completes or is aborted.
* Do not mix `printf()`, blocking write, or `dspic33ak_uart_write_byte()` on the
  same UART while an async TX transfer is active.
* `DSPIC33AK_UART_EVENT_SEND_COMPLETE` means all bytes have been submitted to
  the hardware FIFO/register. It does not guarantee physical line idle. Before
  returning to blocking output, wait for SEND_COMPLETE and then confirm
  `dspic33ak_uart_tx_done()`.
* `dspic33ak_uart_rx_start_clean()` discards old RX ring data and hardware FIFO
  data before arming the new receive. This is the preferred primitive for
  CMSIS-style `Receive()` wrappers and other framed receive operations that
  should observe only bytes arriving for that operation.

## Minimal polling example

```c
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "dspic33ak_uart.h"

static uint32_t app_get_ms(void)
{
    /* Return a monotonic millisecond tick, or set get_ms = NULL to disable timeout handling. */
    return 0u;
}

void app_uart_init(void)
{
    dspic33ak_uart_config_t cfg;

    memset(&cfg, 0, sizeof(cfg));

    cfg.uart_clk_hz = 100000000u;  /* CLKGEN8 frequency */
    cfg.baudrate    = 115200u;
    cfg.timeout_ms  = 10u;
    cfg.get_ms      = app_get_ms;

    cfg.data_bits   = 8u;
    cfg.stop_bits   = 1u;
    cfg.parity      = DSPIC33AK_UART_PARITY_NONE;

    cfg.enable_tx   = true;
    cfg.enable_rx   = true;

    cfg.rx_mode     = DSPIC33AK_UART_RX_MODE_POLLING;

    /*
     * Board-level CLKGEN8 / PPS / GPIO setup must be completed before this call.
     */
    (void)dspic33ak_uart_init(DSPIC33AK_UART_INST_1, &cfg);
}

void app_uart_poll(void)
{
    uint8_t ch;

    if (dspic33ak_uart_read_byte(DSPIC33AK_UART_INST_1, &ch) == DSPIC33AK_UART_OK) {
        (void)dspic33ak_uart_write_byte(DSPIC33AK_UART_INST_1, ch); /* echo */
    }
}
```

## Minimal ISR ring example

In ISR ring mode, the application provides the RX ring buffer storage and owns the interrupt vector.

```c
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "dspic33ak_uart.h"
#include "dspic33ak_uart_rx_isr_ring.h"

static uint8_t uart1_rx_ring[256];

void app_uart_init(void)
{
    dspic33ak_uart_config_t cfg;

    memset(&cfg, 0, sizeof(cfg));

    cfg.uart_clk_hz = 100000000u;  /* CLKGEN8 frequency */
    cfg.baudrate    = 115200u;
    cfg.timeout_ms  = 10u;
    cfg.get_ms      = 0;

    cfg.data_bits   = 8u;
    cfg.stop_bits   = 1u;
    cfg.parity      = DSPIC33AK_UART_PARITY_NONE;

    cfg.enable_tx   = true;
    cfg.enable_rx   = true;

    cfg.rx_mode             = DSPIC33AK_UART_RX_MODE_ISR_RING;
    cfg.rx_ring_buffer      = uart1_rx_ring;
    cfg.rx_ring_buffer_size = sizeof(uart1_rx_ring);
    cfg.rx_irq_priority     = 3u;

    /*
     * Board-level CLKGEN8 / PPS / GPIO setup must be completed before this call.
     */
    (void)dspic33ak_uart_init(DSPIC33AK_UART_INST_1, &cfg);
}

/*
 * Example interrupt vector wrapper.
 *
 * Adjust the interrupt attribute and vector name as needed for your project,
 * device, and compiler settings.
 */
void __attribute__((interrupt, context)) _U1RXInterrupt(void)
{
    dspic33ak_uart_rx_irq_handler(DSPIC33AK_UART_INST_1);
}
```

After initialization, application code can still use the same backend-agnostic RX API:

```c
uint8_t ch;

if (dspic33ak_uart_read_byte(DSPIC33AK_UART_INST_1, &ch) == DSPIC33AK_UART_OK) {
    /* ch came from the ISR software ring in ISR ring mode. */
}
```

## Minimal async TX/RX example

For a compact integration sketch covering application-owned RX/TX vectors,
RX ring storage, IRQ priorities, callback registration after init,
SEND_COMPLETE, `tx_done()`, `rx_start_clean()`, RX_COMPLETE, aborts, and counts,
see:

```text
examples/uart_async_example.c
```

## Scope and limitations

This repository provides a UART byte-stream HAL. It does not provide a complete board support package.

Current scope:

* dsPIC33AK UART instances supported by the device header
* 8 data bits
* No parity
* 1 stop bit
* Blocking TX byte write with optional timeout
* Non-blocking RX byte read
* Polling RX backend
* ISR software-ring RX backend
* Non-blocking async TX
* Non-blocking async RX
* SEND_COMPLETE and RX_COMPLETE callback events
* TX/RX abort
* TX/RX transfer byte counts

You still need project-specific code for:

* CLKGEN8 setup
* PPS input/output routing
* GPIO direction / analog-disable setup
* Interrupt vector wrapper
* stdio retargeting
* Application console or command parser
* Device-specific board initialization

## Notes for ISR ring mode

The RX ISR ring backend is included in this repository, but the interrupt vector itself is not.

The application must define the relevant `_UxRXInterrupt` vector and call:

```c
dspic33ak_uart_rx_irq_handler(DSPIC33AK_UART_INST_1);
```

from that vector.

When async TX is used, the application must also define the relevant
`_UxTXInterrupt` vector and call:

```c
dspic33ak_uart_tx_irq_handler(DSPIC33AK_UART_INST_1);
```

from that vector.

The HAL owns the RX FIFO drain logic and software-ring write/read indices. The caller owns the ring buffer memory.

This keeps the HAL reusable while avoiding project-specific interrupt vector ownership inside the driver.

## License

MIT No Attribution License (MIT-0). See [LICENSE](LICENSE).

Attribution is appreciated but not required.

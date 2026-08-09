# nora-hal-dspic33ak-uart

**NORA-HAL** — *Native On-chip Resource Assistant*

Small, readable UART HAL for Microchip dsPIC33AK devices — part of **NORA-HAL**, a
HAL family whose public API is namespaced `nora_*` / `NORA_*`.

> Want to run it on hardware first?
> Start with [dspic33ak-hal-starter](https://github.com/sulaolab/dspic33ak-hal-starter),
> which vendors validated snapshots of the NORA-HAL repositories and
> provides a ready-to-build MPLAB X project for the dsPIC33AK Curiosity board.

> **This repository is a published snapshot, not the development tree.** Every
> file under `src/` is byte-identical to its counterpart in
> `dspic33ak-hal-starter`, which is in turn byte-identical to the audio-board
> project that runs these sources on hardware. Fixes flow *into* here from that
> validated tree — see [docs/nora_migration.md](docs/nora_migration.md).
>
> **One exception, 2026-08-09.** Comments and the folder README under `src/` were
> corrected *here first*, ahead of the upstream tree: stale file names left behind by
> the rename, `Nora` where the family name is `NORA`, and `dsPIC33A` where the text
> means the dsPIC33AK backend. **No executable code changed.** The same corrections are
> queued for upstream; the files are listed in
> [docs/nora_migration.md](docs/nora_migration.md).

This repository provides a reusable byte-stream UART driver with a clean public API. The public API avoids exposing XC-DSC / DFP bitfield types, while device-specific register mapping is isolated in small internal files.

## Naming

The public API is `nora_*` / `NORA_*`. It replaces the `dspic33ak_*` /
`DSPIC33AK_*` namespace this repository used before 2026-08, and **there are no
compatibility aliases** — a consumer moving to this version renames its call
sites. Apart from the rename the public API is unchanged, with one addition:
`nora_uart.h` now *declares* `nora_uart_rx_irq_handler()`, which the previous
header only mentioned in prose and left every application to declare itself.
The rest of the work in this version is below the contract (the interrupt bits
are now written through the DFP bit aliases instead of a pointer table).

The chip name survives in exactly two places, both deliberate:

* **Implementation file names** carry a backend tag: `nora_uart_dspic33ak.c` is
  the dsPIC33AK backend of the processor-neutral `nora_uart.h`. A second
  processor would add `nora_uart_<tag>.c` beside it, not a second header.
* **Backend-private identifiers** inside those files (register-layer macros and
  statics), which no caller sees. The device / register / ISR-ring internals
  carry the tag for the same reason.

The tag is `_dspic33ak`, the device family this backend actually drives — not
`_dspic33a`, which is the *core* family name (dsPIC33A) and one level too
coarse. A dsPIC33CK backend would be tagged `_dspic33ck`: a different silicon
family (dsPIC33**C**), and never abbreviated to `_dspic33c`.

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
* Non-blocking async TX with `NORA_UART_EVENT_SEND_COMPLETE`
* Non-blocking async RX with `NORA_UART_EVENT_RX_COMPLETE`
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
  nora_uart.h              Public UART HAL API
  nora_uart_dspic33ak.c              UART HAL implementation
  nora_uart_dspic33ak_device.h       Device / IRQ mapping interface
  nora_uart_dspic33ak_device.c       Device register and RX/TX IRQ mapping
  nora_uart_dspic33ak_reg.h          Internal register / bit-mask helper definitions
  nora_uart_dspic33ak_rx_isr_ring.h  RX ISR ring backend API
  nora_uart_dspic33ak_rx_isr_ring.c  RX ISR ring backend implementation
examples/
  uart_async_example.c           Minimal async TX/RX integration example
```

## Device and IRQ mapping

Device-specific UART peripheral and CPU interrupt mappings are isolated in
`nora_uart_dspic33ak_device.c`.

For each supported UART instance, the device layer maps:

* `UxCON` / `UxSTAT` / `UxBRG` / `UxTXB` / `UxRXB`
* RX interrupt flag / enable / priority
* TX interrupt flag / enable / priority

The main UART logic and the RX ISR-ring backend reach the peripheral registers
through the mapped descriptor, and the CPU interrupt bits through small
per-instance accessors in the same device file (`..._rx_irq_clear_flag()`,
`..._set_rx_irq_priority()`, and so on).

Those accessors write the DFP bit aliases (`_UxRXIF`, `_UxRXIE`, `_UxRXIP`, ...)
directly, in a `switch` over the instance, rather than through a table of
`IFSx` / `IECx` pointers plus a mask. This is deliberate, and it replaced a
pointer table that used to live in `nora_uart_dspic33ak_reg.h`:

* an alias write is a single-bit operation only when the bit is known at compile
  time — a pointer-and-mask read-modify-write of `IECx` is not
* a hand-maintained pointer table can name the wrong `IFS` register for one
  instance and silently kill that instance's RX, with nothing to catch it at
  build time

An instance whose `UxCON` exists but whose flag/enable aliases do not is now a
compile-time `#error` rather than a runtime surprise.

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
  `nora_uart_rx_irq_handler()`.
* When async TX is used, the application-owned TX interrupt vector calls
  `nora_uart_tx_irq_handler()`.
* Public functions are placed near the top of each source file. Static helper functions are placed near the bottom, with only prototypes near the top when needed.

## Clock assumption

This is a **dsPIC33AK CLKGEN8 UART HAL**.

`nora_uart_init()` selects the UART clock source as CLKGEN8 and uses fractional baud mode. The board/application code must configure and enable CLKGEN8 before calling `nora_uart_init()`.

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
#include "nora_uart.h"
```

Core API groups:

```text
Initialization:
  nora_uart_init()
  nora_uart_deinit()
  nora_uart_is_present()
  nora_uart_is_initialized()

Status:
  nora_uart_rx_ready()
  nora_uart_tx_ready()
  nora_uart_tx_done()

Byte I/O:
  nora_uart_write_byte()
  nora_uart_read_byte()

Buffer helpers:
  nora_uart_write()
  nora_uart_read()

RX cleanup:
  nora_uart_rx_flush()

Events / callbacks:
  nora_uart_set_callback()

Async transfers:
  nora_uart_tx_start()
  nora_uart_rx_start()
  nora_uart_rx_start_clean()
  nora_uart_tx_abort()
  nora_uart_rx_abort()
  nora_uart_tx_count_get()
  nora_uart_rx_count_get()
  nora_uart_tx_is_busy()
  nora_uart_rx_is_busy()
```

RX backend selection is configured per UART instance:

```text
NORA_UART_RX_MODE_POLLING
  RX is read directly from the hardware RX FIFO.
  No RX interrupt is enabled.

NORA_UART_RX_MODE_ISR_RING
  RX interrupt drains the hardware RX FIFO into a caller-provided software ring.
  nora_uart_rx_ready(), nora_uart_read_byte(), and
  nora_uart_rx_flush() operate on the software ring.
```

Only these two RX modes are valid. `nora_uart_init()` rejects other `rx_mode` values with `NORA_UART_ERR_INVALID_ARG`.

## Build notes

Add these C files to your project:

```text
src/nora_uart_dspic33ak.c
src/nora_uart_dspic33ak_device.c
src/nora_uart_dspic33ak_rx_isr_ring.c
```

Add `src/` to your include path.

Application code should normally include only:

```c
#include "nora_uart.h"
```

If your application defines an RX interrupt vector for ISR ring mode, that vector file should also include:

```c
#include "nora_uart_dspic33ak_rx_isr_ring.h"
```

The header `nora_uart_dspic33ak_reg.h` is an internal helper header used by the HAL implementation. It is part of the source distribution, but user application code should normally not include it directly. (The asynchronous engine's internal hooks are declared in `nora_uart_dspic33ak_rx_isr_ring.h`; they are likewise not for application use.)

If your application uses the asynchronous transfer model and starts TX transfers (`nora_uart_tx_start()`), the application-owned `_UxTXInterrupt` vector must call `nora_uart_tx_irq_handler()`, the same way the RX vector calls `nora_uart_rx_irq_handler()`.

Async transfer-state rules:

* Async TX requires TX enabled and a non-zero `tx_irq_priority`; otherwise `nora_uart_tx_start()` returns `NORA_UART_ERR_UNSUPPORTED` (a transfer with no servicing interrupt would never complete).
* Async RX requires RX enabled and ISR ring mode; otherwise `nora_uart_rx_start()` returns `NORA_UART_ERR_UNSUPPORTED`.
* `nora_uart_rx_start_clean()` is intended for framed/request-style receive APIs that want to discard old ring/FIFO bytes before arming a new async receive.
* `nora_uart_tx_enable(false)` / `nora_uart_rx_enable(false)` return `NORA_UART_ERR_BUSY` while an async transfer is active, so a transfer is never stranded by disabling its line mid-flight.
* Register the callback after `nora_uart_init()`. Init and deinit clear callback state.
* The callback may run from interrupt context. Keep it short and non-blocking:
  record event bits or counters only. Do not call `printf()`, blocking I/O,
  delay routines, or HAL APIs from inside the callback.
* Async TX buffers must remain valid until the transfer completes or is aborted.
* Do not mix `printf()`, blocking write, or `nora_uart_write_byte()` on the
  same UART while an async TX transfer is active.
* `NORA_UART_EVENT_SEND_COMPLETE` means all bytes have been submitted to
  the hardware FIFO/register. It does not guarantee physical line idle. Before
  returning to blocking output, wait for SEND_COMPLETE and then confirm
  `nora_uart_tx_done()`.
* `nora_uart_rx_start_clean()` discards old RX ring data and hardware FIFO
  data before arming the new receive. This is the preferred primitive for
  CMSIS-style `Receive()` wrappers and other framed receive operations that
  should observe only bytes arriving for that operation.

## Minimal polling example

```c
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "nora_uart.h"

static uint32_t app_get_ms(void)
{
    /* Return a monotonic millisecond tick, or set get_ms = NULL to disable timeout handling. */
    return 0u;
}

void app_uart_init(void)
{
    nora_uart_config_t cfg;

    memset(&cfg, 0, sizeof(cfg));

    cfg.uart_clk_hz = 100000000u;  /* CLKGEN8 frequency */
    cfg.baudrate    = 115200u;
    cfg.timeout_ms  = 10u;
    cfg.get_ms      = app_get_ms;

    cfg.data_bits   = 8u;
    cfg.stop_bits   = 1u;
    cfg.parity      = NORA_UART_PARITY_NONE;

    cfg.enable_tx   = true;
    cfg.enable_rx   = true;

    cfg.rx_mode     = NORA_UART_RX_MODE_POLLING;

    /*
     * Board-level CLKGEN8 / PPS / GPIO setup must be completed before this call.
     */
    (void)nora_uart_init(NORA_UART_INST_1, &cfg);
}

void app_uart_poll(void)
{
    uint8_t ch;

    if (nora_uart_read_byte(NORA_UART_INST_1, &ch) == NORA_UART_OK) {
        (void)nora_uart_write_byte(NORA_UART_INST_1, ch); /* echo */
    }
}
```

## Minimal ISR ring example

In ISR ring mode, the application provides the RX ring buffer storage and owns the interrupt vector.

```c
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "nora_uart.h"
#include "nora_uart_dspic33ak_rx_isr_ring.h"

static uint8_t uart1_rx_ring[256];

void app_uart_init(void)
{
    nora_uart_config_t cfg;

    memset(&cfg, 0, sizeof(cfg));

    cfg.uart_clk_hz = 100000000u;  /* CLKGEN8 frequency */
    cfg.baudrate    = 115200u;
    cfg.timeout_ms  = 10u;
    cfg.get_ms      = 0;

    cfg.data_bits   = 8u;
    cfg.stop_bits   = 1u;
    cfg.parity      = NORA_UART_PARITY_NONE;

    cfg.enable_tx   = true;
    cfg.enable_rx   = true;

    cfg.rx_mode             = NORA_UART_RX_MODE_ISR_RING;
    cfg.rx_ring_buffer      = uart1_rx_ring;
    cfg.rx_ring_buffer_size = sizeof(uart1_rx_ring);
    cfg.rx_irq_priority     = 3u;

    /*
     * Board-level CLKGEN8 / PPS / GPIO setup must be completed before this call.
     */
    (void)nora_uart_init(NORA_UART_INST_1, &cfg);
}

/*
 * Example interrupt vector wrapper.
 *
 * Adjust the interrupt attribute and vector name as needed for your project,
 * device, and compiler settings.
 */
void __attribute__((interrupt, context)) _U1RXInterrupt(void)
{
    nora_uart_rx_irq_handler(NORA_UART_INST_1);
}
```

After initialization, application code can still use the same backend-agnostic RX API:

```c
uint8_t ch;

if (nora_uart_read_byte(NORA_UART_INST_1, &ch) == NORA_UART_OK) {
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
nora_uart_rx_irq_handler(NORA_UART_INST_1);
```

from that vector.

When async TX is used, the application must also define the relevant
`_UxTXInterrupt` vector and call:

```c
nora_uart_tx_irq_handler(NORA_UART_INST_1);
```

from that vector.

The HAL owns the RX FIFO drain logic and software-ring write/read indices. The caller owns the ring buffer memory.

This keeps the HAL reusable while avoiding project-specific interrupt vector ownership inside the driver.

## License

MIT No Attribution License (MIT-0). See [LICENSE](LICENSE).

Attribution is appreciated but not required.

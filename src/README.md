# NORA UART HAL

`nora_uart.h` is the public UART byte-stream contract. It provides configuration,
blocking byte and buffer I/O, optional ISR-ring receive, asynchronous transfer
callbacks, and runtime status without exposing compiler SFR types or register names.

## Ownership boundary

- Application and board code select `nora_uart_instance_t`, its role (console,
  telemetry, or another feature), clock frequency, pin routing, and GPIO state.
- The HAL accepts that selected instance and configuration; it does not choose a
  console UART or require one logical instance to map to the same physical UART on
  every NORA-supported device.
- The dsPIC33AK backend owns register access, IRQ mappings and device availability.
  Its headers and source use explicit `nora_uart_dspic33ak_*` names.
- Platform-owned interrupt vectors forward to `nora_uart_rx_irq_handler()` and
  `nora_uart_tx_irq_handler()`. The handlers are ordinary functions, not vectors.

## Non-goals

PPS, GPIO, clock-generator setup, stdio retargeting, console parsing and application
protocols are outside this HAL. The CMSIS USART adapter consumes the public NORA API,
but its ARM CMSIS contract is maintained separately.

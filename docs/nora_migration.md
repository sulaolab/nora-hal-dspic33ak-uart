# NORA migration — where these sources come from (2026-08-09)

This repository used to be the place the UART HAL was edited. It is not any more.
Since the NORA migration it is a **published snapshot**: `src/` is filled from
the tree that is actually built and run on hardware, and this file records which
tree, which commit, and how the equality was checked.

For this module the **public API is effectively unchanged apart from the rename** —
but the implementation is not. The refresh brings across the fleet-wide
interrupt-access rework described below, which is in fact the rework this module's
own bug prompted.

## The chain

```
dsp-sonora audio board project        the tree that runs on hardware
  main = 91adb63
        |  vendored, byte-for-byte
        v
sulaolab/dspic33ak-hal-starter        MPLAB X project, 11 HAL modules
  refactor/nora-hal = b70982d
        |  published, byte-for-byte
        v
sulaolab/nora-hal-dspic33ak-uart      this repository
```

Direction matters: it used to run the other way (the starter vendored *from* the
standalone repos). It was reversed because only the board project runs the console
under real interrupt load, so it is the only place a fix can be validated before
it is published. A fix made here and not upstream would be a fork.

## What the two commits did

| commit | what |
|---|---|
| `510e4ef` | **rename only** — 7 files, all detected as R100 (100 % similarity). No byte of content changed, so the rename is reviewable on its own. |
| `092f676` | **content refresh** — the 7 files replaced with the starter's bytes, plus a new `src/README.md`. |

### The rename mapping

| before | after | why |
|---|---|---|
| `src/dspic33ak_uart.h` | `src/nora_uart.h` | public header: no chip in the name |
| `src/dspic33ak_uart.c` | `src/nora_uart_dspic33ak.c` | backend: tagged |
| `src/dspic33ak_uart_device.c` | `src/nora_uart_dspic33ak_device.c` | backend: tagged |
| `src/dspic33ak_uart_device.h` | `src/nora_uart_dspic33ak_device.h` | backend-private: tagged |
| `src/dspic33ak_uart_reg.h` | `src/nora_uart_dspic33ak_reg.h` | backend-private register layer: tagged |
| `src/dspic33ak_uart_rx_isr_ring.c` | `src/nora_uart_dspic33ak_rx_isr_ring.c` | backend: tagged |
| `src/dspic33ak_uart_rx_isr_ring.h` | `src/nora_uart_dspic33ak_rx_isr_ring.h` | backend-private: tagged |
| *(new)* | `src/README.md` | source-tree guide, carried over from upstream |

The tag is `_dspic33ak`, not `_dspic33a`. `dsPIC33A` is the *core* family name;
these files drive dsPIC33AK UART SFRs. A dsPIC33CK backend would be `_dspic33ck` —
a different silicon family (dsPIC33**C**), never shortened to `_dspic33c`.

`nora_uart_dspic33ak_rx_isr_ring.h` keeps the tag even though application vector
code is told to include it: it is the backend's ISR entry, not the portable
contract. An application that switches processors changes that include; it does
not change its calls to `nora_uart.h`.

## Proof of identity with the upstream tree

Git blob hashes, this repository at `092f676` vs `dspic33ak-hal-starter` at
`b70982d`. Identical hash = identical bytes; git normalises EOLs into the blob on
both sides, so the CRLF working trees do not disturb the comparison.

| file | blob | bytes |
|---|---|---|
| `src/README.md` | `990e7ccd034f` | 1193 |
| `nora_uart.h` | `68c247e3a3e4` | 18309 |
| `nora_uart_dspic33ak.c` | `dbadd1e3c81b` | 43553 |
| `nora_uart_dspic33ak_device.c` | `5c149468d1f6` | 11584 |
| `nora_uart_dspic33ak_device.h` | `5fa91ac40be1` | 3091 |
| `nora_uart_dspic33ak_reg.h` | `569e62d97835` | 6035 |
| `nora_uart_dspic33ak_rx_isr_ring.c` | `4d5220856db4` | 18623 |
| `nora_uart_dspic33ak_rx_isr_ring.h` | `1ec2ba7e11ad` | 7517 |

**8 of 8 identical.**

## What actually changed in the content refresh

Method: take each new file, reverse the naming (`nora_` → `dspic33ak_`,
`NORA_` → `DSPIC33AK_`, and strip the `_dspic33ak` backend tag from the file
names), and diff against the pre-rename blob. Whatever is left is *not* naming.

| file | +added | −removed | non-comment | verdict |
|---|---|---|---|---|
| `nora_uart.h` | 26 | 22 | +2 / −0 | two added declarations |
| `nora_uart_dspic33ak.c` | 10 | 28 | +9 / −22 | real change |
| `nora_uart_dspic33ak_device.c` | 196 | 68 | +152 / −56 | real change |
| `nora_uart_dspic33ak_device.h` | 19 | 0 | +9 / −0 | real change |
| `nora_uart_dspic33ak_reg.h` | 14 | 71 | +0 / −52 | real change (removal only) |
| `nora_uart_dspic33ak_rx_isr_ring.c` | 8 | 16 | +7 / −13 | real change |
| `nora_uart_dspic33ak_rx_isr_ring.h` | 0 | 10 | +0 / −2 | real change (removal only) |
| `src/README.md` | — | — | — | new file |

**0 of 7 pure rename, 1 new file** — but read the non-comment columns before
reading that number. The public header's entire delta is two added lines.

### API delta: zero, with one caveat stated

**32 functions and 30 macros/enumerators, before and after.** One measurement
detail worth being explicit about, because it flatters the number: the two lines
`nora_uart.h` gained are the declarations of the IRQ handlers, and
`nora_uart_rx_irq_handler()` was previously *named in the header's prose* while
every application had to declare it itself. The name-set comparison therefore
finds it on both sides and reports no delta. That is not wrong — nothing a caller
could call has changed — but the honest description is "a function callers were
already told to call is now actually declared", not "nothing changed in the
header".

### What the rest of the delta consists of

One change, in one place, with consequences spread over five files: **CPU
interrupt bits are no longer reached through a table of `IFSx` / `IECx` pointers
plus a mask.** They are written through the DFP bit aliases (`_UxRXIF`, `_UxRXIE`,
`_UxRXIP`, …) in small per-instance accessors in `nora_uart_dspic33ak_device.c`.

That accounts for the shape of the numbers exactly:

* `_device.c` **+152 / −56** — the accessors, one `switch` arm per instance per
  bit, replacing the descriptor's interrupt fields.
* `_device.h` **+9 with nothing removed** — their declarations.
* `_reg.h` **−52 with nothing added** — the pointer table it used to hold is gone.
* `nora_uart_dspic33ak.c` **+9 / −22** and `_rx_isr_ring.c` **+7 / −13** — call
  sites moving from table lookups to accessor calls, which is shorter at each site.

Two reasons the fleet made this change, the second of them measured here:

* An alias write is a genuine single-bit operation only when the bit is known at
  compile time. A pointer-and-mask read-modify-write of `IECx` touches the whole
  word, which is not what a caller enabling one interrupt asked for.
* A hand-maintained pointer table can name the wrong `IFS` register for one
  instance and silently kill that instance's RX, with nothing to catch it at build
  time. **That is not hypothetical: it happened to UART1, whose table entry
  pointed at `IFS2` instead of `IFS3`, and the symptom was "printf works but keys
  do nothing" — TX fine, RX dead.** The rework, and the sweep across the other
  HALs, came out of that bug.

An instance whose `UxCON` exists but whose flag/enable aliases do not is now a
compile-time `#error` rather than a runtime surprise.

One consequence for readers of the old documentation: the README used to tell
integrators to reach the interrupt bits through the mapped descriptor rather than
the raw `_UxRXIF` aliases. That advice is now inverted for the CPU interrupt bits —
peripheral registers still go through the descriptor, interrupt bits go through
the accessors that write the aliases. The README has been corrected accordingly.

## Comment corrections made here, ahead of upstream

Everything above describes the state as published on 2026-08-08, when every file under
`src/` was byte-identical to upstream. On **2026-08-09** a documentation review found a
class of error that the identity proof above cannot see, and it was fixed here first
rather than waiting for the next upstream refresh.

* `src/README.md` — the title wrote the HAL family name as `Nora`, and the layering
  bullet said `dsPIC33A` where it means the dsPIC33AK backend. The `dsPIC33A/h/` DFP
  path in `nora_uart_dspic33ak_reg.h` was left alone; it is a literal directory name.

No executable code changed. The edits are comments and Markdown; the compiled
result is unchanged.

### Why the proof in "Proof of identity" does not catch this

Step 3 reverse-normalises the NORA names back to `dspic33ak_*` and diffs against the
pre-rename blob, so whatever is left is not naming. Two error classes cancel out exactly
in that diff and are therefore invisible to it:

* **A document reference to a file that was renamed.** A prose mention of
  `nora_<mod>_hw.{c,h}` reverse-normalises to `dspic33ak_<mod>_hw.{c,h}`, which is the
  *correct* pre-rename name — the diff is empty, yet the file is now called
  `nora_<mod>_dspic33ak_hw.{c,h}` and the reference is dead. The same cancellation hides
  `Nora` vs `NORA` and `dsPIC33A` vs `dsPIC33AK`: both sides of the diff are naming, so
  naming errors are exactly what it is blind to.
* **A document that omits a file the refresh added.** An absent line produces no diff
  line at all.

Both blind spots were observed across the NORA-HAL migration fleet; the subset that
affected *this* repository is the list above. Neither is detectable by
reverse-normalisation. What does detect them is resolving every `nora_*.{c,h}`
mentioned in prose against the actual contents of `src/`, and reading every
`dsPIC33A` / `Nora` hit rather than counting them — which is how these were found.

## Hardware evidence

There is no build or test in this repository — it is sources only. The evidence is
the upstream project's: `dspic33ak-hal-starter`
`docs/nora_hal_migration_analysis.md` §11e records a PASS run of all 11 NORA-ised
modules on PKOB4 `020085204RYN000057` (dsPIC33AK512MPS512, Device ID `0xa77c`) on
2026-08-09.

Scope, stated plainly: for this module the coverage is direct and it is the
strongest of the eleven, because the console in that run *is* this HAL — startup
log, interactive command entry, and RX echo all ran, which exercises init, the
baud generator, blocking TX, and the ISR ring RX path under interrupt. Interactive
RX in particular is the exact failure the interrupt rework addresses: had the
accessors named a wrong register, keys would not have worked. The async transfer
engine (TX/RX start, SEND_COMPLETE, aborts, counts) is **not** re-validated by
§11e; its evidence remains the async self-test recorded in the README's Status
section. Unlike the pure-rename modules, these bytes are **not** the ones this
repository published before.

## Consumer impact

* The public namespace changed from `dspic33ak_*` / `DSPIC33AK_*` to `nora_*` /
  `NORA_*` and **no compatibility aliases were added**. Call sites must be
  renamed; the substitution is purely textual.
* The `#include` names changed — see the rename mapping above. An application that
  defines the RX vector includes `nora_uart_dspic33ak_rx_isr_ring.h`.
* Nothing else at the API level. An application may now delete its own
  hand-written declaration of `nora_uart_rx_irq_handler()`; keeping it is harmless
  as long as it is spelled the same.
* **`dspic33ak-usart-cmsis-driver` will not build against this version.** Its
  `tools/sync_hal_from_upstream.py` hardcodes both the upstream repository name
  and the HAL file list, and its wrapper calls `dspic33ak_uart_*`. Both need
  updating; that work is tracked with the other CMSIS-Driver repositories, not
  here.

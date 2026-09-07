# Ruuvi post-mortem diagnostics v2

Built from the supplied `ruuvi.firmware.c-CRASH-DIAGS.zip` baseline.

## What changed

- `RESETREAS` is captured once at application entry, then those sticky bits are cleared.
  `DIAG?` reports the captured value for the current boot, so an older reset cannot
  contaminate a later report.
- The persistent FDS record now stores a reset source as well as optional fatal
  `rd_status_t`, source filename and line.
- Direct reset call sites that can safely persist FDS state are tagged:
  - `APP_FATAL`
  - `CART_GESTURE`
  - `BOOTLOADER_FALLBACK`
- `RSET!` was added as a validation command for a direct software reset that bypasses
  `app_on_error()`.
- Existing `CRASH`, `CLEAR`, and `DIAG?` remain.
- No diagnostic code uses GPREGRET or GPREGRET2.

Flash-init and Peer Manager failure paths still purge FDS before resetting, so FDS cannot
reliably preserve a source tag there. A fresh `RESETREAS=SREQ` with no persistent record
still distinguishes a software reset from watchdog/lockup/pin-reset classes, but those
storage-failure paths remain possible explanations for `SREQ + source=NONE`.

## Commands

Write these five ASCII bytes to Nordic UART RX (`6E400002...`), with notifications enabled
on Nordic UART TX (`6E400003...`):

- `DIAG?` - emit one PM v2 report
- `CLEAR` - delete the persistent post-mortem record
- `CRASH` - intentional fatal through `RD_ERROR_CHECK -> app_on_error -> reset`
- `RSET!` - intentional direct reset that bypasses `app_on_error`

## PM v2 packet

Exactly 20 bytes:

```
50 4D 02 SS FF RR RR RR RR EE EE EE EE LL LL HH HH TT TT 00
```

- `SS`: reset source
  - `0`: NONE
  - `1`: APP_FATAL
  - `2`: CART_GESTURE
  - `3`: BOOTLOADER_FALLBACK
  - `4`: DIAG_DIRECT
- `FF`: flags
  - bit 0: persistent record valid
  - bit 1: fatal data valid
- `RR`: captured boot `RESETREAS`, uint32 little-endian
- `EE`: fatal `rd_status_t`, uint32 little-endian
- `LL`: source line, uint16 little-endian
- `HH`: 16-bit filename hash
- `TT`: flash-load status low 16 bits

## Validation

### Fatal path

1. `CLEAR`
2. send `CRASH`
3. reconnect after reboot
4. enable TX notifications
5. send `DIAG?`

For this exact tree, the expected fatal report is:

```
50 4D 02 01 03 04 00 00 00 20 00 00 00 8A 01 C2 2C 00 00 00
```

That decodes to:

- source: `APP_FATAL`
- record/fatal valid: yes/yes
- RESETREAS: `0x00000004` (`SREQ`)
- error: `0x00000020` (`RD_ERROR_INVALID_STATE`)
- line: `394`
- file hash: `0x2CC2` (`app_comms.c`)

### Direct-reset path

1. `CLEAR`
2. send `RSET!`
3. reconnect
4. send `DIAG?`

Expected classification:

- source: `DIAG_DIRECT`
- record valid: yes
- fatal valid: no
- RESETREAS: `SREQ`

Use `decode_ruuvi_pmr.py` to decode a raw 20-byte packet.

## Real DFU failure

After a visible unexpected reboot, do not retry DFU first. Reconnect in nRF Connect,
enable UART TX notifications, and send `DIAG?`.

The important classifications are:

- `SREQ + APP_FATAL + fatal_valid=1`: exact application fatal; use error/file/line.
- `SREQ + tagged direct source`: explicit reset call site identified.
- `DOG + source=NONE`: watchdog reset.
- `LOCKUP + source=NONE`: CPU lockup class.
- `SREQ + source=NONE`: software reset bypassed the tagged/persisted paths, or FDS was
  unavailable/purged during the reset path.

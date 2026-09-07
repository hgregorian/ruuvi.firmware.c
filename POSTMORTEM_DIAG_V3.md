# Post-mortem diagnostics v3

This revision adds an append-only raw-flash reset journal in the first 4 KiB page of `.storage_flash`.
That page is outside the actual FDS data range and `ri_flash_purge()` now preserves it.

Purpose: retain evidence from the *first* reset even if the following boot hits `rt_flash_init()` failure, purges FDS, and software-resets again.

`DIAG?` sends the existing 20-byte `PM` v2 packet followed by up to four `JR` v1 packets, newest first.

JR v1 (20 bytes):
`4A 52 01 II SS FF QQ QQ QQ QQ EE EE EE EE LL LL HH HH 00 00`

- `II`: reverse index, 0 newest
- `SS`: reset source
- `FF`: bit0 fatal details valid
- `QQ`: journal sequence
- `EE`: rd_status_t
- `LL`: source line
- `HH`: filename hash

Reset sources add `5 = FLASH_INIT_FAILURE`.

`CLEAR` deletes the FDS post-mortem record and erases the raw journal page.

The journal never uses GPREGRET/GPREGRET2 and does not alter RESETREAS.

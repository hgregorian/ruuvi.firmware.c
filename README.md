# Connection-parameters teardown fix

Baseline: `ruuvi.firmware.c-diag-v3.zip`

This test changes exactly one production source file:

`src/ruuvi.drivers.c/src/nrf5_sdk15_platform/communication/ruuvi_nrf5_sdk15_communication_ble_gatt.c`

`ri_gatt_disconnect()` now calls `ble_conn_params_stop()` before the intentional GAP disconnect. A genuine stop failure is propagated; otherwise the existing `sd_ble_gap_disconnect()` path is unchanged.

The post-mortem v3 diagnostics and 1-second heartbeat are left unchanged.

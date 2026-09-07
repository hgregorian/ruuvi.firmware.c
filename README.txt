Ruuvi graceful-disconnect experiment
====================================

Purpose
-------
Test one hypothesis only: the intermittent remote config/DFU transition is caused
by cycling the BLE/SoftDevice stack underneath an active connection.

Behavioral change
-----------------
For a successful BLE password/unlock command:

OLD:
  password accepted
  -> app_comms_configure_next_enable()
  -> immediately uninitialize/reinitialize BLE/GATT while the link is live

EXPERIMENT:
  password accepted
  -> request a normal GAP disconnect
  -> wait for BLE_GAP_EVT_DISCONNECTED
  -> only then uninitialize/reinitialize BLE/GATT in config mode

The existing public app_comms_configure_next_enable() behavior is intentionally
left unchanged so this experiment is narrowly scoped to the BLE password path.

Files
-----
src/app_comms.c
src/app_comms.h                                      (unchanged, included for completeness)
src/ruuvi.drivers.c/src/tasks/ruuvi_task_gatt.c
src/ruuvi.drivers.c/src/tasks/ruuvi_task_gatt.h
src/ruuvi.drivers.c/src/interfaces/communication/ruuvi_interface_communication_ble_gatt.h
src/ruuvi.drivers.c/src/nrf5_sdk15_platform/communication/ruuvi_nrf5_sdk15_communication_ble_gatt.c

Notes
-----
- No sleeps or retry loops were added.
- m_conn_handle is NOT cleared when the disconnect is requested. It is cleared
  by the existing BLE_GAP_EVT_DISCONNECTED handler.
- Your app_comms_bleadv_interval_set() addition is preserved.
- Your 1-second heartbeat configuration is not changed by these files.

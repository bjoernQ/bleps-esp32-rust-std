# Using bleps BLE stack on ESP32-Rust-Std

This is an example to use [bleps](https://github.com/bjoernQ/bleps) on ESP32 Rust-STD.

There is one example for sync and one for async.

All it does is connecting ESP32's VHCI to the stack.

In this example the TWDT is disabled (for simplicity) and the Bluetooth is enabled in sdkconfig.

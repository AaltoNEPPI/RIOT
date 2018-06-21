# Overview

This package provides necessary Makefiles and glue code to use the Nordic S132
SoftDevice as Bluetooth BLE transport for RIOT's 6lowpan stack.

This version is based on nRF5 SDK version 15.0.0 a53641a

# Usage

"gnrc_netdev_default" has a dependency to "nordic_softdevice_ble", so all
examples automatically download the SDK and compile / link / flash all needed
code.

If you want to manually set up included modules, add "USEPKG +=
nordic_softdevice_ble" to your application's Makefile.

See README-BLE-6LoWPAN.md for instructions on how to set up 6lowpan over BLE on
Linux.

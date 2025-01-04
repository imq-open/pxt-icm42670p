
# pxt-icm42670p

BBC Micro:bit MakeCode extension for TDK ICM-42670-P motion sensors.

Supports Micro:bit V2 only.

## Hardware Configurations

This extension occupies the I2C bus (edge pins P19 and P20).

There must be one digital pin allocated for the INT1 signal of the sensor (P16 by default).


## Use as Extension

This repository can be added as an **extension** in MakeCode.

* open [https://makecode.microbit.org/](https://makecode.microbit.org/)
* click on **New Project**
* click on **Extensions** under the gearwheel menu
* search for **https://github.com/imq-open/pxt-icm42670p** and import

## API Usage

The device ID for event source is `EventBusSource.ICM42670P_DEVICE_ID` (74). Defined event IDs are: 

- `EventBusValue.ICM42670P_EVT_DATA_UPDATED` (1): fired when sensor data updated and are available for reading


Examine the [main.ts](./main.ts) example to learn the most commonly used APIs. 


## License

MIT.

See also the [Third-Party Notices](./THIRD_PARTY_NOTICES.md).

## Metadata (used for search, rendering)

* for PXT/microbit
(Micro:bit V2 only)

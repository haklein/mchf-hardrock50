# mcHF to Hardrock 50 Frequency Bridge

ESP32-S3 firmware that reads the current frequency from an mcHF transceiver via USB (FT-817 CAT protocol) and forwards it to a Hardrock 50 amplifier via UART (Elecraft K3 FA command).

## Hardware

- ESP32-S3-DevKitC-1
- mcHF transceiver connected to the USB-OTG port
- Hardrock 50 connected to GPIO 17 (TX) → ACC jack pin 2 (serial in), plus common GND (pin 5)

## Hardrock 50 Setup

The HR50 ACC serial port baud rate must be set to **19200** to match the firmware default.

## Building

```
pio run
pio run -t upload
pio device monitor
```

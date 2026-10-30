# Rocket Tracker Receiver

Ground-station firmware for a LoRa rocket tracker. It receives position packets
from the matching transmitter and works out where the rocket is relative to you.
It then sends the distance and compass bearing to an Android phone over Bluetooth.

## Hardware

| Part | Notes |
|------|-------|
| Arduino MKR WAN 1310 | Point-to-point LoRa (not LoRaWAN) |
| Arduino MKR GPS Shield | Stacked on the MKR headers (uses `Serial1`) |
| Bluetooth serial module | HC-05 / HC-06 (Classic) or HM-10 (BLE), 3.3 V logic |
| LoRa antenna | Matched to the frequency in `TrackerPacket.h` |

### Bluetooth wiring

The GPS shield uses `Serial1`, so the Bluetooth module goes on a second
hardware UART (SERCOM3) on D0/D1:

| MKR pin | Module pin |
|---------|------------|
| D0 (TX) | RXD |
| D1 (RX) | TXD |
| GND     | GND |
| VCC     | VCC (check your module's supply range) |

To connect the GPS with the I2C/ESLOV cable instead, set `GPS_MODE` to
`GPS_MODE_I2C` in `src/main.cpp`.

## Building and flashing

This is a [PlatformIO](https://platformio.org/) project. Libraries
(`sandeepmistry/LoRa`, `arduino-libraries/Arduino_MKRGPS`) are fetched
automatically.

```sh
pio run -t upload
pio device monitor        # 115200 baud
```

## Using it with a phone

1. Install **Serial Bluetooth Terminal** (or any Bluetooth serial terminal) on Android.
2. Pair the module in Android Bluetooth settings (PIN is usually `1234` or `0000`).
3. Connect to the module in the app. Use the Bluetooth LE device list if you
   have an HM-10.
4. Each packet received from the rocket prints:

```
RX #42 RSSI=-87 SNR=7.5 lost=0/43
  Rocket: 40.1234567,-105.1234567 alt=5612ft sats=9 age=0s
  DIST 1843 ft  BRG 47 deg NE  ALT +312 ft
```

- **DIST**: straight-line distance across the ground to the rocket, in feet.
- **BRG**: direction to the rocket in degrees from **true north**, plus a
  16-point compass direction. A magnetic compass will differ by your local
  magnetic declination.
- **ALT**: how far above (+) or below (−) the receiver the rocket is, in feet.
- **lost**: packets missed out of all those expected, worked out from gaps in
  the sequence number.

When the rocket or the receiver doesn't have a GPS fix yet, the output says so.
USB serial also shows a Google Maps link and the receiver's own position.

## Configuration

- **Radio settings**: frequency, spreading factor, bandwidth and sync word are in
  `include/TrackerPacket.h`. They must match the transmitter.
- **Bluetooth baud rate**: set by `BT_BAUD` in `src/main.cpp` (default 9600, the
  factory default for most modules).

## Packet format

The transmitter sends a 24-byte binary `TrackerPacket` (defined in
`include/TrackerPacket.h`) about once a second. If you change the struct, change
it on both the transmitter and receiver, and bump `PACKET_VERSION`.

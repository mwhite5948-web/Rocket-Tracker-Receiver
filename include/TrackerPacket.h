#pragma once

#include <stdint.h>

// Radio settings -- must match on transmitter and receiver.
// 915E6 for North America/Australia, 868E6 for Europe, 433E6 for Asia.
#define LORA_FREQUENCY        915E6
#define LORA_SPREADING_FACTOR 9       // 7 (fast, short range) .. 12 (slow, long range)
#define LORA_BANDWIDTH        125E3
#define LORA_CODING_RATE      5       // 4/5
#define LORA_SYNC_WORD        0x52    // private network ID; ignores other LoRa traffic
#define LORA_TX_POWER_DBM     17      // 2..20 dBm on PA_BOOST

#define PACKET_MAGIC   0x52  // 'R'
#define PACKET_VERSION 1

#define FIX_AGE_NEVER 0xFF

// Fixed-point binary packet (24 bytes) to keep air time short.
struct __attribute__((packed)) TrackerPacket {
  uint8_t  magic;
  uint8_t  version;
  uint16_t seq;          // increments every packet; gaps = lost packets
  int32_t  latE7;        // latitude  * 1e7
  int32_t  lonE7;        // longitude * 1e7
  int32_t  altCm;        // altitude above MSL, centimetres
  uint16_t speedDkmh;    // ground speed, 0.1 km/h
  uint8_t  satellites;
  uint8_t  fixAgeS;      // seconds since last fix, FIX_AGE_NEVER if no fix yet
  uint32_t gpsTime;      // Unix epoch seconds from GPS, 0 if unknown
};

static_assert(sizeof(TrackerPacket) == 24, "TrackerPacket layout changed");

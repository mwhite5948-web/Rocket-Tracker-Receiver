// Rocket Tracker Receiver
// MKR WAN 1310 + MKR GPS Shield -> receives TrackerPacket over LoRa, computes
// distance and bearing from the receiver to the rocket, and reports it over
// USB serial and a Bluetooth serial module (HC-05/HC-06/HM-10) to a phone.

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <Arduino_MKRGPS.h>
#include "wiring_private.h"  // pinPeripheral()
#include "TrackerPacket.h"

// GPS_MODE_SHIELD: shield stacked on the MKR headers (uses Serial1).
// GPS_MODE_I2C:    shield connected with the I2C/ESLOV cable.
#define GPS_MODE GPS_MODE_SHIELD

// Bluetooth module on an extra hardware UART (SERCOM3), since Serial1 is
// taken by the GPS shield.  Wiring: D0 (TX) -> module RXD, D1 (RX) <- module TXD.
// Module VCC/GND per its datasheet; logic must be 3.3 V.
#define BT_TX_PIN 0   // PA22, SERCOM3/PAD[0]
#define BT_RX_PIN 1   // PA23, SERCOM3/PAD[1]
#define BT_BAUD   9600  // HC-05/HC-06/HM-10 factory default

Uart BTSerial(&sercom3, BT_RX_PIN, BT_TX_PIN, SERCOM_RX_PAD_1, UART_TX_PAD_0);
void SERCOM3_Handler() { BTSerial.IrqHandler(); }

const double EARTH_RADIUS_FT = 20902231.0;  // mean radius, 6371 km
const double M_TO_FT = 3.28084;

// Periodic status line so the phone sees the link is alive (and gets the
// receiver's own position) even when no rocket packets are arriving.
const unsigned long STATUS_INTERVAL_MS = 2000;
unsigned long lastStatusMs = 0;

int32_t lastSeq = -1;
uint32_t received = 0;
uint32_t lost = 0;

// Receiver's own position from its GPS.
bool haveLocalFix = false;
double localLat = 0, localLon = 0;
float localAltM = 0;
int localSats = 0;

// Great-circle distance (haversine), feet.
double distanceFt(double lat1, double lon1, double lat2, double lon2) {
  double p1 = radians(lat1), p2 = radians(lat2);
  double dp = radians(lat2 - lat1), dl = radians(lon2 - lon1);
  double a = sin(dp / 2) * sin(dp / 2) + cos(p1) * cos(p2) * sin(dl / 2) * sin(dl / 2);
  return EARTH_RADIUS_FT * 2 * atan2(sqrt(a), sqrt(1 - a));
}

// Initial bearing from point 1 to point 2, degrees true (0..360).
double bearingDeg(double lat1, double lon1, double lat2, double lon2) {
  double p1 = radians(lat1), p2 = radians(lat2), dl = radians(lon2 - lon1);
  double y = sin(dl) * cos(p2);
  double x = cos(p1) * sin(p2) - sin(p1) * cos(p2) * cos(dl);
  double b = degrees(atan2(y, x));
  return b < 0 ? b + 360.0 : b;
}

const char *compassPoint(double deg) {
  static const char *points[] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
                                 "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};
  return points[(int)((deg + 11.25) / 22.5) % 16];
}

// Print the same text to USB serial and Bluetooth.
template <typename T> void out(T v)            { Serial.print(v); BTSerial.print(v); }
template <typename T> void out(T v, int prec)  { Serial.print(v, prec); BTSerial.print(v, prec); }
void outln()                                   { Serial.print("\r\n"); BTSerial.print("\r\n"); }

void setup() {
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && millis() - start < 5000) {}

  pinMode(LED_BUILTIN, OUTPUT);

  BTSerial.begin(BT_BAUD);
  pinPeripheral(BT_TX_PIN, PIO_SERCOM);
  pinPeripheral(BT_RX_PIN, PIO_SERCOM);

  if (!GPS.begin(GPS_MODE)) {
    out("GPS init failed -- check shield connection"); outln();
    while (true) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(100); }
  }

  if (!LoRa.begin(LORA_FREQUENCY)) {
    out("LoRa init failed"); outln();
    while (true) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(250); }
  }
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSignalBandwidth(LORA_BANDWIDTH);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.enableCrc();

  out("Receiver ready"); outln();
}

void loop() {
  // GPS.available() must be called often so the NMEA stream gets parsed.
  if (GPS.available()) {
    localLat = GPS.latitude();
    localLon = GPS.longitude();
    localAltM = GPS.altitude();
    localSats = GPS.satellites();
    haveLocalFix = true;
  }

  if (millis() - lastStatusMs >= STATUS_INTERVAL_MS) {
    lastStatusMs = millis();
    if (haveLocalFix) {
      out("  Receiver: "); out(localLat, 7); out(","); out(localLon, 7);
      out(" sats=");       out(localSats);
    } else {
      out("  Receiver: no GPS fix yet");
    }
    outln();
  }

  int size = LoRa.parsePacket();
  if (size == 0) return;

  if (size != sizeof(TrackerPacket)) {
    Serial.print("Ignoring packet of size ");
    Serial.println(size);
    while (LoRa.available()) LoRa.read();
    return;
  }

  TrackerPacket pkt;
  LoRa.readBytes((uint8_t *)&pkt, sizeof(pkt));
  int rssi = LoRa.packetRssi();
  float snr = LoRa.packetSnr();

  if (pkt.magic != PACKET_MAGIC || pkt.version != PACKET_VERSION) {
    Serial.println("Ignoring packet with bad magic/version");
    return;
  }

  digitalWrite(LED_BUILTIN, HIGH);

  // Track lost packets via sequence gaps (handles uint16 wraparound).
  if (lastSeq >= 0) {
    uint16_t gap = (uint16_t)(pkt.seq - (uint16_t)lastSeq);
    if (gap > 1) lost += gap - 1;
  }
  lastSeq = pkt.seq;
  received++;

  out("RX #");    out(pkt.seq);
  out(" RSSI=");  out(rssi);
  out(" SNR=");   out(snr, 1);
  out(" lost=");  out(lost);
  out("/");       out(received + lost);
  outln();

  if (pkt.fixAgeS == FIX_AGE_NEVER) {
    out("  Rocket: no GPS fix yet"); outln();
  } else {
    double lat = pkt.latE7 / 1e7;
    double lon = pkt.lonE7 / 1e7;
    out("  Rocket: ");  out(lat, 7); out(","); out(lon, 7);
    out(" alt=");       out(pkt.altCm / 100.0 * M_TO_FT, 0);
    out("ft sats=");    out(pkt.satellites);
    out(" age=");       out(pkt.fixAgeS); out("s");
    outln();

    if (haveLocalFix) {
      double dist = distanceFt(localLat, localLon, lat, lon);
      double brg = bearingDeg(localLat, localLon, lat, lon);
      double altDiffFt = (pkt.altCm / 100.0 - localAltM) * M_TO_FT;
      out("  DIST "); out(dist, 0); out(" ft");
      out("  BRG ");  out(brg, 0);  out(" deg "); out(compassPoint(brg));
      out("  ALT ");  if (altDiffFt >= 0) out("+"); out(altDiffFt, 0); out(" ft");
      outln();
    } else {
      out("  Receiver: no GPS fix yet"); outln();
    }

    Serial.print("  https://maps.google.com/?q=");
    Serial.print(lat, 7); Serial.print(","); Serial.println(lon, 7);
  }

  if (haveLocalFix) {
    Serial.print("  Receiver: "); Serial.print(localLat, 7);
    Serial.print(",");            Serial.print(localLon, 7);
    Serial.print(" sats=");       Serial.println(localSats);
  }

  digitalWrite(LED_BUILTIN, LOW);
}

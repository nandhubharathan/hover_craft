/*
 * receiver_nano.ino — 433 MHz Hovercraft RX Bridge
 * ==================================================
 * Platform  : Arduino Nano (ATmega328P, 16 MHz)
 * Role      : Wireless RX → Wired UART bridge to Pico
 *
 * Wiring:
 *   433 MHz RX DATA → D12   (RadioHead rxPin)
 *   D11              → 1kΩ → Pico GP9  (SoftwareSerial TX)
 *   Pico GP9         → 2kΩ → GND       (voltage divider: 5V→3.3V)
 *   Nano GND         → Pico GND
 *
 * Protocol to Pico (57600 bps, 8 bytes per packet):
 *   [0xAA][0x55][thrust][steer][lift][ai][brake][checksum]
 *    sync0  sync1   1B     1B    1B   1B    1B       1B
 *
 *   thrust   : uint8  0–255
 *   steer    : int8   cast to uint8 for transmission (-100…+100)
 *   lift     : uint8  0 or 1
 *   ai       : uint8  0 or 1
 *   brake    : uint8  0 or 1
 *   checksum : thrust ^ steer ^ lift ^ ai ^ brake
 *
 * RadioHead config:
 *   RH_ASK(2000, rxPin=12, txPin=11, pttPin=10)
 *   txPin=11 is unused (RX-only node) but must be specified.
 *   pttPin=10 is unused, set HIGH to disable PTT.
 */

#include <RH_ASK.h>
#include <SPI.h>
#include <SoftwareSerial.h>

// ── RadioHead driver ──────────────────────────────────────────────────
// speed=2000, rxPin=12, txPin=11 (unused), pttPin=10 (unused)
RH_ASK rf_driver(2000, 12, 11, 10);

// ── SoftwareSerial to Pico ────────────────────────────────────────────
// RX pin=10 unused (we never receive from Pico), TX pin=11
SoftwareSerial picoSerial(10, 11);  // (rxPin, txPin)

// ── Packet struct (matches Nano TX HoverPacket exactly) ───────────────
#pragma pack(push, 1)
typedef struct {
    uint8_t thrust;
    int8_t  steer;
    uint8_t liftToggle;
    uint8_t aiMode;
    uint8_t brakeMode;
    uint8_t checksum;
} HoverPacket;
#pragma pack(pop)

// ── Frame constants ───────────────────────────────────────────────────
#define SYNC0  0xAA
#define SYNC1  0x55
#define FRAME_LEN 8   // SYNC0 + SYNC1 + 5 payload + 1 checksum

// ── Stats ─────────────────────────────────────────────────────────────
uint32_t rxCount   = 0;
uint32_t badCount  = 0;
uint32_t lastPrint = 0;


// ══════════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(9600);
    picoSerial.begin(57600);

    Serial.println(F("========================================"));
    Serial.println(F("  HOVERCRAFT RX BRIDGE v1.0"));
    Serial.println(F("  433MHz D12 → RadioHead → D11 → Pico"));
    Serial.println(F("  SoftwareSerial @ 57600 bps"));
    Serial.println(F("========================================"));

    if (!rf_driver.init()) {
        Serial.println(F("[ERROR] RadioHead init FAILED — check D12"));
        pinMode(LED_BUILTIN, OUTPUT);
        while (1) {
            digitalWrite(LED_BUILTIN, HIGH); delay(200);
            digitalWrite(LED_BUILTIN, LOW);  delay(200);
        }
    }

    Serial.println(F("[OK] RadioHead ready on D12 — waiting for TX..."));
}


// ══════════════════════════════════════════════════════════════════════
void loop() {
    uint8_t  buf[RH_ASK_MAX_MESSAGE_LEN];
    uint8_t  bufLen = sizeof(buf);

    // ── Non-blocking receive ─────────────────────────────────────────
    if (!rf_driver.recv(buf, &bufLen)) {
        // Nothing received this poll — print stats every 2 s
        uint32_t now = millis();
        if (now - lastPrint >= 2000) {
            lastPrint = now;
            Serial.print(F("[BRIDGE] rx="));
            Serial.print(rxCount);
            Serial.print(F("  bad="));
            Serial.println(badCount);
        }
        return;
    }

    // ── Size check ───────────────────────────────────────────────────
    if (bufLen != sizeof(HoverPacket)) {
        badCount++;
        Serial.print(F("[WARN] unexpected packet length: "));
        Serial.println(bufLen);
        return;
    }

    // ── Cast and verify checksum ──────────────────────────────────────
    HoverPacket* pkt = (HoverPacket*)buf;
    uint8_t expected = pkt->thrust
                     ^ (uint8_t)pkt->steer
                     ^ pkt->liftToggle
                     ^ pkt->aiMode
                     ^ pkt->brakeMode;

    if (pkt->checksum != expected) {
        badCount++;
        Serial.println(F("[WARN] checksum mismatch — dropped"));
        return;
    }

    rxCount++;

    // ── Forward to Pico ───────────────────────────────────────────────
    // Frame: [0xAA][0x55][thrust][steer_u8][lift][ai][brake][csum]
    // steer is int8 but transmitted as uint8 (Pico re-interprets as signed)
    uint8_t frame[FRAME_LEN] = {
        SYNC0,
        SYNC1,
        pkt->thrust,
        (uint8_t)pkt->steer,   // int8 → uint8 bit-cast; Pico unpacks as int8
        pkt->liftToggle,
        pkt->aiMode,
        pkt->brakeMode,
        pkt->checksum           // reuse the already-verified RF checksum
    };

    picoSerial.write(frame, FRAME_LEN);

    // ── Debug print every 20 packets ─────────────────────────────────
    if (rxCount % 20 == 1) {
        Serial.print(F("[RX #"));
        Serial.print(rxCount);
        Serial.print(F("]  THR="));
        Serial.print(pkt->thrust);
        Serial.print(F("  STR="));
        Serial.print((int)pkt->steer);
        Serial.print(F("  LIFT="));
        Serial.print(pkt->liftToggle);
        Serial.print(F("  AI="));
        Serial.print(pkt->aiMode);
        Serial.print(F("  BRK="));
        Serial.println(pkt->brakeMode);
    }
}

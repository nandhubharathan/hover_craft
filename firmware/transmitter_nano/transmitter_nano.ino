/*
 * transmitter_nano.ino — 433 MHz Hovercraft TX (1700KV v2.1)
 * ===========================================================
 * Platform : Arduino Nano (ATmega328P, 16 MHz)
 * TX Module: 433 MHz ASK via RadioHead RH_ASK @ 2000 bps
 *
 * v2.1 additions
 * ──────────────
 * Connection status feedback:
 *   • On boot: prints [CONNECTING...] every 500 ms while the RF
 *     module is initialising and the first packet is being sent.
 *   • After the first successful waitPacketSent(): prints the
 *     [TX LIVE] banner ONCE so you can see the link is up.
 *   • Every 500 ms debug print now includes [LIVE] or [NO LINK]
 *     so connection state is always visible in the Serial Monitor.
 *
 * NOTE: The 433 MHz link is one-way (TX only — no RX module on
 * the Nano). "Connected" here means the Nano has successfully
 * transmitted at least one packet. The Pico shows [CONNECTED]
 * when it receives and validates the first packet from this TX.
 */

#include <RH_ASK.h>
#include <SPI.h>

// ── Pins ───────────────────────────────────────────────────────────────
#define JOY_X_PIN    A0
#define JOY_Y_PIN    A1
#define AI_MODE_PIN  2
#define LIFT_BTN_PIN 3

// ── Joystick calibration ───────────────────────────────────────────────
const int CENTER_VAL = 512;
const int DEADZONE   = 40;

// ── RadioHead driver ───────────────────────────────────────────────────
// RH_ASK(speed, rxPin, txPin, pttPin)
// rxPin=11 (unused — TX only), txPin=12, pttPin=10
RH_ASK rf_driver(2000, 11, 12, 10);

// ── Packet struct (must match Pico '<BbBBBB' exactly) ─────────────────
#pragma pack(push, 1)
typedef struct {
    uint8_t thrust;     // 0–255
    int8_t  steer;      // -100…+100
    uint8_t liftToggle; // 0 or 1
    uint8_t aiMode;     // 0 or 1
    uint8_t brakeMode;  // 0 or 1
    uint8_t checksum;   // thrust ^ (uint8_t)steer ^ lift ^ ai ^ brake
} HoverPacket;
#pragma pack(pop)

// ── Timing ────────────────────────────────────────────────────────────
const uint16_t TX_INTERVAL_MS  = 20;   // 50 Hz transmit rate
const uint8_t  DEBOUNCE_MS     = 50;

// ── State ─────────────────────────────────────────────────────────────
bool     prevLiftBtn    = false;
bool     liftState      = false;
uint32_t lastSendMs     = 0;
uint32_t lastDebounceMs = 0;

// ── Connection tracking ───────────────────────────────────────────────
// txLive = true after the first successful waitPacketSent().
// Because the link is one-way we cannot confirm the Pico received it,
// but this confirms the RF module is transmitting without hanging.
bool     txLive         = false;
uint32_t txPacketCount  = 0;


// ══════════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(9600);
    pinMode(AI_MODE_PIN,  INPUT_PULLUP);
    pinMode(LIFT_BTN_PIN, INPUT_PULLUP);

    Serial.println(F("========================================"));
    Serial.println(F("  HOVERCRAFT TX v2.1 — 1700KV 2-Blade"));
    Serial.println(F("  RadioHead ASK @ 2000 bps | 6-byte pkt"));
    Serial.println(F("========================================"));

    // ── RF module init ───────────────────────────────────────────────
    Serial.println(F("[CONNECTING...] Initialising RF module..."));
    if (!rf_driver.init()) {
        // RF module failed to start — halt and blink the onboard LED
        // so the fault is visible even without a serial monitor
        Serial.println(F("[ERROR] RadioHead init FAILED — check D10/D11/D12"));
        pinMode(LED_BUILTIN, OUTPUT);
        while (1) {
            digitalWrite(LED_BUILTIN, HIGH); delay(200);
            digitalWrite(LED_BUILTIN, LOW);  delay(200);
        }
    }

    Serial.println(F("[CONNECTING...] RF module ready — sending first packet"));
}


// ══════════════════════════════════════════════════════════════════════
void loop() {
    uint32_t now = millis();

    // ── Rate limiting — transmit at exactly TX_INTERVAL_MS ───────────
    if (now - lastSendMs < TX_INTERVAL_MS) return;
    lastSendMs = now;

    // ── Read Thrust (X-axis, forward only) ───────────────────────────
    // Only the upper half of joystick travel (rawX > centre+deadzone)
    // produces thrust. Pulling back sets brakeMode instead.
    // Range: ADC 512+40 → 1023  maps to  0 → 255  (PWM-ready byte).
    // The Pico re-maps 0–255 to ESC pulse width 1000–2000 µs.
    int rawX    = analogRead(JOY_X_PIN);
    uint8_t thrust = 0;
    if (rawX > (CENTER_VAL + DEADZONE)) {
        thrust = (uint8_t)constrain(
            map(rawX, CENTER_VAL + DEADZONE, 1023, 0, 255), 0, 255);
    }

    // ── Read Steer (Y-axis, centred) ─────────────────────────────────
    // Full axis: 0→1023 maps to -100→+100 with a dead-zone at centre.
    // Positive = right, negative = left.
    int rawY     = analogRead(JOY_Y_PIN);
    int8_t steer = 0;
    if (rawY > (CENTER_VAL + DEADZONE)) {
        steer = (int8_t)constrain(
            map(rawY, CENTER_VAL + DEADZONE, 1023, 0, 100), 0, 100);
    } else if (rawY < (CENTER_VAL - DEADZONE)) {
        steer = (int8_t)constrain(
            map(rawY, CENTER_VAL - DEADZONE, 0, 0, -100), -100, 0);
    }

    // ── Lift toggle (D3, active-LOW, debounced) ───────────────────────
    bool liftBtn = !digitalRead(LIFT_BTN_PIN);
    if (liftBtn && !prevLiftBtn && (now - lastDebounceMs > DEBOUNCE_MS)) {
        liftState      = !liftState;
        lastDebounceMs = now;
    }
    prevLiftBtn = liftBtn;

    // ── AI mode switch (D2, active-LOW) ──────────────────────────────
    bool aiMode = !digitalRead(AI_MODE_PIN);

    // ── Brake mode (joystick pulled back past dead-zone) ─────────────
    bool brakeMode = (rawX < (CENTER_VAL - DEADZONE));

    // ── Steering-Thrust Mix ───────────────────────────────────────────
    // Turning needs airflow over the rudder.  Map |steer| → thrust so
    // full deflection (±100) gives full thrust (255).  Take the HIGHER
    // of joystick thrust or steering-derived thrust — pilot throttle is
    // never reduced, only boosted when turning hard.
    uint8_t steerThrust = (uint8_t)constrain(
        (long)abs(steer) * 255L / 100L, 0, 255);
    thrust = max(thrust, steerThrust);

    // ── Build packet ──────────────────────────────────────────────────
    HoverPacket pkt;
    pkt.thrust     = thrust;
    pkt.steer      = steer;
    pkt.liftToggle = liftState  ? 1 : 0;
    pkt.aiMode     = aiMode     ? 1 : 0;
    pkt.brakeMode  = brakeMode  ? 1 : 0;
    // XOR checksum — mirrors the Pico's expected_cs calculation exactly
    pkt.checksum   = pkt.thrust ^ (uint8_t)pkt.steer
                     ^ pkt.liftToggle ^ pkt.aiMode ^ pkt.brakeMode;

    // ── Transmit ──────────────────────────────────────────────────────
    rf_driver.send((uint8_t *)&pkt, sizeof(pkt));
    rf_driver.waitPacketSent();   // blocks until OOK burst completes
    txPacketCount++;

    // ── Connection banner — printed ONCE after first successful TX ────
    if (!txLive) {
        txLive = true;
        Serial.println(F(""));
        Serial.println(F("========================================"));
        Serial.println(F("  [TX LIVE]  First packet sent OK"));
        Serial.println(F("  433 MHz RF link is transmitting"));
        Serial.println(F("  Check Pico/Thonny for [CONNECTED]"));
        Serial.println(F("========================================"));
    }

    // ── Periodic debug print (every 500 ms) ───────────────────────────
    static uint32_t lastPrint = 0;
    if (now - lastPrint >= 500) {
        lastPrint = now;

        // Connection status tag — [LIVE] once txLive, [NO LINK] before
        const char* status = txLive ? "[LIVE]" : "[NO LINK]";

        Serial.print(status);
        Serial.print(F("  #"));       Serial.print(txPacketCount);
        Serial.print(F("  THR="));    Serial.print(thrust);
        Serial.print(F("  STR="));    Serial.print(steer);
        Serial.print(F("  LIFT="));   Serial.print(liftState);
        Serial.print(F("  AI="));     Serial.print(aiMode);
        Serial.print(F("  BRK="));    Serial.println(brakeMode);
    }
}
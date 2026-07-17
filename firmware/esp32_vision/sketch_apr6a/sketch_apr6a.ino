/*
 * sketch_apr6a.ino — ESP32-CAM Data Streamer (v5.4)
 * ====================================================
 * Platform  : ESP32-CAM (AI-Thinker)
 * Role      : Camera frame grab → person bounding-box geometry
 *             + HC-SR04 ultrasonic distance → 4-byte UART packet
 *
 * UART Protocol to Pico (115200 baud, 4 bytes):
 * ──────────────────────────────────────────────
 *   BYTE 0 : 0xD4         — sync header (geometry packet)
 *   BYTE 1 : aspect_x10   — uint8: bounding-box aspect ratio × 10
 *   BYTE 2 : bb_height    — uint8: bounding-box pixel height (0–255)
 *   BYTE 3 : ultra_dist   — uint8: ultrasonic distance in cm (0–255)
 *
 *   No CRC in this 4-byte compact format — sync header + field
 *   range validation on the Pico side provides adequate integrity.
 *
 * Ultrasonic Sensor (HC-SR04):
 *   TRIG_PIN = 13
 *   ECHO_PIN = 15
 *   Non-blocking timer: triggers pulse every 60 ms (well above the
 *   38 ms max echo timeout). Distance clamped to 255 cm on timeout.
 *
 * Camera:
 *   FRAMESIZE_QVGA (320×240) at ~10 FPS.
 *   Uses fb→buf for raw frame data.  Bounding-box detection is a
 *   placeholder — replace with your actual detection model or
 *   colour-blob / motion-delta logic.
 *
 * Pin Map (ESP32-CAM AI-Thinker):
 *   GPIO 13 → HC-SR04 TRIG
 *   GPIO 15 → HC-SR04 ECHO
 *   GPIO 1  → Serial TX (to Pico UART0 RX on GP1)
 *   GPIO 3  → Serial RX (unused by Pico)
 *
 * ⚠  GPIO 13 and 15 are shared with the SD card on AI-Thinker.
 *    SD card must NOT be initialised when using ultrasonic.
 */

#include "esp_camera.h"

// ── Ultrasonic Pins ──────────────────────────────────────────────────
#define TRIG_PIN  13
#define ECHO_PIN  15

// ── Non-blocking ultrasonic timing ───────────────────────────────────
//  State machine:
//    IDLE       → wait for interval timer
//    TRIGGERED  → 10µs pulse sent, waiting for echo
//
//  ULTRA_INTERVAL_MS must be > 38 ms (max echo round-trip for 6.5 m)
//  60 ms gives ~16 Hz measurement rate with zero camera interference.

#define ULTRA_INTERVAL_MS   60     // ms between trigger pulses
#define ULTRA_TIMEOUT_US    23200  // µs — max echo for ~400 cm (practical limit)

enum UltraState { ULTRA_IDLE, ULTRA_TRIGGERED };
static UltraState  ultra_state       = ULTRA_IDLE;
static uint32_t    ultra_last_ms     = 0;   // millis() of last trigger
static uint8_t     ultra_dist_cm     = 255; // last valid distance (255 = no echo)

// ── Camera pin definitions (AI-Thinker ESP32-CAM) ────────────────────
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ── Bounding-box state (from detection) ──────────────────────────────
static uint8_t  bb_aspect_x10 = 0;   // aspect ratio × 10 (0 = no detection)
static uint8_t  bb_height     = 0;   // pixel height (0 = no detection)

// ── Forward declarations ─────────────────────────────────────────────
void     ultra_tick();
void     send_geometry_packet();
void     detect_person(camera_fb_t *fb);
bool     init_camera();


// =====================================================================
//  SETUP
// =====================================================================

void setup() {
    Serial.begin(115200);   // UART to Pico (GP1 RX)

    // ── Ultrasonic GPIO ──────────────────────────────────────────────
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    digitalWrite(TRIG_PIN, LOW);

    // ── Camera init ──────────────────────────────────────────────────
    if (!init_camera()) {
        Serial.println("[ESP32] Camera init FAILED");
        // Continue anyway — ultrasonic still works
    } else {
        Serial.println("[ESP32] Camera ready — QVGA 10 FPS");
    }

    Serial.println("[ESP32] v5.4 — Geometry + Ultrasonic streamer online");
    ultra_last_ms = millis();
}


// =====================================================================
//  MAIN LOOP — ~10 FPS camera + non-blocking ultrasonic
// =====================================================================

void loop() {
    // ── Non-blocking ultrasonic measurement ──────────────────────────
    ultra_tick();

    // ── Camera frame grab + detection ────────────────────────────────
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
        detect_person(fb);
        esp_camera_fb_return(fb);
    }

    // ── Send 4-byte geometry packet ──────────────────────────────────
    send_geometry_packet();
}


// =====================================================================
//  NON-BLOCKING ULTRASONIC
// =====================================================================
//
//  Called every loop() iteration.  Uses a timer to avoid blocking
//  the camera pipeline.  The pulseIn() call has a timeout so it
//  never hangs longer than ULTRA_TIMEOUT_US (~23 ms).
//
//  Measurement cadence: every ULTRA_INTERVAL_MS (60 ms).
//  Between measurements the function returns immediately (< 1 µs).

void ultra_tick() {
    uint32_t now = millis();

    if (ultra_state == ULTRA_IDLE) {
        // ── Wait for interval ────────────────────────────────────────
        if ((now - ultra_last_ms) < ULTRA_INTERVAL_MS) return;

        // ── Send 10 µs trigger pulse ─────────────────────────────────
        digitalWrite(TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN, LOW);

        ultra_state   = ULTRA_TRIGGERED;
        ultra_last_ms = now;
    }

    if (ultra_state == ULTRA_TRIGGERED) {
        // ── Read echo (with timeout) ─────────────────────────────────
        //  pulseIn returns 0 on timeout.
        unsigned long duration_us = pulseIn(ECHO_PIN, HIGH, ULTRA_TIMEOUT_US);

        if (duration_us == 0) {
            // No echo — out of range or sensor error
            ultra_dist_cm = 255;
        } else {
            // Speed of sound ≈ 343 m/s → 29.15 µs/cm (round trip ÷ 2)
            unsigned long dist = duration_us / 58;   // cm
            ultra_dist_cm = (dist > 255) ? 255 : (uint8_t)dist;
        }

        ultra_state = ULTRA_IDLE;
    }
}


// =====================================================================
//  CAMERA INIT (AI-Thinker pinout)
// =====================================================================

bool init_camera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;
    config.pin_d0       = Y2_GPIO_NUM;
    config.pin_d1       = Y3_GPIO_NUM;
    config.pin_d2       = Y4_GPIO_NUM;
    config.pin_d3       = Y5_GPIO_NUM;
    config.pin_d4       = Y6_GPIO_NUM;
    config.pin_d5       = Y7_GPIO_NUM;
    config.pin_d6       = Y8_GPIO_NUM;
    config.pin_d7       = Y9_GPIO_NUM;
    config.pin_xclk     = XCLK_GPIO_NUM;
    config.pin_pclk     = PCLK_GPIO_NUM;
    config.pin_vsync    = VSYNC_GPIO_NUM;
    config.pin_href     = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn     = PWDN_GPIO_NUM;
    config.pin_reset    = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size   = FRAMESIZE_QVGA;   // 320×240
    config.jpeg_quality = 12;
    config.fb_count     = 1;

    esp_err_t err = esp_camera_init(&config);
    return (err == ESP_OK);
}


// =====================================================================
//  PERSON DETECTION (placeholder — replace with your model)
// =====================================================================
//
//  This is a stub detection function.  Replace the body with your
//  actual detection pipeline (TFLite Micro, colour-blob, motion-delta,
//  or any ML model running on the ESP32).
//
//  The function must set:
//    bb_aspect_x10  — bounding-box width/height × 10 (uint8, 0–255)
//    bb_height      — bounding-box pixel height      (uint8, 0–255)
//
//  Set both to 0 when no detection is present.

void detect_person(camera_fb_t *fb) {
    // ── PLACEHOLDER: no detection by default ─────────────────────────
    // Replace this section with your actual detection code.
    //
    // Example (if you have a bounding box from a model):
    //   float aspect = (float)bb_w / (float)bb_h;
    //   bb_aspect_x10 = constrain((uint8_t)(aspect * 10.0f), 0, 255);
    //   bb_height     = constrain(bb_h, 0, 255);
    //
    // For now: output zeros (no human detected)
    bb_aspect_x10 = 0;
    bb_height     = 0;
}


// =====================================================================
//  SEND 4-BYTE GEOMETRY PACKET
// =====================================================================
//
//  Packet format:
//    [0xD4] [aspect_x10] [bb_height] [ultra_dist_cm]
//
//  Sent once per loop() call (~10 Hz, paced by camera frame rate).
//  No CRC — the Pico validates via sync byte + field range checks.

void send_geometry_packet() {
    uint8_t packet[4];
    packet[0] = 0xD4;            // Sync header
    packet[1] = bb_aspect_x10;   // Aspect ratio × 10
    packet[2] = bb_height;       // Bounding-box pixel height
    packet[3] = ultra_dist_cm;   // Ultrasonic distance (0–255 cm)

    Serial.write(packet, 4);
}

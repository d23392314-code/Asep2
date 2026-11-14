/*
  ASEP MOCHI - Teman ngobrol AI (ESP32-S3 CAM)
  -------------------------------------------------
  Layar  : LCD 1.5" ST7789 240x240  (SCL19 SDA20 RES21 DC47 BLK38)
  Mic    : INMP441   (WS1 SCK2 SD42, L/R=GND)
  Speaker: MAX98357A (LRC41 BCLK40 DIN39)
  Kamera : OV2640/OV5640 (pin Freenove ESP32-S3-WROOM CAM, ubah di bawah kalau beda)
  Tombol : BOOT (GPIO0)  tekan sebentar = foto soal | tahan 3 detik = reset WiFi

  Library (Library Manager):
    - Adafruit GFX Library
    - Adafruit ST7735 and ST7789 Library
    - ArduinoJson (versi 7.x)
  Board: "ESP32S3 Dev Module"  | PSRAM: "OPI PSRAM" | Flash 16MB (sesuaikan)
         USB CDC On Boot: Disabled  (GPIO19/20 dipakai LCD)
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <vector>
#include "driver/i2s.h"
#include "esp_camera.h"
#include "mbedtls/base64.h"

// ======================= KONFIGURASI =======================
// API OpenRouter (otak + telinga). Sebaiknya nanti key ini diganti/revoke karena sudah pernah dishare.
#define OR_API_KEY   "__OR_KEY__"
#define OR_URL       "https://openrouter.ai/api/v1/chat/completions"
#define OR_MODEL     "openrouter/free"   // model untuk chat & baca foto soal
#define STT_MODEL    "openrouter/free"   // model untuk transkripsi suara (harus support input audio!)

// Text-to-Speech (kosongkan dulu = Asep bisu, tapi teks tetap tampil di layar)
// Default format: OpenAI-compatible /v1/audio/speech dengan output PCM 16-bit mono.
#define TTS_API_KEY  ""
#define TTS_URL      "https://api.openai.com/v1/audio/speech"
#define TTS_MODEL    "tts-1"
#define TTS_VOICE    "nova"
#define TTS_RATE     24000
#define TTS_VOLUME   0.8f

#define WAKE_NAME_HINT "Halo Asep"
#define AP_NAME      "Asep-Setup"
#define AWAKE_MS     20000        // berapa lama Asep standby dengerin setelah dipanggil

// Pin LCD
#define LCD_SCL 19
#define LCD_SDA 20
#define LCD_RES 21
#define LCD_DC  47
#define LCD_BLK 38
// Pin INMP441
#define MIC_WS  1
#define MIC_SCK 2
#define MIC_SD  42
// Pin MAX98357A
#define SPK_LRC  41
#define SPK_BCLK 40
#define SPK_DIN  39
// Tombol
#define PIN_BTN 0
// Pin kamera (Freenove ESP32-S3-WROOM CAM)
#define CAM_XCLK 15
#define CAM_SIOD 4
#define CAM_SIOC 5
#define CAM_Y2 11
#define CAM_Y3 9
#define CAM_Y4 8
#define CAM_Y5 10
#define CAM_Y6 12
#define CAM_Y7 18
#define CAM_Y8 17
#define CAM_Y9 16
#define CAM_VSYNC 6
#define CAM_HREF 7
#define CAM_PCLK 13
#define CAM_VFLIP 0
#define CAM_HMIRROR 0

// ======================= GLOBAL =======================
#define RGB(r, g, b) ((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3))
static const uint16_t C_BG     = RGB(255, 222, 233);
static const uint16_t C_BODY   = RGB(255, 243, 247);
static const uint16_t C_SHADOW = RGB(238, 196, 212);
static const uint16_t C_EYE    = RGB(70, 45, 60);
static const uint16_t C_CHEEK  = RGB(255, 163, 187);
static const uint16_t C_MOUTH  = RGB(130, 55, 75);
static const uint16_t C_TONGUE = RGB(255, 140, 160);
static const uint16_t C_TXTBG  = RGB(28, 26, 40);
static const uint16_t C_LINE   = RGB(255, 160, 190);
static const uint16_t C_PINK   = RGB(255, 150, 185);
static const uint16_t C_CYAN   = RGB(120, 220, 255);
static const uint16_t C_YEL    = RGB(255, 220, 100);
static const uint16_t C_GREEN  = RGB(120, 240, 150);
static const uint16_t C_RED    = RGB(255, 100, 100);
static const uint16_t C_GRAY   = RGB(170, 170, 190);
static const uint16_t C_WHITE  = 0xFFFF;

Adafruit_ST7789 tft(&SPI, -1, LCD_DC, LCD_RES);
GFXcanvas16* fc = nullptr;
SemaphoreHandle_t tftMux;
#define LOCK()   xSemaphoreTake(tftMux, portMAX_DELAY)
#define UNLOCK() xSemaphoreGive(tftMux)

enum FaceState : uint8_t { F_HAPPY, F_LISTEN, F_THINK, F_TALK, F_ERROR, F_LOVE, F_SURPRISE, F_SLEEPY, F_SLEEP };
volatile FaceState faceState = F_HAPPY;
volatile bool forceFace = false;

Preferences prefs;
WebServer server(80);
DNSServer dns;
bool portalActive = false;
bool camOK = false;

#define FRAME 480              // 30 ms @16kHz
#define RING_FR 10
#define SIL_FR 25
#define REC_MAX_SAMPLES 160000 // 10 detik
uint8_t* wavBuf = nullptr;     // 44 byte header + PCM
float noiseFloor = 120;

enum Mode { SLEEPING, AWAKE };
Mode mode = SLEEPING;
uint32_t awakeUntil = 0;

bool btnShort = false, btnLong = false;
volatile uint32_t lastActivity = 0;   // terakhir kali ada aktivitas (buat ekspresi ngantuk)

String hUser[3], hBot[3];
int hCount = 0;

void onOnline();
void showError(const String& msg);
void setStatus(const char* s, uint16_t col);
struct Buf;   // forward declaration (biar prototipe otomatis Arduino tidak error)

// ======================= TOMBOL =======================
void pollButton() {
  static bool down = false, longFired = false;
  static uint32_t t0 = 0;
  bool p = digitalRead(PIN_BTN) == LOW;
  uint32_t now = millis();
  if (p && !down) { down = true; t0 = now; longFired = false; lastActivity = now; }
  else if (p && down && !longFired && now - t0 >= 3000) { longFired = true; btnLong = true; }
  else if (!p && down) { down = false; if (!longFired && now - t0 >= 40) btnShort = true; }
}

// ======================= WAJAH MOCHI (animasi) =======================
static const uint16_t C_CHEEK2 = RGB(255, 125, 160);
static const uint16_t C_HEART  = RGB(255, 80, 130);
static const uint16_t C_TEAR   = RGB(120, 185, 255);
static const uint16_t C_ZZZ    = RGB(120, 105, 195);
static const uint16_t C_WAVE   = RGB(255, 140, 175);

static volatile FaceState overrideState = F_HAPPY;
static volatile uint32_t overrideUntil = 0;
// Paksa ekspresi tertentu selama 'ms' milidetik (misal kaget saat dipanggil, mata hati saat dipuji)
void flashFace(FaceState s, uint32_t ms) { overrideState = s; overrideUntil = millis() + ms; }

static void fillEllipse(int cx, int cy, int rx, int ry, uint16_t c) {
  if (rx < 1) rx = 1;
  if (ry < 1) { fc->drawFastHLine(cx - rx, cy, 2 * rx + 1, c); return; }
  for (int y = -ry; y <= ry; y++) {
    float k = 1.0f - (float)(y * y) / (float)(ry * ry);
    int w = (int)(rx * sqrtf(k < 0 ? 0 : k) + 0.5f);
    fc->drawFastHLine(cx - w, cy + y, 2 * w + 1, c);
  }
}

static void thickDot(int x, int y, int t, uint16_t c) { fc->fillRect(x - t / 2, y - t / 2, t, t, c); }

static void heart(int cx, int cy, float r, uint16_t c) {
  int ri = (int)(r + 0.5f);
  fc->fillCircle(cx - (int)(0.9f * r), cy - (int)(0.3f * r), ri, c);
  fc->fillCircle(cx + (int)(0.9f * r), cy - (int)(0.3f * r), ri, c);
  fc->fillTriangle(cx - (int)(1.85f * r), cy, cx + (int)(1.85f * r), cy, cx, cy + (int)(2.1f * r), c);
}

static void sparkle(int x, int y, int s, uint16_t c) {
  fc->drawFastHLine(x - s, y, 2 * s + 1, c);
  fc->drawFastVLine(x, y - s, 2 * s + 1, c);
  if (s > 2) {
    int d = s / 2;
    fc->drawPixel(x - d, y - d, c); fc->drawPixel(x + d, y - d, c);
    fc->drawPixel(x - d, y + d, c); fc->drawPixel(x + d, y + d, c);
  }
}

// Senyum  (tengah lebih rendah dari ujung)
static void mouthSmile(int cx, int cy, int w, int depth, uint16_t c) {
  for (int dx = -w; dx <= w; dx++) thickDot(cx + dx, cy + depth - (depth * dx * dx) / (w * w), 2, c);
}
// Cemberut (tengah lebih tinggi dari ujung)
static void mouthFrown(int cx, int cy, int w, int depth, uint16_t c) {
  for (int dx = -w; dx <= w; dx++) thickDot(cx + dx, cy + (depth * dx * dx) / (w * w), 2, c);
}
// Mulut kucing "w" yang imut
static void mouthCat(int cx, int cy, uint16_t c) {
  for (int dx = -8; dx <= 8; dx++) {
    float u = (dx < 0) ? (dx + 8) / 8.0f : dx / 8.0f;
    thickDot(cx + dx, cy + (int)(4.0f * sinf(PI * u)), 2, c);
  }
}
// Mulut terbuka (atas rata, bawah melengkung) + lidah
static void mouthOpen(int cx, int cy, int w, int h) {
  if (h < 2) { mouthSmile(cx, cy, w, 3, C_MOUTH); return; }
  for (int y = 0; y <= h; y++) {
    float k = 1.0f - (float)(y * y) / (float)(h * h);
    int hw = (int)(w * sqrtf(k < 0 ? 0 : k) + 0.5f);
    fc->drawFastHLine(cx - hw, cy + y, 2 * hw + 1, C_MOUTH);
    if (h >= 7 && y >= h / 2 && y < h) fc->drawFastHLine(cx - (hw * 7) / 10, cy + y, 2 * ((hw * 7) / 10) + 1, C_TONGUE);
  }
}

static void renderFace(FaceState s, float t, float gx, float gy, float blink, bool giggle, float mopen, float mw) {
  uint32_t now = millis();
  float bs = (s == F_SLEEP) ? 0.9f : (s == F_SLEEPY) ? 1.3f : 2.0f;
  int bob = (int)(sinf(t * bs) * 2.5f);                       // napas
  int bw = 180 + ((s == F_TALK) ? (int)(mopen * 0.3f) : 0);   // badan melar saat ngomong
  int ox = (s == F_LOVE) ? (int)(3.0f * sinf(t * 3.0f)) : 0;  // goyang saat senang
  int jump = (s == F_SURPRISE) ? 6 : 0;                       // lompat kaget

  fc->fillScreen(C_BG);
  if (s != F_SLEEP && s != F_SLEEPY) {   // bintang kecil berkelip di latar
    sparkle(16, 22, 2 + (int)((sinf(t * 2.1f) + 1.0f) * 1.5f), C_WHITE);
    sparkle(224, 30, 2 + (int)((sinf(t * 1.7f + 2.0f) + 1.0f) * 1.5f), C_WHITE);
    sparkle(20, 100, 2 + (int)((sinf(t * 2.5f + 4.0f) + 1.0f) * 1.2f), C_WHITE);
    sparkle(222, 98, 2 + (int)((sinf(t * 1.9f + 1.0f) + 1.0f) * 1.5f), C_WHITE);
  }
  fillEllipse(120 + ox, 124, 72 - bob * 3 - jump * 2, 5, C_SHADOW);   // bayangan
  int top = 10 - bob - jump;
  int h = 110 + bob * 2;
  int x0 = 120 - bw / 2 + ox;
  fc->fillRoundRect(x0 - 2, top - 2, bw + 4, h + 4, min(56, (h + 4) / 2), C_SHADOW);  // outline
  fc->fillRoundRect(x0, top, bw, h, min(54, h / 2), C_BODY);                          // badan mochi
  fillEllipse(72 + ox, top + 22, 14, 6, C_WHITE);                                     // kilau

  // ---------- mata ----------
  int exL = 90 + ox, exR = 150 + ox, ey = 58;
  int egx = (int)gx, egy = (int)gy;
  int px = (int)(gx * 0.35f) + ox, py = (int)(gy * 0.35f);     // paralaks pipi & mulut
  int mx = 120 + px, my = 74 + py;

  if (s == F_LOVE) {
    float r = 6.5f + 1.5f * sinf(t * 8.0f);
    heart(exL, ey, r, C_HEART);
    heart(exR, ey, r, C_HEART);
  } else if (s == F_SLEEP) {
    for (int dx = -7; dx <= 7; dx++) {
      int y = ey + 2 - (dx * dx) / 12;
      thickDot(exL + dx, y, 3, C_EYE);
      thickDot(exR + dx, y, 3, C_EYE);
    }
  } else if (giggle && s == F_HAPPY) {
    for (int dx = -7; dx <= 7; dx++) {
      int y = ey - 3 + (dx * dx) / 10;
      thickDot(exL + dx, y, 3, C_EYE);
      thickDot(exR + dx, y, 3, C_EYE);
    }
  } else {
    int rx = 6, ry = 8;
    if (s == F_LISTEN) { rx = 8; ry = 10; }
    else if (s == F_SURPRISE) { rx = 9; ry = 12; }
    float open = (s == F_SLEEPY) ? blink * 0.45f : blink;
    int ryE = max(1, (int)(ry * open));
    int cxs[2] = {exL, exR};
    for (int i = 0; i < 2; i++) {
      int cx = cxs[i] + egx, cy = ey + egy;
      fillEllipse(cx, cy, rx, ryE, C_EYE);
      if (open > 0.5f) {
        fc->fillCircle(cx - rx / 3, cy - ryE / 3, max(2, rx / 3), C_WHITE);
        if (s == F_LISTEN || s == F_HAPPY) fc->fillCircle(cx + rx / 3, cy + ryE / 3, 1, C_WHITE);
      }
      if (s == F_SLEEPY) fc->drawFastHLine(cx - rx - 1, cy - ryE - 1, 2 * rx + 3, C_EYE);  // kelopak ngantuk
    }
    if (s == F_ERROR) {   // alis sedih
      for (int o = 0; o < 3; o++) {
        fc->drawLine(exL - 10 + egx, ey - 8 - o, exL + 7 + egx, ey - 15 - o, C_EYE);
        fc->drawLine(exR - 7 + egx, ey - 15 - o, exR + 10 + egx, ey - 8 - o, C_EYE);
      }
    }
  }

  // ---------- pipi ----------
  uint16_t cc = (s == F_LOVE || s == F_SURPRISE) ? C_CHEEK2 : C_CHEEK;
  fillEllipse(64 + px, 78 + py, 10, 6, cc);
  fillEllipse(176 + px, 78 + py, 10, 6, cc);
  if (s == F_LOVE) {
    for (int k = 0; k < 3; k++) {
      fc->drawLine(58 + px + k * 5, 75 + py, 61 + px + k * 5, 81 + py, C_CHEEK2);
      fc->drawLine(170 + px + k * 5, 75 + py, 173 + px + k * 5, 81 + py, C_CHEEK2);
    }
  }

  // ---------- mulut ----------
  switch (s) {
    case F_HAPPY:
      if (giggle) mouthOpen(mx, my - 1, 9, 8); else mouthCat(mx, my, C_MOUTH);
      break;
    case F_LISTEN:
      fillEllipse(mx, my + 3, 3, 4, C_MOUTH);
      break;
    case F_THINK:
      for (int dx = -9; dx <= 9; dx++) thickDot(mx + dx, my + 4 + (int)(2.5f * sinf(dx * 0.6f + t * 4.0f)), 2, C_MOUTH);
      break;
    case F_TALK:
      mouthOpen(mx, my - 2, (int)mw, (int)mopen);
      break;
    case F_ERROR: {
      int jit = (now / 70) % 2;   // bibir gemetar
      mouthFrown(mx + jit, my + 4, 8, 6, C_MOUTH);
      break;
    }
    case F_LOVE:
      mouthOpen(mx, my - 1, 9, 9);
      break;
    case F_SURPRISE:
      fillEllipse(mx, my + 3, 5, 7, C_MOUTH);
      break;
    case F_SLEEPY:
    case F_SLEEP:
      fillEllipse(mx, my + 4, 2, 2 + (int)((sinf(t * bs * 1.2f) + 1.0f) * 1.2f), C_MOUTH);
      break;
  }

  // ---------- efek tambahan ----------
  if (s == F_LISTEN) {   // gelombang suara di kiri-kanan = lagi dengerin
    int wv = 1 + (int)((now / 260) % 3);
    for (int i = 1; i <= wv; i++) {
      int r = 5 + i * 7;
      fc->drawCircleHelper(212, 64, r, 0x6, C_WAVE);
      fc->drawCircleHelper(212, 64, r + 1, 0x6, C_WAVE);
      fc->drawCircleHelper(28, 64, r, 0x9, C_WAVE);
      fc->drawCircleHelper(28, 64, r + 1, 0x9, C_WAVE);
    }
  }
  if (s == F_THINK) {    // titik-titik mikir
    int nd = 1 + (int)((now / 450) % 3);
    for (int k = 0; k < nd; k++) fc->fillCircle(162 + k * 14, 24, 4, C_EYE);
  }
  if (s == F_SURPRISE) { // tanda seru
    fc->setTextSize(3);
    fc->setTextColor(C_EYE);
    fc->setCursor(190, 8);
    fc->print('!');
  }
  if (s == F_LOVE) {     // hati melayang
    static const int hxs[3] = {28, 212, 196};
    for (int i = 0; i < 3; i++) {
      float p = fmodf(t * 0.7f + i * 0.33f, 1.0f);
      int hx = hxs[i] + (int)(6.0f * sinf(p * 7.0f + i));
      int hy = 80 - (int)(p * 70.0f);
      if (hy > 2) heart(hx, hy, 2.5f + (1.0f - p) * 1.5f, C_HEART);
    }
  }
  if (s == F_ERROR) {    // air mata menetes
    int ty = ey + 12 + (int)((now / 60) % 22);
    fillEllipse(exL - 4 + egx, ty, 2, 3, C_TEAR);
  }
  if (s == F_SLEEPY || s == F_SLEEP) {   // Zzz melayang
    for (int i = 0; i < 3; i++) {
      float p = fmodf(t * 0.5f + i * 0.34f, 1.0f);
      fc->setTextSize(p < 0.5f ? 1 : 2);
      fc->setTextColor(C_ZZZ);
      fc->setCursor(150 + (int)(p * 40.0f), 44 - (int)(p * 34.0f));
      fc->print('z');
    }
  }
}

static void pushFace() {
  LOCK();
  tft.drawRGBBitmap(0, 0, fc->getBuffer(), 240, 130);
  UNLOCK();
}

static void faceTask(void*) {
  float gx = 0, gy = 0, tx = 0, ty = 0;
  float mopen = 0, mw = 8, tmo = 0, tmw = 8;
  uint32_t nextGaze = 0, nextBlink = millis() + 2500, blinkStart = 0;
  uint32_t giggleUntil = 0, nextGiggle = millis() + 9000, nextMouth = 0;
  bool blinking = false;
  int dbl = 0;
  for (;;) {
    uint32_t t0 = millis();
    float t = t0 / 1000.0f;

    // ekspresi efektif
    FaceState s = faceState;
    if (t0 < overrideUntil) s = overrideState;
    else if (s == F_HAPPY && mode == SLEEPING && !portalActive) {
      uint32_t idle = t0 - lastActivity;
      if (idle > 90000) s = F_SLEEP;          // tidur nyenyak
      else if (idle > 45000) s = F_SLEEPY;    // ngantuk
    }

    // arah lirikan mata
    if (t0 >= nextGaze) {
      nextGaze = t0 + random(1200, 3500);
      switch (s) {
        case F_HAPPY:
          if (random(100) < 35) { tx = 0; ty = 0; } else { tx = random(-9, 10); ty = random(-4, 5); }
          break;
        case F_LISTEN: tx = random(-2, 3); ty = random(-2, 3); nextGaze = t0 + random(800, 1800); break;
        case F_THINK: tx = 8; ty = -6; break;
        case F_ERROR: case F_SLEEPY: case F_SLEEP: tx = 0; ty = 4; break;
        default: tx = random(-2, 3); ty = 0; break;
      }
    }
    if (s == F_THINK) tx = 8.0f + 3.0f * sinf(t * 1.5f);
    gx += (tx - gx) * 0.25f;
    gy += (ty - gy) * 0.25f;

    // kedip (kadang dobel)
    float blink = 1.0f;
    if (!blinking && t0 >= nextBlink) { blinking = true; blinkStart = t0; }
    if (blinking) {
      float p = (t0 - blinkStart) / 170.0f;
      if (p >= 1.0f) {
        blinking = false;
        if (dbl == 0 && random(100) < 18) { dbl = 1; nextBlink = t0 + 120; }
        else { dbl = 0; nextBlink = t0 + random(2200, 5200); }
      } else blink = 1.0f - sinf(p * PI);
    }

    // ketawa kecil (mata ^ ^) sesekali
    if (s == F_HAPPY && t0 >= nextGiggle) { giggleUntil = t0 + 1300; nextGiggle = t0 + random(9000, 18000); }
    bool giggle = (t0 < giggleUntil);

    // gerak mulut saat ngomong
    if (t0 >= nextMouth) {
      nextMouth = t0 + random(70, 130);
      tmo = random(2, 14); tmw = random(6, 11);
      if (random(100) < 15) tmo = 1;
    }
    if (s == F_TALK) { mopen += (tmo - mopen) * 0.5f; mw += (tmw - mw) * 0.5f; }
    else mopen *= 0.6f;

    renderFace(s, t, gx, gy, blink, giggle, mopen, mw);
    pushFace();
    uint32_t el = millis() - t0;
    vTaskDelay(pdMS_TO_TICKS(el < 45 ? 45 - el : 5));
  }
}

// ======================= TEKS & PAGER =======================
#define TXT_Y 134

static String clean(const String& s) {
  String o;
  size_t n = s.length();
  for (size_t i = 0; i < n; i++) {
    unsigned char c = s[i];
    if (c == '\r' || c == '*' || c == '#' || c == '`' || c == '_') continue;
    if (c < 0x80) { o += (char)c; continue; }
    unsigned char c1 = (i + 1 < n) ? s[i + 1] : 0, c2 = (i + 2 < n) ? s[i + 2] : 0;
    if (c == 0xE2 && c1 == 0x80) {
      if (c2 == 0x9C || c2 == 0x9D) o += '"';
      else if (c2 == 0x98 || c2 == 0x99) o += '\'';
      else if (c2 == 0x93 || c2 == 0x94 || c2 == 0xA2) o += '-';
      else if (c2 == 0xA6) o += "...";
      i += 2;
    } else if (c == 0xC3 && c1 == 0x97) { o += 'x'; i += 1; }
    else if (c == 0xC3 && c1 == 0xB7) { o += '/'; i += 1; }
    else {  // lewati karakter multi-byte lain (emoji dll)
      int skip = (c >= 0xF0) ? 3 : (c >= 0xE0) ? 2 : 1;
      i += skip;
    }
  }
  return o;
}

static void wrapText(const String& t, int maxc, std::vector<String>& out) {
  out.clear();
  String line = "", word = "";
  auto flushWord = [&]() {
    if (word.length() == 0) return;
    while ((int)word.length() > maxc) {
      if (line.length()) { out.push_back(line); line = ""; }
      out.push_back(word.substring(0, maxc));
      word = word.substring(maxc);
    }
    if ((int)(line.length() + (line.length() ? 1 : 0) + word.length()) > maxc) { out.push_back(line); line = word; }
    else { if (line.length()) line += ' '; line += word; }
    word = "";
  };
  for (size_t i = 0; i < t.length(); i++) {
    char c = t[i];
    if (c == '\n') { flushWord(); out.push_back(line); line = ""; }
    else if (c == ' ') flushWord();
    else word += c;
  }
  flushWord();
  if (line.length()) out.push_back(line);
}

static std::vector<String> pgLines;
static int pgPage = 0, pgPages = 1, pgLPP = 5, pgCPL = 19, pgSize = 2;
static uint32_t pgNext = 0;
static String pgLabel;
static uint16_t pgCol = C_PINK;

static void renderPage() {
  LOCK();
  tft.fillRect(0, TXT_Y, 240, 240 - TXT_Y, C_TXTBG);
  tft.drawFastHLine(0, TXT_Y, 240, C_LINE);
  tft.setTextSize(1);
  tft.setTextColor(pgCol);
  tft.setCursor(6, TXT_Y + 5);
  tft.print(pgLabel);
  if (pgPages > 1) { tft.print("  ("); tft.print(pgPage + 1); tft.print("/"); tft.print(pgPages); tft.print(")"); }
  tft.setTextSize(pgSize);
  tft.setTextColor(C_WHITE);
  int lh = (pgSize == 2) ? 18 : 10, chars = 0;
  for (int i = 0; i < pgLPP; i++) {
    int idx = pgPage * pgLPP + i;
    if (idx >= (int)pgLines.size()) break;
    tft.setCursor(6, 150 + i * lh);
    tft.print(pgLines[idx]);
    chars += pgLines[idx].length();
  }
  UNLOCK();
  pgNext = millis() + 1800 + chars * (pgSize == 2 ? 30 : 20);
}

static void pagerSet(const char* label, uint16_t col, const String& txt) {
  String t = clean(txt);
  t.trim();
  pgSize = (t.length() <= 95) ? 2 : 1;
  pgCPL = (pgSize == 2) ? 19 : 38;
  pgLPP = (pgSize == 2) ? 5 : 9;
  wrapText(t, pgCPL, pgLines);
  pgPages = max(1, (int)((pgLines.size() + pgLPP - 1) / pgLPP));
  pgPage = 0; pgLabel = label; pgCol = col;
  renderPage();
}

static bool pagerDone() { return pgPage >= pgPages - 1 && millis() >= pgNext; }
static void pagerTick() {
  if (pgPage < pgPages - 1 && millis() >= pgNext) { pgPage++; renderPage(); }
}

void setStatus(const char* s, uint16_t col) {
  LOCK();
  tft.fillRect(0, TXT_Y + 1, 240, 14, C_TXTBG);
  tft.setTextSize(1);
  tft.setTextColor(col);
  tft.setCursor(6, TXT_Y + 5);
  tft.print(s);
  UNLOCK();
}

static void bootStatus(const char* s) {
  tft.fillRect(0, 150, 240, 30, 0x0000);
  tft.setTextSize(1);
  tft.setTextColor(C_GRAY);
  int w = strlen(s) * 6;
  tft.setCursor(max(0, (240 - w) / 2), 156);
  tft.print(s);
}

// ======================= AUDIO =======================
static void micInit() {
  i2s_config_t c; memset(&c, 0, sizeof(c));
  c.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
  c.sample_rate = 16000;
  c.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  c.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
  c.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  c.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  c.dma_buf_count = 8;
  c.dma_buf_len = 256;
  i2s_pin_config_t p; memset(&p, 0, sizeof(p));
  p.mck_io_num = I2S_PIN_NO_CHANGE;
  p.bck_io_num = MIC_SCK;
  p.ws_io_num = MIC_WS;
  p.data_out_num = I2S_PIN_NO_CHANGE;
  p.data_in_num = MIC_SD;
  i2s_driver_install(I2S_NUM_0, &c, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &p);
}

static void spkInit() {
  i2s_config_t c; memset(&c, 0, sizeof(c));
  c.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  c.sample_rate = 16000;
  c.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  c.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  c.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  c.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  c.dma_buf_count = 8;
  c.dma_buf_len = 256;
  c.tx_desc_auto_clear = true;
  i2s_pin_config_t p; memset(&p, 0, sizeof(p));
  p.mck_io_num = I2S_PIN_NO_CHANGE;
  p.bck_io_num = SPK_BCLK;
  p.ws_io_num = SPK_LRC;
  p.data_out_num = SPK_DIN;
  p.data_in_num = I2S_PIN_NO_CHANGE;
  i2s_driver_install(I2S_NUM_1, &c, 0, NULL);
  i2s_set_pin(I2S_NUM_1, &p);
  i2s_zero_dma_buffer(I2S_NUM_1);
}

static void readSamples(int16_t* out, size_t n) {
  static int32_t tmp[256];
  static float dc = 0;
  size_t done = 0;
  while (done < n) {
    size_t m = min((size_t)256, n - done), br = 0;
    i2s_read(I2S_NUM_0, tmp, m * 4, &br, portMAX_DELAY);
    size_t got = br / 4;
    for (size_t i = 0; i < got; i++) {
      float v = (float)(tmp[i] >> 14);
      dc = dc * 0.999f + v * 0.001f;
      v -= dc;
      if (v > 32767) v = 32767; if (v < -32768) v = -32768;
      out[done + i] = (int16_t)v;
    }
    done += got;
  }
}

static float frameRms(const int16_t* f, size_t n) {
  double s = 0;
  for (size_t i = 0; i < n; i++) s += (double)f[i] * f[i];
  return sqrtf((float)(s / n));
}

static void micFlush() { static int16_t fr[FRAME]; for (int i = 0; i < 4; i++) readSamples(fr, FRAME); }

static void calibrateNoise() {
  static int16_t fr[FRAME];
  float sum = 0; int k = 0;
  for (int i = 0; i < 33; i++) { readSamples(fr, FRAME); if (i >= 3) { sum += frameRms(fr, FRAME); k++; } }
  noiseFloor = constrain(sum / k, 20.0f, 1500.0f);
}

static void beepTone(int freq, int ms) {
  static int16_t st[512];
  int total = 16 * ms, done = 0;
  float ph = 0, inc = 2.0f * PI * freq / 16000.0f;
  while (done < total) {
    int n = min(256, total - done);
    for (int i = 0; i < n; i++) {
      float env = 1.0f;
      int pos = done + i;
      if (pos < 160) env = pos / 160.0f;
      if (total - pos < 160) env = (total - pos) / 160.0f;
      int16_t v = (int16_t)(sinf(ph) * 6000 * env);
      ph += inc;
      st[2 * i] = v; st[2 * i + 1] = v;
    }
    size_t bw;
    i2s_write(I2S_NUM_1, st, n * 4, &bw, portMAX_DELAY);
    done += n;
  }
  i2s_zero_dma_buffer(I2S_NUM_1);
}

// Utterance capture dengan VAD sederhana. Hasil PCM ada di wavBuf+44.
static int16_t ring[RING_FR][FRAME];
static size_t captureUtterance(uint32_t waitMs, uint32_t minMs, uint32_t maxMs, bool dropIfLong) {
  int16_t* pcm = (int16_t*)(wavBuf + 44);
  size_t maxS = min((size_t)maxMs * 16, (size_t)REC_MAX_SAMPLES);
  int ringIdx = 0, ringCnt = 0, hit = 0, sil = 0;
  bool rec = false;
  size_t n = 0;
  uint32_t t0 = millis();
  static int16_t fr[FRAME];
  while (true) {
    readSamples(fr, FRAME);
    float r = frameRms(fr, FRAME);
    pollButton();
    if (btnShort || btnLong) return 0;
    if (!rec) {
      memcpy(ring[ringIdx], fr, sizeof(fr));
      ringIdx = (ringIdx + 1) % RING_FR;
      if (ringCnt < RING_FR) ringCnt++;
      float thr = noiseFloor * 3.0f + 250.0f;
      if (r > thr) hit++;
      else { hit = 0; noiseFloor = constrain(noiseFloor * 0.97f + r * 0.03f, 20.0f, 3000.0f); }
      if (hit >= 2) {
        rec = true; sil = 0; lastActivity = millis();
        int start = (ringCnt < RING_FR) ? 0 : ringIdx;
        for (int i = 0; i < ringCnt; i++) { memcpy(pcm + n, ring[(start + i) % RING_FR], FRAME * 2); n += FRAME; }
      } else if (waitMs && millis() - t0 > waitMs) return 0;
    } else {
      memcpy(pcm + n, fr, FRAME * 2);
      n += FRAME;
      float thr = noiseFloor * 2.0f + 180.0f;
      if (r > thr) sil = 0; else sil++;
      if (sil >= SIL_FR) { n -= (SIL_FR - 6) * FRAME; break; }
      if (n + FRAME > maxS) {
        if (dropIfLong) {  // bukan "Halo Asep" (kelamaan) -> buang tanpa kirim API
          int s2 = 0, guard = 0;
          while (s2 < SIL_FR && guard++ < 340) { readSamples(fr, FRAME); if (frameRms(fr, FRAME) > thr) s2 = 0; else s2++; }
          return 0;
        }
        break;
      }
    }
  }
  if (n < (size_t)minMs * 16) return 0;
  return n;
}

static void writeWavHeader(uint8_t* b, uint32_t nSamples) {
  uint32_t ds = nSamples * 2, v32;
  uint16_t v16;
  memcpy(b, "RIFF", 4); v32 = 36 + ds; memcpy(b + 4, &v32, 4);
  memcpy(b + 8, "WAVEfmt ", 8);
  v32 = 16; memcpy(b + 16, &v32, 4);
  v16 = 1; memcpy(b + 20, &v16, 2);
  v16 = 1; memcpy(b + 22, &v16, 2);
  v32 = 16000; memcpy(b + 24, &v32, 4);
  v32 = 32000; memcpy(b + 28, &v32, 4);
  v16 = 2; memcpy(b + 32, &v16, 2);
  v16 = 16; memcpy(b + 34, &v16, 2);
  memcpy(b + 36, "data", 4);
  memcpy(b + 40, &ds, 4);
}

// ======================= KAMERA =======================
static bool initCamera() {
  camera_config_t c; memset(&c, 0, sizeof(c));
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = CAM_Y2; c.pin_d1 = CAM_Y3; c.pin_d2 = CAM_Y4; c.pin_d3 = CAM_Y5;
  c.pin_d4 = CAM_Y6; c.pin_d5 = CAM_Y7; c.pin_d6 = CAM_Y8; c.pin_d7 = CAM_Y9;
  c.pin_xclk = CAM_XCLK; c.pin_pclk = CAM_PCLK; c.pin_vsync = CAM_VSYNC; c.pin_href = CAM_HREF;
  c.pin_sccb_sda = CAM_SIOD; c.pin_sccb_scl = CAM_SIOC;
  c.pin_pwdn = -1; c.pin_reset = -1;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size = FRAMESIZE_XGA;   // 1024x768, cukup tajam buat baca soal
  c.jpeg_quality = 10;
  c.fb_count = 2;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;
  if (esp_camera_init(&c) != ESP_OK) return false;
  sensor_t* s = esp_camera_sensor_get();
  if (s) { s->set_vflip(s, CAM_VFLIP); s->set_hmirror(s, CAM_HMIRROR); }
  return true;
}

static camera_fb_t* grabPhoto() {
  for (int i = 0; i < 3; i++) { camera_fb_t* f = esp_camera_fb_get(); if (f) esp_camera_fb_return(f); delay(60); }
  return esp_camera_fb_get();
}

// ======================= OPENROUTER =======================
struct Buf {
  char* p = nullptr; size_t len = 0, cap = 0;
  bool alloc(size_t c) { cap = c + 16; p = (char*)ps_malloc(cap); if (!p) p = (char*)malloc(cap); len = 0; return p != nullptr; }
  void add(const char* s) { size_t n = strlen(s); if (len + n < cap) { memcpy(p + len, s, n); len += n; } }
  void addEsc(const char* s) {
    for (; *s; s++) {
      if (len + 8 >= cap) break;
      unsigned char c = *s;
      if (c == '"') { p[len++] = '\\'; p[len++] = '"'; }
      else if (c == '\\') { p[len++] = '\\'; p[len++] = '\\'; }
      else if (c == '\n') { p[len++] = '\\'; p[len++] = 'n'; }
      else if (c < 0x20) p[len++] = ' ';
      else p[len++] = c;
    }
  }
  void addB64(const uint8_t* d, size_t n) { size_t ol = 0; mbedtls_base64_encode((unsigned char*)p + len, cap - len - 1, &ol, d, n); len += ol; }
  void release() { if (p) free(p); p = nullptr; len = cap = 0; }
};

static const char* SYS_CHAT =
  "Kamu adalah Asep, robot mochi kecil yang jadi teman ngobrol dekat penggunanya. "
  "Gaya: hangat, lucu, santai, pakai bahasa Indonesia sehari-hari (aku/kamu), peduli dan suportif. "
  "Jawaban dibacakan lewat speaker dan tampil di layar kecil, jadi buat singkat (1-3 kalimat) kecuali diminta detail. "
  "JANGAN pakai emoji, markdown, tanda bintang, atau daftar bernomor. "
  "Input berasal dari transkripsi suara jadi mungkin ada salah dengar; tebak maksud pengguna dengan wajar. "
  "Kalau diminta menerjemahkan, langsung beri terjemahan yang akurat ke bahasa tujuan (sebutkan bahasa tujuannya singkat). "
  "Kalau ditanya pelajaran atau hitungan, jawab benar dan jelas.";

static const char* SYS_PHOTO =
  "Kamu adalah Asep, asisten pintar segala mata pelajaran. Pengguna mengirim FOTO soal atau benda. "
  "Baca isi foto dengan teliti (teks, angka, gambar, grafik), lalu jawab seakurat dan selengkap mungkin dalam bahasa Indonesia. "
  "Untuk soal: tulis JAWABAN AKHIR lebih dulu, lalu langkah pengerjaan singkat dan jelas. Untuk pilihan ganda sebutkan huruf pilihan dan alasannya. "
  "Kalau fotonya buram atau terpotong, katakan bagian mana yang tidak terbaca dan jawab sebisanya. "
  "Gunakan teks polos untuk layar kecil: tanpa markdown, tanpa LaTeX, tanpa emoji. Tulis rumus dengan karakter biasa (x^2, sqrt(9), 3/4).";

static const char* STT_PROMPT =
  "Transkripsikan audio ini persis apa adanya. Bahasa utama Indonesia (boleh campur Inggris). "
  "Balas HANYA teks transkripsinya tanpa tambahan apa pun. Jika tidak ada ucapan yang jelas, balas persis: [kosong]";

static bool parseOR(const String& resp, String& out, String& err) {
  JsonDocument doc;
  if (deserializeJson(doc, resp)) { err = "Respon bukan JSON"; return false; }
  if (!doc["error"].isNull()) {
    const char* m = doc["error"]["message"] | "error tidak diketahui";
    err = String(m);
    return false;
  }
  JsonVariant c = doc["choices"][0]["message"]["content"];
  if (!c.is<const char*>()) { err = "Jawaban kosong"; return false; }
  out = String(c.as<const char*>());
  out.trim();
  if (out.length() == 0) { err = "Jawaban kosong"; return false; }
  return true;
}

static bool orRequest(const char* body, size_t len, String& out, String& err) {
  for (int a = 0; a < 3; a++) {
    if (WiFi.status() != WL_CONNECTED) { err = "WiFi putus"; return false; }
    WiFiClientSecure cl; cl.setInsecure();
    HTTPClient http;
    http.setTimeout(60000);
    if (!http.begin(cl, OR_URL)) { err = "Gagal konek ke server"; return false; }
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Authorization", "Bearer " OR_API_KEY);
    http.addHeader("HTTP-Referer", "https://asep.local");
    http.addHeader("X-Title", "Asep Mochi");
    int code = http.POST((uint8_t*)body, len);
    String resp = (code > 0) ? http.getString() : "";
    http.end();
    Serial.printf("[OR] try %d code %d\n", a + 1, code);
    if (code == 200) {
      if (parseOR(resp, out, err)) return true;
      delay(1200);
      continue;
    }
    String m;
    { JsonDocument d; if (!deserializeJson(d, resp)) { const char* x = d["error"]["message"] | ""; m = x; } }
    err = "HTTP " + String(code) + (m.length() ? (": " + m) : "");
    if (code == 429 || code >= 500 || code < 0) { delay(2000 * (a + 1)); continue; }
    return false;
  }
  if (err.length() == 0) err = "Server sibuk, coba lagi";
  return false;
}

static bool buildMediaBody(Buf& b, const char* model, const char* sys, const char* userText, bool audio,
                           const uint8_t* data, size_t n, int maxTok) {
  size_t cap = ((n + 2) / 3) * 4 + strlen(sys) * 2 + strlen(userText) * 2 + 700;
  if (!b.alloc(cap)) return false;
  b.add("{\"model\":\""); b.add(model);
  b.add("\",\"max_tokens\":"); b.add(String(maxTok).c_str());
  b.add(",\"messages\":[");
  if (sys && *sys) { b.add("{\"role\":\"system\",\"content\":\""); b.addEsc(sys); b.add("\"},"); }
  b.add("{\"role\":\"user\",\"content\":[{\"type\":\"text\",\"text\":\""); b.addEsc(userText); b.add("\"},");
  if (audio) b.add("{\"type\":\"input_audio\",\"input_audio\":{\"data\":\"");
  else b.add("{\"type\":\"image_url\",\"image_url\":{\"url\":\"data:image/jpeg;base64,");
  b.addB64(data, n);
  b.add(audio ? "\",\"format\":\"wav\"}}" : "\"}}");
  b.add("]}]}");
  b.p[b.len] = 0;
  return true;
}

static bool transcribe(size_t nSamples, String& text, String& err) {
  writeWavHeader(wavBuf, nSamples);
  Buf b;
  if (!buildMediaBody(b, STT_MODEL, "", STT_PROMPT, true, wavBuf, 44 + nSamples * 2, 300)) { err = "Memori kurang"; return false; }
  bool ok = orRequest(b.p, b.len, text, err);
  b.release();
  return ok;
}

static bool chatLLM(const String& q, String& reply, String& err) {
  JsonDocument d;
  d["model"] = OR_MODEL;
  d["max_tokens"] = 700;
  JsonArray m = d["messages"].to<JsonArray>();
  JsonObject s = m.add<JsonObject>(); s["role"] = "system"; s["content"] = SYS_CHAT;
  for (int i = 0; i < hCount; i++) {
    JsonObject u = m.add<JsonObject>(); u["role"] = "user"; u["content"] = hUser[i];
    JsonObject a = m.add<JsonObject>(); a["role"] = "assistant"; a["content"] = hBot[i];
  }
  JsonObject u = m.add<JsonObject>(); u["role"] = "user"; u["content"] = q;
  String body; serializeJson(d, body);
  bool ok = orRequest(body.c_str(), body.length(), reply, err);
  if (ok) {
    if (hCount == 3) { for (int i = 0; i < 2; i++) { hUser[i] = hUser[i + 1]; hBot[i] = hBot[i + 1]; } hCount = 2; }
    hUser[hCount] = q.substring(0, 300);
    hBot[hCount] = reply.substring(0, 300);
    hCount++;
  }
  return ok;
}

// ======================= TTS (opsional) =======================
static bool speakText(const String& text) {
  if (strlen(TTS_API_KEY) == 0) return false;   // belum ada key -> Asep bisu
  WiFiClientSecure cl; cl.setInsecure();
  HTTPClient http;
  if (!http.begin(cl, TTS_URL)) return false;
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", "Bearer " TTS_API_KEY);
  JsonDocument d;
  d["model"] = TTS_MODEL; d["voice"] = TTS_VOICE; d["input"] = clean(text); d["response_format"] = "pcm";
  String body; serializeJson(d, body);
  int code = http.POST(body);
  if (code != 200) { Serial.printf("[TTS] HTTP %d\n", code); http.end(); return false; }
  WiFiClient* s = http.getStreamPtr();
  int remaining = http.getSize();   // -1 kalau chunked
  i2s_set_clk(I2S_NUM_1, TTS_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
  static uint8_t buf[1024];
  static int16_t st[1024];
  uint32_t lastData = millis();
  while ((http.connected() || s->available()) && remaining != 0) {
    size_t av = s->available();
    size_t n = min(av, sizeof(buf)) & ~(size_t)1;
    if (n == 0) { if (millis() - lastData > 8000) break; delay(2); pagerTick(); continue; }
    lastData = millis();
    int r = s->readBytes(buf, n);
    if (remaining > 0) remaining -= r;
    int ns = r / 2;
    for (int i = 0; i < ns; i++) {
      int16_t v = (int16_t)(buf[2 * i] | (buf[2 * i + 1] << 8));
      v = (int16_t)(v * TTS_VOLUME);
      st[2 * i] = v; st[2 * i + 1] = v;
    }
    size_t bw;
    i2s_write(I2S_NUM_1, st, ns * 4, &bw, portMAX_DELAY);
    pagerTick();
  }
  delay(150);
  i2s_zero_dma_buffer(I2S_NUM_1);
  i2s_set_clk(I2S_NUM_1, 16000, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
  http.end();
  return true;
}

// ======================= TAMPILAN JAWABAN =======================
static void showReply(const String& t) {
  pagerSet("ASEP", C_PINK, t);
  faceState = F_TALK;
  speakText(t);
  while (!pagerDone()) {
    pagerTick();
    pollButton();
    if (btnShort) { btnShort = false; pgNext = 0; }  // tekan BOOT = lanjut halaman
    delay(30);
  }
  faceState = F_LISTEN;
  micFlush();
}

void showError(const String& msg) {
  Serial.println("[ERR] " + msg);
  faceState = F_ERROR;
  pagerSet("ERROR", C_RED, msg);
  while (!pagerDone()) { pagerTick(); delay(30); }
  faceState = (mode == AWAKE) ? F_LISTEN : F_HAPPY;
}

// ======================= KATA KUNCI =======================
static String norm(const String& s) {
  String o; bool sp = true;
  for (size_t i = 0; i < s.length(); i++) {
    unsigned char c = s[i];
    if (isalnum(c)) { o += (char)tolower(c); sp = false; }
    else if (!sp) { o += ' '; sp = true; }
  }
  o.trim();
  return o;
}
static bool hasW(const String& p, const char* w) { String k = String(" ") + w + " "; return p.indexOf(k) >= 0; }

static bool isWake(const String& low, String& rest) {
  String p = " " + low + " ";
  static const char* greet[] = {"halo", "hallo", "hello", "helo", "alo", "hai", "hei", "hey"};
  static const char* names[] = {"asep", "acep", "asap", "aseb", "azep", "asip"};
  int nameIdx = -1; const char* nameHit = nullptr;
  for (auto n : names) { int i = p.indexOf(String(" ") + n + " "); if (i >= 0) { nameIdx = i; nameHit = n; break; } }
  if (nameIdx < 0) return false;
  bool g = false;
  for (auto w : greet) if (hasW(p, w)) { g = true; break; }
  if (!g && low.length() > 12) return false;   // nama doang boleh kalau ucapannya pendek
  rest = p.substring(nameIdx + strlen(nameHit) + 1);
  rest.trim();
  return true;
}

static bool wantsPhoto(const String& low) {
  String p = " " + low + " ";
  static const char* w[] = {"foto", "fotoin", "fotokan", "difoto", "kamera", "lihat", "liat", "scan", "soal ini", "baca soal", "jawab soal", "ini apa"};
  for (auto x : w) if (p.indexOf(String(" ") + x + " ") >= 0) return true;
  return false;
}

static bool isBye(const String& low) {
  String p = " " + low + " ";
  static const char* w[] = {"sudah dulu", "udah dulu", "sampai jumpa", "dadah", "bye", "selamat tinggal", "tidur dulu", "udahan", "sudahan"};
  for (auto x : w) if (p.indexOf(String(" ") + x + " ") >= 0) return true;
  return false;
}

// ======================= FOTO SOAL =======================
static void doPhoto(const String& q) {
  if (!camOK) { showError("Kamera tidak aktif. Cek kabel & pin kamera."); return; }
  faceState = F_THINK;
  setStatus("Foto soal... tahan ya!", C_YEL);
  beepTone(1200, 70);
  camera_fb_t* fb = grabPhoto();
  if (!fb) { showError("Gagal ambil foto"); return; }
  beepTone(1800, 90);
  Buf b; String reply, err; bool ok = false;
  if (buildMediaBody(b, OR_MODEL, SYS_PHOTO, q.c_str(), false, fb->buf, fb->len, 2500)) {
    setStatus("Mikir jawabannya...", C_YEL);
    ok = orRequest(b.p, b.len, reply, err);
  } else err = "Memori kurang";
  esp_camera_fb_return(fb);
  b.release();
  if (!ok) { showError(err); return; }
  showReply(reply);
}

static void doChat(const String& text) {
  faceState = F_THINK;
  setStatus("Mikir dulu...", C_YEL);
  String reply, err;
  if (!chatLLM(text, reply, err)) { showError(err); return; }
  showReply(reply);
}

static void goSleep() {
  lastActivity = millis();
  mode = SLEEPING;
  faceState = F_HAPPY;
  setStatus("Bilang: " WAKE_NAME_HINT, C_GRAY);
}

static void handleQuery(const String& text) {
  pagerSet("KAMU", C_CYAN, text);
  delay(700);
  String low = norm(text);
  if (isBye(low)) {
    showReply("Oke, dadah! Panggil aku lagi kalau butuh ya.");
    goSleep();
    return;
  }
  if (wantsPhoto(low)) doPhoto(text); else doChat(text);
  {  // dipuji / disayang -> mata hati
    String pp = " " + low + " ";
    static const char* pw[] = {"makasih", "terima kasih", "sayang", "cinta", "pintar", "hebat", "keren", "lucu", "imut", "gemes", "baik banget"};
    for (auto w : pw) if (pp.indexOf(w) >= 0) { flashFace(F_LOVE, 3000); break; }
  }
  awakeUntil = millis() + AWAKE_MS;
  faceState = F_LISTEN;
  setStatus("Aku dengerin...", C_GREEN);
}

// ======================= WIFI PORTAL =======================
enum ConnState { C_IDLE, C_WAIT, C_OK, C_WRONG, C_NOSSID };
volatile ConnState connState = C_IDLE;
String pSsid, pPass, scanJson = "[]";
bool connReq = false;
uint32_t closeAt = 0;
bool routesSet = false;

static const char PAGE[] PROGMEM = R"rawliteral(<!doctype html><html lang="id"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Asep WiFi</title>
<style>
*{box-sizing:border-box;font-family:system-ui,Arial,sans-serif}
body{margin:0;background:#ffdee9;display:flex;justify-content:center;padding:18px}
.c{background:#fff;border-radius:28px;padding:22px;max-width:380px;width:100%;box-shadow:0 8px 24px #d9739c40}
.f{font-size:52px;text-align:center}h1{font-size:20px;text-align:center;margin:4px 0 2px;color:#7a2e4d}
p.s{text-align:center;color:#9a6b80;margin:0 0 16px;font-size:14px}
label{font-size:13px;color:#7a2e4d;font-weight:600}
input[type=text],input[type=password]{width:100%;padding:12px;border:2px solid #f3b6cc;border-radius:14px;font-size:16px;margin:6px 0 12px}
button{width:100%;padding:14px;border:0;border-radius:14px;background:#ff7fae;color:#fff;font-size:17px;font-weight:700}
button:disabled{opacity:.6}
#m{margin-top:14px;text-align:center;font-weight:600;min-height:22px}
.err{color:#d62f4f}.ok{color:#1f9d55}.w{color:#9a6b80}
.sh{font-size:13px;color:#7a2e4d;display:flex;gap:6px;align-items:center;margin-bottom:12px}
</style></head><body><div class="c">
<div class="f">&#127841;</div><h1>Asep Mochi</h1><p class="s">Sambungkan Asep ke WiFi kamu</p>
<form id="f"><label>Nama WiFi (SSID)</label>
<input type="text" id="ssid" list="list" autocomplete="off" required placeholder="Pilih / ketik nama WiFi">
<datalist id="list"></datalist>
<label>Password</label>
<input type="password" id="pass" placeholder="Password WiFi">
<div class="sh"><input type="checkbox" id="show"> <span>Tampilkan password</span></div>
<button id="b" type="submit">Hubungkan</button></form>
<div id="m"></div></div>
<script>
const $=i=>document.getElementById(i);
function msg(t,c){const m=$('m');m.textContent=t;m.className=c||'';}
fetch('/scan').then(r=>r.json()).then(l=>{$('list').innerHTML=l.map(s=>'<option value="'+s.replace(/"/g,'&quot;')+'">').join('')}).catch(()=>{});
$('show').onchange=e=>{$('pass').type=e.target.checked?'text':'password'};
async function poll(){
 try{const t=await (await fetch('/status')).text();
  if(t=='wait'){setTimeout(poll,1000);return;}
  if(t=='ok'){msg('Terhubung! Asep sudah online. Halaman ini bisa ditutup.','ok');return;}
  if(t=='wrong')msg('Password anda salah, coba lagi','err');
  else if(t=='nossid')msg('WiFi tidak ditemukan, cek nama WiFi lalu coba lagi','err');
  else msg('Gagal, coba lagi','err');
  $('b').disabled=false;
 }catch(e){setTimeout(poll,1500);}
}
$('f').onsubmit=async e=>{
 e.preventDefault();$('b').disabled=true;msg('Menghubungkan... tunggu sebentar','w');
 const b=new URLSearchParams({ssid:$('ssid').value,pass:$('pass').value});
 try{await fetch('/connect',{method:'POST',body:b});}catch(x){}
 setTimeout(poll,800);
};
</script></body></html>)rawliteral";

static void startPortal() {
  portalActive = true;
  connState = C_IDLE;
  faceState = F_HAPPY;
  WiFi.disconnect(true);
  WiFi.mode(WIFI_AP_STA);
  setStatus("Mencari WiFi sekitar...", C_YEL);
  int n = WiFi.scanNetworks();
  JsonDocument d;
  JsonArray a = d.to<JsonArray>();
  for (int i = 0; i < n; i++) {
    String s = WiFi.SSID(i);
    if (s.length() == 0) continue;
    bool dup = false;
    for (JsonVariant v : a) if (s == v.as<String>()) { dup = true; break; }
    if (!dup) a.add(s);
  }
  scanJson = ""; serializeJson(d, scanJson);
  WiFi.scanDelete();
  WiFi.softAP(AP_NAME);
  delay(150);
  IPAddress ip = WiFi.softAPIP();
  dns.start(53, "*", ip);
  if (!routesSet) {
    routesSet = true;
    server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", PAGE); });
    server.on("/scan", HTTP_GET, []() { server.send(200, "application/json", scanJson); });
    server.on("/status", HTTP_GET, []() {
      const char* s = connState == C_WAIT ? "wait" : connState == C_OK ? "ok" : connState == C_WRONG ? "wrong" : connState == C_NOSSID ? "nossid" : "idle";
      server.send(200, "text/plain", s);
    });
    server.on("/connect", HTTP_POST, []() {
      pSsid = server.arg("ssid"); pPass = server.arg("pass"); pSsid.trim();
      if (pSsid.length() == 0) { server.send(400, "text/plain", "kosong"); return; }
      connState = C_WAIT; connReq = true;
      server.send(200, "text/plain", "started");
    });
    server.onNotFound([]() {   // captive portal: semua URL diarahkan ke halaman setup
      server.sendHeader("Location", "http://192.168.4.1/", true);
      server.send(302, "text/plain", "");
    });
  }
  server.begin();
  pagerSet("SETUP WIFI", C_YEL, "1. Sambungkan HP ke WiFi \"" AP_NAME "\"\n2. Halaman setup terbuka otomatis (atau buka 192.168.4.1)");
  setStatus("Menunggu HP...", C_YEL);
}

static void stopPortal() {
  server.stop();
  dns.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  portalActive = false;
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(pSsid.c_str(), pPass.c_str());
    uint32_t t = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t < 10000) delay(200);
  }
}

static void doConnect() {
  connState = C_WAIT;
  setStatus("Menghubungkan WiFi...", C_YEL);
  WiFi.disconnect(false);
  delay(100);
  WiFi.begin(pSsid.c_str(), pPass.c_str());
  uint32_t t = millis();
  wl_status_t st = WL_IDLE_STATUS;
  while (millis() - t < 15000) {
    st = WiFi.status();
    if (st == WL_CONNECTED || st == WL_CONNECT_FAILED) break;
    if (st == WL_NO_SSID_AVAIL && millis() - t > 4000) break;
    dns.processNextRequest(); server.handleClient();
    delay(100);
  }
  if (st == WL_CONNECTED) {
    prefs.putString("ssid", pSsid);
    prefs.putString("pass", pPass);
    connState = C_OK;
    closeAt = millis() + 3500;
    setStatus("WiFi terhubung!", C_GREEN);
  } else {
    WiFi.disconnect(false);
    connState = (st == WL_NO_SSID_AVAIL) ? C_NOSSID : C_WRONG;
    setStatus(connState == C_NOSSID ? "WiFi tidak ditemukan" : "Password anda salah!", C_RED);
  }
}

static void portalLoop() {
  dns.processNextRequest();
  server.handleClient();
  if (connReq) { connReq = false; doConnect(); }
  if (connState == C_OK && millis() > closeAt) { stopPortal(); onOnline(); }
}

static void resetWifi() {
  prefs.remove("ssid"); prefs.remove("pass");
  mode = SLEEPING;
  startPortal();
}

void onOnline() {
  portalActive = false;
  WiFi.setSleep(false);          // cegah WiFi tidur/putus setelah beberapa menit
  WiFi.setAutoReconnect(true);
  mode = SLEEPING;
  faceState = F_HAPPY;
  lastActivity = millis();
  hCount = 0;
  pagerSet("ASEP", C_PINK, "Halo! Aku Asep, temen ngobrolmu. Bilang \"" WAKE_NAME_HINT "\" buat manggil aku. Tekan BOOT buat foto soal.");
  setStatus("Dengerin suara sekitar...", C_GRAY);
  calibrateNoise();
  delay(1500);
  goSleep();
}

// ======================= SETUP & LOOP =======================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_BTN, INPUT_PULLUP);
  pinMode(LCD_BLK, OUTPUT);
  digitalWrite(LCD_BLK, HIGH);
  tftMux = xSemaphoreCreateMutex();

  SPI.begin(LCD_SCL, -1, LCD_SDA, -1);
  tft.init(240, 240, SPI_MODE3);
  tft.setRotation(0);
  tft.fillScreen(0x0000);
  tft.setTextSize(3);
  tft.setTextColor(C_WHITE);
  tft.setCursor(21, 100);
  tft.print("STARTING...");
  uint32_t t0 = millis();

  fc = new GFXcanvas16(240, 130);
  fc->setTextWrap(false);
  if (!psramFound()) { bootStatus("PSRAM TIDAK TERDETEKSI!"); delay(2500); }
  wavBuf = (uint8_t*)ps_malloc(44 + REC_MAX_SAMPLES * 2);
  if (!wavBuf) wavBuf = (uint8_t*)malloc(44 + 16000 * 2 * 3);

  bootStatus("Init mic & speaker...");
  micInit();
  spkInit();
  bootStatus("Init kamera...");
  camOK = initCamera();
  if (!camOK) { bootStatus("Kamera gagal init"); delay(800); }

  prefs.begin("asep", false);
  String ssid = prefs.getString("ssid", "");
  String pass = prefs.getString("pass", "");
  bool wifiOK = false;
  if (ssid.length()) {
    bootStatus("Menghubungkan WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), pass.c_str());
    uint32_t t = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t < 12000) delay(200);
    wifiOK = (WiFi.status() == WL_CONNECTED);
  }
  while (millis() - t0 < 3000) delay(10);   // minimal 3 detik "STARTING..."

  tft.fillScreen(C_TXTBG);
  lastActivity = millis();
  xTaskCreatePinnedToCore(faceTask, "face", 8192, NULL, 1, NULL, 0);
  if (wifiOK) onOnline(); else startPortal();
}

void loop() {
  pollButton();
  if (btnLong) { btnLong = false; btnShort = false; resetWifi(); return; }
  if (portalActive) { portalLoop(); delay(2); return; }

  // Penjaga memori: kalau heap hampir habis (bocor), restart otomatis biar tidak macet
  if (ESP.getFreeHeap() < 25000) { Serial.println("[WARN] heap rendah, restart"); delay(200); ESP.restart(); }

  if (WiFi.status() != WL_CONNECTED) {
    static uint32_t lostAt = 0, lastTry = 0;
    if (!lostAt) lostAt = millis();
    setStatus("WiFi putus, nyambung lagi...", C_RED);
    if (millis() - lastTry > 10000) { lastTry = millis(); WiFi.reconnect(); }
    if (millis() - lostAt > 90000) { lostAt = 0; startPortal(); }
    delay(300);
    if (WiFi.status() == WL_CONNECTED) { lostAt = 0; if (mode == SLEEPING) goSleep(); else setStatus("Aku dengerin...", C_GREEN); }
    return;
  }

  if (btnShort) {   // tombol BOOT = foto soal
    btnShort = false;
    doPhoto("Jawab soal pada foto ini selengkap dan seakurat mungkin.");
    mode = AWAKE; awakeUntil = millis() + AWAKE_MS;
    faceState = F_LISTEN;
    setStatus("Aku dengerin...", C_GREEN);
    return;
  }

  uint32_t wait = 0;
  if (mode == AWAKE) wait = (awakeUntil > millis()) ? (awakeUntil - millis()) : 1;
  size_t n = captureUtterance(wait, mode == AWAKE ? 400 : 500, mode == AWAKE ? 10000 : 2500, mode == SLEEPING);
  if (n == 0) {
    if (mode == AWAKE && millis() >= awakeUntil) goSleep();
    return;
  }

  faceState = F_THINK;
  setStatus("Memproses suara...", C_YEL);
  String text, err;
  if (!transcribe(n, text, err)) {
    showError("STT: " + err);
    if (mode == SLEEPING) goSleep(); else setStatus("Aku dengerin...", C_GREEN);
    return;
  }
  text = clean(text); text.trim();
  Serial.println("[STT] " + text);
  String low = norm(text);
  if (low.length() == 0 || low.indexOf("kosong") == 0) {
    if (mode == SLEEPING) goSleep(); else { faceState = F_LISTEN; setStatus("Aku dengerin...", C_GREEN); }
    return;
  }

  if (mode == SLEEPING) {
    String rest;
    if (isWake(low, rest)) {
      mode = AWAKE;
      awakeUntil = millis() + AWAKE_MS;
      flashFace(F_SURPRISE, 900);
      beepTone(880, 90); beepTone(1320, 120);
      faceState = F_LISTEN;
      setStatus("Aku dengerin...", C_GREEN);
      micFlush();
      if (rest.length() >= 4) handleQuery(rest);   // "Halo Asep, jam berapa?" langsung dijawab
    } else {
      goSleep();   // bukan kata kunci -> abaikan
    }
    return;
  }
  handleQuery(text);
}

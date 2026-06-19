/**
 * ================================================================
 *  ESP32-S3 TRICORDER — Test Build v3.8.2
 * ================================================================
 *
 *  CHANGES FROM v3.8.1:
 *
 *  SPEAKER IDLE HISS FIX — soft-ramp PWM wake/sleep.
 *
 *  The PWM carrier (78kHz on GPIO38) was running constantly even
 *  when no sound was playing, producing faint hiss on the bare
 *  speaker. Now the LEDC channel is detached and the pin held LOW
 *  whenever the audio engine is idle — pure silence.
 *
 *  Wake (when a sound starts): attach pin, ramp duty 0 → 128 over
 *  ~30ms. The slow ramp lets the speaker cone settle smoothly so
 *  there's no pop from the carrier engaging.
 *
 *  Sleep (after audio task idles for 50ms): ramp duty 128 → 0 over
 *  ~30ms, detach pin, drive LOW. Same gentle ramp on the way down.
 *
 *  Net effect: in scan-loop modes (ENV/MEASURE/THERMAL) the speaker
 *  stays awake the whole time so loops play seamlessly. In silent
 *  modes (SOUND/WIFI/FILES/BIO) the speaker shuts off ~50ms after
 *  any click sound finishes.
 *
 *  CHANGES FROM v3.8:
 *
 *  WEB FILE BROWSER — file delete via /delete?path=...
 *  Each file in the directory listing has a [del] link. JS confirm
 *  prompts before deletion. Refuses to delete directories.
 *
 *  CHANGES FROM v3.7.6:
 *
 *  SOUND MODE OVERHAUL — gapless recording + oscilloscope UI.
 *  readSoundFast() drains the I2S buffer every loop iteration; PDM
 *  DMA buffers doubled to ~745ms slack; MAX_REC_SECONDS auto-stop;
 *  thermal-style oscilloscope UI with RMS/Peak/Freq stats.
 *
 *  CHANGES FROM v3.7.5:
 *
 *  SCAN LOOP RESILIENCE — scan SFX persists across button presses.
 *  Only stops on freeze. Resumes on unfreeze or mode change.
 *
 *  CHANGES FROM v3.7.4:
 *
 *  AUDIO POP FIX — park PWM at silence midpoint between sounds,
 *  extended fade window from 5ms to 15ms.
 *
 *  CHANGES FROM v3.7.3:
 *
 *  AUDIO QUALITY — MASTER_VOLUME and AUDIO_DITHER defines.
 *  WAV parser hot-fix for trimmed/edited files.
 *
 *  CHANGES FROM v3.7.2:
 *
 *  AUDIO TIMING — absolute-time tracking, watchdog-safe playback.
 *
 *  CHANGES FROM v3.7:
 *
 *  AUDIO ENGINE — non-blocking PWM WAV player with fade-in/out and
 *  looping mode for scan SFX.
 *
 *  Per-trigger SFX with graceful fallback to beep() if the WAV is
 *  missing or SD isn't mounted:
 *
 *    boot.wav            startup
 *    left.wav  right.wav LEFT / RIGHT mode buttons
 *    top.wav   topHold.wav  TOP tap / hold + click events
 *    bottom.wav          BOTTOM (snapshot freeze)
 *    clkWise.wav cntClkWise.wav  encoder direction
 *    envScan.wav         ENV mode (looping, persists across clicks)
 *    measureScan.wav     MEASURE mode (looping, persists across clicks)
 *    thermalScan.wav     THERMAL mode (looping, persists across clicks)
 *    wifiScan.wav        TOP-hold wireless scan (one-shot)
 *
 *  PRIOR FEATURES (v3.5 and earlier):
 *    - WiFi submenu (TOP-hold in FILES) for AP host / STA join /
 *      HTTP file browser at http://192.168.4.1
 *    - Data-driven 8x8 matrix icons per mode (5 in ENV, 3 in MEASURE)
 *    - Thermal: 8x8 → 16x16 bilinear interpolation, ABS/REL modes
 *    - APDS9999 native driver with white-balance gain calibration
 *    - Per-mode CSV snapshots + master /logs/log.csv
 *    - Diagnostic log at /tricorder/config/diag.log
 * ================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>
#include <stdarg.h>

#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Adafruit_LEDBackpack.h>

#include <Adafruit_LIS2MDL.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LTR390.h>
#include <Adafruit_BME680.h>
#include <Adafruit_AMG88xx.h>
#include <vl53l4cx_class.h>

#include <WiFi.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <WebServer.h>
#include <FS.h>
#include <time.h>

#include <driver/i2s.h>
#include <esp_timer.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// ================================================================
//  PINS
// ================================================================
#define I2C_SDA       5
#define I2C_SCL       6
#define TFT_DC        7
#define TFT_RST       8
#define TFT_CS        9
#define SPI_MOSI      10
#define SPI_SCK       11
#define SPI_MISO      12
#define PDM_CLK_PIN   13
#define PDM_DATA_PIN  14
#define SD_CS         19
#define ENC_SW        20
#define BTN_RIGHT     21
#define SPEAKER_PIN   38
#define BTN_LEFT      39
#define BTN_BOTTOM    40
#define ENC_CLK       41
#define ENC_DT        42
#define LED_GROUP     47
#define BTN_TOP       48

#define LEDC_SPK_CH   0
#define LEDC_SPK_RES  8
#define LEDC_LED_CH   1
#define LEDC_LED_FREQ 1000
#define LEDC_LED_RES  8

// ================================================================
//  MUX
// ================================================================
#define MUX_ADDR  0x71
#define CH_APDS   5
#define CH_LIS    2
#define CH_LTR    3
#define CH_BME    1
#define CH_MATRIX 0
#define CH_AMG    6
#define CH_VL53   7

// ================================================================
//  DISPLAY
// ================================================================
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
#define TFT_HEADER_H  18
#define TFT_DATA_Y    22
#define TFT_W         160
#define TFT_H         128
#define TFT_ORANGE    0xFD20
#define TFT_DARK_TEAL 0x1082
#define VAL_X         64
#define VAL_W         (TFT_W - VAL_X)

// ================================================================
//  LED MATRIX
// ================================================================
Adafruit_8x8matrix matrix = Adafruit_8x8matrix();

void switchMux(uint8_t ch);
void muxDisable();
void tftDrawWirelessList();
void tftDrawFilesList();
void tftDrawSoundScope();
void readWireless();
void tftDrawHeader();

void matrixSelect()       { switchMux(CH_MATRIX); }
void matrixDeselect()     { muxDisable(); }
void matrixWriteDisplay() { matrix.writeDisplay(); }
void matrixClear()        { matrix.clear(); }
inline void px(int x, int y) {
  if(x>=0&&x<8&&y>=0&&y<8) matrix.drawPixel(x,y,LED_ON);
}

void drawModeEnvironmental(uint32_t f);
void drawModeMeasure(uint32_t f);
void drawModeSound(uint32_t f);
void drawModeBio(uint32_t f);
void drawModeWireless(uint32_t f);
void drawModeFiles(uint32_t f);

// ================================================================
//  DATA STRUCTS
// ================================================================
struct EnvData {
  float temperature=0,humidity=0,pressure=0,gasKOhm=0;
  float uvRaw=0,uvIndex=0;
  bool  valid=false;
};
struct MeasureData {
  int      distanceMM=0;
  bool     distanceValid=false;
  float    magX=0,magY=0,magZ=0,heading=0;
  float    rawMagX=0,rawMagY=0,rawMagZ=0;
  float    magOffX=0,magOffY=0,magOffZ=0;
  bool     magCalibrated=false;
  // APDS9999 is 20-bit per channel (max 1,048,575). Green doubles as
  // luminance/clear reference. IR is a separate channel.
  uint32_t colorR=0,colorG=0,colorB=0,colorC=0;
  uint32_t colorIR=0;
  bool     apdsOk=false;
  // Legacy mirror for snapshot CSV compatibility
  uint32_t regColorR=0,regColorG=0,regColorB=0,regColorC=0;
  bool     valid=false;
};
struct ThermalData {
  float pixels[64]={0};
  float interp[256]={0};
  float centerTemp=0,minTemp=0,maxTemp=0,deltaTemp=0;
  bool  valid=false;
};
struct SoundData {
  int16_t buffer[1024]={0};
  float   rmsLevel=0,peakLevel=0;
  bool    valid=false;
};
struct WirelessData {
  int  wifiCount=0;
  char wifiSSID[10][33]={};
  int  wifiRSSI[10]={0};
  int  wifiChannel[10]={0};
  int  bleCount=0;
  char bleName[10][33]={};
  char bleAddress[10][18]={};
  int  bleRSSI[10]={0};
  bool scanning=false,valid=false;
};
#define MAX_FILES 20
struct FileEntry {
  char name[33];
  bool isDir;
  int  size;
  bool valid;
};
struct FilesData {
  FileEntry entries[MAX_FILES];
  int       count;
  char      currentPath[64];
  bool      needsRefresh;
};

// ================================================================
//  SYSTEM STATE
// ================================================================
enum MacroMode : uint8_t {
  MODE_ENVIRONMENTAL=0,MODE_MEASURE,MODE_SOUND,
  MODE_BIO,MODE_THERMAL,MODE_WIRELESS,MODE_FILES,MODE_COUNT
};
const char* const MODE_NAMES[MODE_COUNT]={
  "ENVIRONMENTAL","MEASURE","SOUND","BIO","THERMAL","WIRELESS","FILES"
};
const char* const MODE_SHORT[MODE_COUNT]={
  "ENV","MEASURE","SOUND","BIO","THERMAL","WIFI","FILES"
};
#define ITEMS_PER_PAGE 4

struct SystemState {
  MacroMode     currentMode        = MODE_ENVIRONMENTAL;
  MacroMode     lastMode           = MODE_COUNT;
  bool          snapshotFrozen     = false;
  bool          sdMounted          = false;
  bool          calibrating        = false;
  bool          labelsDrawn        = false;
  bool          thermalAbsolute    = true;
  bool          wirelessShowBLE    = false;
  int           wifiScrollIdx      = 0;
  int           bleScrollIdx       = 0;
  bool          wirelessShowDetail = false;
  int           wirelessDetailIdx  = 0;

  // Matrix sub-mode indices — encoder scrolls through icon variants
  // ENV: 0=temp 1=humidity 2=pressure 3=gas 4=uv
  // MEASURE: 0=distance 1=heading(compass) 2=color(bargraph)
  int           envIconIdx     = 0;
  int           measureIconIdx = 0;
  int           filesScrollIdx     = 0;
  bool          filesShowDetail    = false;
  int           filesDetailIdx     = 0;
  unsigned long lastSensorUpdate   = 0;
  unsigned long sensorIntervalMs   = 500;
  unsigned long lastMatrixUpdate   = 0;
  unsigned long lastLedUpdate      = 0;
  int           matrixFrame        = 0;
  bool          scanLoopWanted     = false;  // scan loop should be playing for current mode (kept true across clicks)
};

EnvData      envData;
MeasureData  measureData;
ThermalData  thermalData;
SoundData    soundData;
WirelessData wirelessData;
FilesData    filesData;
SystemState  sys;

// ================================================================
//  WIFI FEATURE STATE (v3.5)
// ================================================================
enum WifiSubmode : uint8_t {
  WIFI_OFF_STATE = 0,    // not in submenu, radio off
  WIFI_MENU,             // submenu open, scrolling options
  WIFI_AP_ACTIVE,        // hosting AP
  WIFI_STA_ACTIVE,       // connected to home WiFi
  WIFI_AP_SETUP,         // AP mode serving setup form (no creds)
  WIFI_STA_CONNECTING    // mid-connect, show progress
};

struct WifiState {
  WifiSubmode    mode             = WIFI_OFF_STATE;
  int            menuIdx          = 0;        // 0=Host AP, 1=Join WiFi, 2=Exit
  char           apSsid[32]       = {0};      // "Tricorder-XXXX"
  char           ipStr[16]        = {0};      // current IP for display
  char           connectedSsid[33]= {0};      // SSID we're joined to
  bool           ntpSynced        = false;
  unsigned long  startTime        = 0;        // when wifi powered on
  unsigned long  connectAttemptStart = 0;
  int            httpReqCount     = 0;
  bool           setupFallback    = false;    // true if STA failed → AP setup
};
WifiState wifiSt;
WebServer httpServer(80);

// Forward decls for the WiFi module
void wifiEnter();
void wifiExit();
void wifiStartAP(bool setupMode);
void wifiStartSTA();
void wifiTickConnecting();
void wifiTickActive();
void wifiDrawTFT();
bool wifiLoadCreds(char* ssid, size_t ssidLen, char* pass, size_t passLen);
bool wifiSaveCreds(const char* ssid, const char* pass);
void httpHandleRoot();
void httpHandleBrowse();
void httpHandleFile();
void httpHandleDelete();
void httpHandleStatus();
void httpHandleSetup();
void httpHandleSetupPost();
void httpHandleNotFound();

// ================================================================
//  SENSOR OBJECTS
// ================================================================
// APDS9999 / APDS-9250 register map (Broadcom, native driver — no Adafruit lib)
// The Adafruit_APDS9960 library was the wrong chip and would never work here.
#define APDS_ADDR        0x52
#define APDS_MAIN_CTRL   0x00
#define APDS_LS_MEAS_RATE 0x04
#define APDS_LS_GAIN     0x05
#define APDS_PART_ID     0x06
#define APDS_MAIN_STATUS 0x07
#define APDS_LS_DATA_IR  0x0A  // 3 bytes (20-bit)
#define APDS_LS_DATA_GRN 0x0D  // 3 bytes — also the lux/clear reference
#define APDS_LS_DATA_BLU 0x10
#define APDS_LS_DATA_RED 0x13

// ----------------------------------------------------------------
//  APDS9999 WHITE BALANCE GAINS — calibrated against plain white
//  paper under your room lighting on 2026-04-28.
//  Source: /wb_cal.log, 30-sample average.
//  Raw avg was R=3273 G=5395 B=3290, so green is ~1.65x more
//  sensitive than red and blue — these gains compensate.
//  After applying: white paper should read ~5395 across all channels.
// ----------------------------------------------------------------
static const float WB_GAIN_R = 1.6483f;
static const float WB_GAIN_G = 1.0000f;
static const float WB_GAIN_B = 1.6398f;

Adafruit_LIS2MDL  lis2mdl(12345);
Adafruit_LTR390   ltr;
Adafruit_BME680   bme;
Adafruit_AMG88xx  amg;
VL53L4CX          tof(&Wire,-1);

// ================================================================
//  AUDIO ENGINE — non-blocking PWM WAV player
//
//  Plays 22050Hz mono 16-bit WAV files from SD via the existing LEDC
//  channel on GPIO38. Sample buffer lives in PSRAM (we have 8MB).
//  A FreeRTOS task pumps samples to the PWM duty cycle at the audio
//  sample rate. Pinned to core 0 so it doesn't fight Arduino's main
//  loop on core 1. Main loop keeps running sensors/encoder/UI.
//
//  If a WAV is missing or SD isn't mounted, callers fall back to
//  the original beep() patterns so the device always makes sound.
//
//  Only one sound at a time. New playSfx() calls preempt any
//  currently-playing sound.
// ================================================================
#define AUDIO_SAMPLE_RATE   22050
#define AUDIO_PWM_FREQ      78125   // ~256 steps at 8-bit, well above audio band
#define AUDIO_TICK_US       (1000000UL / AUDIO_SAMPLE_RATE)   // 45us per sample
#define SOUNDS_DIR          "/tricorder/sounds"

// Master volume — scales all SFX. 0=mute, 100=full, default 50.
// Higher = louder but harsher. Lower = quieter and softer-sounding.
// Tweak this freely; cost is paid once at WAV load, not during playback.
#define MASTER_VOLUME 25

// Dither — adds a small random offset before 16→8 bit truncation to
// mask quantization noise. Quantization grit becomes soft hiss instead.
// Set to 0 to disable, 1 to enable. Negligible CPU cost.
#define AUDIO_DITHER  1

struct AudioEngine {
  uint8_t*       buf       = nullptr;    // PSRAM, 8-bit unsigned PCM
  volatile size_t  len     = 0;
  volatile size_t  pos     = 0;
  volatile bool    playing = false;
  volatile bool    loop    = false;      // restart at end-of-buffer if true
  volatile bool    spkAwake = false;     // PWM pin attached + ramped up
  size_t         capacity  = 0;
} audio;

portMUX_TYPE audioMux = portMUX_INITIALIZER_UNLOCKED;
SemaphoreHandle_t audioSpkMutex = nullptr;   // protects wake/sleep ramps

// Wake the speaker: attach the LEDC pin and slowly ramp duty from 0
// to 128 (silence midpoint) over ~30ms. The slow ramp prevents the
// PWM carrier coming online from causing a pop. Idempotent — calling
// when already awake is a no-op.
//
// The speaker stays awake until audioSleepSpeaker() is called. While
// awake, PWM is constantly switching the GPIO at AUDIO_PWM_FREQ,
// which on a bare speaker (no LPF) produces faint hiss. So we keep
// it awake only while sounds are actively playing — sleep takes the
// pin LOW which is silent.
//
// Mutex-protected: wake and sleep ramps cannot run concurrently.
void audioWakeSpeaker() {
  if(!audioSpkMutex) return;
  xSemaphoreTake(audioSpkMutex, portMAX_DELAY);
  if(audio.spkAwake) { xSemaphoreGive(audioSpkMutex); return; }
  ledcAttachPin(SPEAKER_PIN, LEDC_SPK_CH);
  // Ramp duty 0 → 128 over ~30ms (15 steps, ~2ms each)
  for(int d = 0; d <= 128; d += 9) {
    ledcWrite(LEDC_SPK_CH, d);
    delay(2);
  }
  ledcWrite(LEDC_SPK_CH, 128);
  audio.spkAwake = true;
  xSemaphoreGive(audioSpkMutex);
}

// Sleep the speaker: ramp duty from current (assumed 128) down to 0
// over ~30ms, then detach the pin and pull GPIO38 LOW. Eliminates
// idle hiss from the PWM carrier. Idempotent.
void audioSleepSpeaker() {
  if(!audioSpkMutex) return;
  xSemaphoreTake(audioSpkMutex, portMAX_DELAY);
  if(!audio.spkAwake) { xSemaphoreGive(audioSpkMutex); return; }
  for(int d = 128; d >= 0; d -= 9) {
    ledcWrite(LEDC_SPK_CH, d);
    delay(2);
  }
  ledcWrite(LEDC_SPK_CH, 0);
  ledcDetachPin(SPEAKER_PIN);
  pinMode(SPEAKER_PIN, OUTPUT);
  digitalWrite(SPEAKER_PIN, LOW);
  audio.spkAwake = false;
  xSemaphoreGive(audioSpkMutex);
}

// Audio playback task — runs on core 0, pinned. Samples are written to
// the LEDC duty cycle at exactly 22050Hz using absolute-time tracking.
//
// Design notes:
//   - We track playStartUs (the wall-clock time playback began) and
//     compute the target time for sample N as playStartUs + N * 45us.
//     This means if vTaskDelay sleeps slightly long (it sleeps 1-2ms
//     instead of exactly 1ms), the next batch automatically catches
//     up by writing samples faster — playback never drifts.
//   - We process samples in 10ms batches (220 samples) so the
//     vTaskDelay(1) overhead is amortized to <10% of batch time.
//   - vTaskDelay(1) is essential — it lets the idle task on core 0
//     run, which feeds the watchdog. taskYIELD() is NOT enough here
//     because nothing else at our priority is ready on core 0.
//   - If we fall more than 50ms behind (e.g. SD card ate CPU briefly)
//     we skip ahead rather than try to catch up at warp speed.
void audioTask(void* /*param*/) {
  const int     SAMPLES_PER_BATCH = 220;          // ~10ms at 22050Hz
  const int64_t MAX_DRIFT_US      = 50000;        // 50ms before we resync
  int64_t       playStartUs       = 0;
  size_t        playStartPos      = 0;

  while(true) {
    bool playing;
    portENTER_CRITICAL(&audioMux);
    playing = audio.playing;
    portEXIT_CRITICAL(&audioMux);

    if(!playing) {
      // Idle — sleep the speaker if we've been idle long enough that
      // no new sound is coming. Grace window is ~50ms so rapid-fire
      // clicks (button mash) don't ramp pin off then on again.
      static unsigned long idleSinceMs = 0;
      if(audio.spkAwake) {
        unsigned long now = millis();
        if(idleSinceMs == 0) {
          idleSinceMs = now;
        } else if(now - idleSinceMs > 50) {
          // Confirm still idle right before ramping
          portENTER_CRITICAL(&audioMux);
          bool stillIdle = !audio.playing;
          portEXIT_CRITICAL(&audioMux);
          if(stillIdle) audioSleepSpeaker();
          idleSinceMs = 0;
        }
      } else {
        idleSinceMs = 0;
      }
      vTaskDelay(pdMS_TO_TICKS(10));
      playStartUs = 0;        // reset on next play
      continue;
    }

    // First iteration of a new playback: anchor our timing to "now".
    if(playStartUs == 0) {
      playStartUs  = esp_timer_get_time();
      portENTER_CRITICAL(&audioMux);
      playStartPos = audio.pos;
      portEXIT_CRITICAL(&audioMux);
    }

    for(int i = 0; i < SAMPLES_PER_BATCH; i++) {
      size_t pos;
      bool   stillPlaying;
      portENTER_CRITICAL(&audioMux);
      pos          = audio.pos;
      stillPlaying = audio.playing;
      portEXIT_CRITICAL(&audioMux);

      if(!stillPlaying) break;

      if(pos >= audio.len) {
        if(audio.loop) {
          // Loop reset: anchor timing fresh so seam doesn't drift.
          portENTER_CRITICAL(&audioMux);
          audio.pos    = 0;
          portEXIT_CRITICAL(&audioMux);
          playStartUs  = esp_timer_get_time();
          playStartPos = 0;
          continue;
        }
        portENTER_CRITICAL(&audioMux);
        audio.playing = false;
        portEXIT_CRITICAL(&audioMux);
        ledcWrite(LEDC_SPK_CH, 128);   // park at silence midpoint, not 0 (would pop)
        playStartUs = 0;
        break;
      }

      ledcWrite(LEDC_SPK_CH, audio.buf[pos]);

      portENTER_CRITICAL(&audioMux);
      audio.pos++;
      portEXIT_CRITICAL(&audioMux);

      // Compute absolute target time for the *next* sample.
      // Sample N (relative to playStartPos) plays at playStartUs + N*45us.
      int64_t nextSamplePos = (int64_t)(pos + 1 - playStartPos);
      int64_t targetUs      = playStartUs + nextSamplePos * AUDIO_TICK_US;
      int64_t now           = esp_timer_get_time();
      int64_t drift         = now - targetUs;

      if(drift > MAX_DRIFT_US) {
        // Fell badly behind — resync rather than playing back at warp speed.
        playStartUs  = now;
        playStartPos = pos + 1;
      } else {
        // Tight spin until target time. Only ~45us per iteration in the
        // common case; can skip entirely if we're already late.
        while(esp_timer_get_time() < targetUs) { }
      }
    }

    // Yield 1 tick to feed the idle task / watchdog. The absolute-time
    // tracking above means the next batch will catch up if this sleeps long.
    vTaskDelay(1);
  }
}

void audioInit() {
  audioSpkMutex = xSemaphoreCreateMutex();
  ledcSetup(LEDC_SPK_CH, AUDIO_PWM_FREQ, LEDC_SPK_RES);
  // Don't attach pin yet — start in sleep state, hold pin LOW. Pin is
  // attached on first playSfx() / beep() call by audioWakeSpeaker().
  pinMode(SPEAKER_PIN, OUTPUT);
  digitalWrite(SPEAKER_PIN, LOW);
  audio.spkAwake = false;

  // Allocate a generous PSRAM buffer up front. Largest WAV is ~90KB
  // (boot.wav at 2s); 256KB gives plenty of room for future sounds.
  if(psramFound()) {
    audio.capacity = 256 * 1024;
    audio.buf = (uint8_t*)ps_malloc(audio.capacity);
  }
  if(!audio.buf) {
    Serial.println("[AUDIO] PSRAM alloc failed — SFX disabled, beeps only");
    audio.capacity = 0;
    return;
  }
  Serial.printf("[AUDIO] PSRAM buffer %u KB ready\n", audio.capacity/1024);

  // Start playback task on core 0, priority 5 (high enough to keep
  // sample timing tight, low enough not to starve WiFi/system tasks).
  xTaskCreatePinnedToCore(audioTask, "audioTask", 4096, NULL, 5, NULL, 0);
}

// Blocking beep — used for fallback and during cal screens where
// blocking is fine. Stops any currently-playing SFX. Wakes the
// speaker if needed; relies on the audio task's idle detection to
// sleep it again after the beep sequence.
void beep(uint32_t hz, uint32_t ms) {
  portENTER_CRITICAL(&audioMux);
  audio.playing = false;
  audio.loop    = false;
  portEXIT_CRITICAL(&audioMux);
  audioWakeSpeaker();
  ledcWriteTone(LEDC_SPK_CH, hz);
  ledcWrite(LEDC_SPK_CH, 128);
  delay(ms);
  ledcWrite(LEDC_SPK_CH, 128);   // back to silence midpoint, don't snap to 0
  ledcWriteTone(LEDC_SPK_CH, 0);
  // Restore PWM frequency for sample playback
  ledcSetup(LEDC_SPK_CH, AUDIO_PWM_FREQ, LEDC_SPK_RES);
}

// Load a WAV from SD into the PSRAM buffer and start playback.
// Returns true on success. Expects 22050Hz mono 16-bit PCM.
// If loop=true, plays continuously until audioStop() or another playSfx().
// A short fade-in/out is applied to kill the pop from PWM duty stepping.
bool playSfx(const char* filename, bool loop = false) {
  if(!sys.sdMounted || !audio.buf) return false;

  char path[64];
  snprintf(path, sizeof(path), "%s/%s", SOUNDS_DIR, filename);
  File f = SD.open(path, FILE_READ);
  if(!f) {
    Serial.printf("[AUDIO] Missing: %s\n", path);
    return false;
  }

  // Parse WAV by scanning chunks — handles non-standard headers, fact/LIST
  // metadata chunks, and fmt chunks larger than 16 bytes.
  uint8_t riff[12];
  if(f.read(riff, 12) != 12 ||
     memcmp(riff, "RIFF", 4) != 0 || memcmp(riff+8, "WAVE", 4) != 0) {
    Serial.printf("[AUDIO] Not a WAV: %s\n", path);
    f.close(); return false;
  }

  uint16_t channels = 0, bitsPer = 0;
  uint32_t sampleRate = 0, dataSize = 0;
  bool fmtFound = false, dataFound = false;

  while(f.available() >= 8) {
    uint8_t chunkId[4]; uint8_t chunkSzBuf[4];
    if(f.read(chunkId, 4) != 4 || f.read(chunkSzBuf, 4) != 4) break;
    uint32_t chunkSz = chunkSzBuf[0] | ((uint32_t)chunkSzBuf[1]<<8)
                     | ((uint32_t)chunkSzBuf[2]<<16) | ((uint32_t)chunkSzBuf[3]<<24);

    if(memcmp(chunkId, "fmt ", 4) == 0) {
      uint8_t fmt[16] = {};
      uint32_t toRead = (chunkSz < 16) ? chunkSz : 16;
      if(f.read(fmt, toRead) != (int)toRead) { f.close(); return false; }
      if(chunkSz > 16) f.seek(f.position() + (chunkSz - 16));  // skip extra fmt bytes
      uint16_t audioFmt = fmt[0] | (fmt[1]<<8);
      if(audioFmt != 1) {
        Serial.printf("[AUDIO] Not PCM (audioFmt=%u): %s\n", audioFmt, path);
        f.close(); return false;
      }
      channels   = fmt[2]  | (fmt[3]<<8);
      sampleRate = fmt[4]  | ((uint32_t)fmt[5]<<8) | ((uint32_t)fmt[6]<<16) | ((uint32_t)fmt[7]<<24);
      bitsPer    = fmt[14] | (fmt[15]<<8);
      fmtFound   = true;
    } else if(memcmp(chunkId, "data", 4) == 0) {
      dataSize  = chunkSz;
      dataFound = true;
      break;  // file cursor is now at start of PCM data
    } else {
      // Unknown chunk — skip it (chunks are word-aligned)
      uint32_t skip = (chunkSz + 1) & ~1u;
      f.seek(f.position() + skip);
    }
  }

  if(!fmtFound || !dataFound) {
    Serial.printf("[AUDIO] Incomplete WAV %s (fmt=%d data=%d)\n", path, fmtFound, dataFound);
    f.close(); return false;
  }
  if(channels != 1 || sampleRate != AUDIO_SAMPLE_RATE || bitsPer != 16) {
    Serial.printf("[AUDIO] Wrong format %s: %uch %uHz %ubit (want 1ch 22050Hz 16bit)\n",
      path, channels, sampleRate, bitsPer);
    f.close(); return false;
  }

  portENTER_CRITICAL(&audioMux);
  audio.playing = false;
  audio.loop    = false;
  audio.pos     = 0;
  portEXIT_CRITICAL(&audioMux);
  ledcWrite(LEDC_SPK_CH, 128);   // park at silence between sounds

  // Read 16-bit samples. Apply volume scale and TPDF dither, then
  // truncate to 8-bit unsigned for PWM duty.
  //
  // Volume: scale each signed sample by (MASTER_VOLUME/100) before
  //   bit-depth reduction. Doing it in the int domain BEFORE
  //   truncation preserves quiet detail that would be lost if we
  //   scaled the 8-bit output.
  //
  // Dither: triangular probability density function (TPDF) — sum of
  //   two uniform random values, in the range of one bit. Added before
  //   we drop the bottom 8 bits. Decorrelates quantization error from
  //   the signal so the resulting noise is broadband hiss instead of
  //   harsh harmonic distortion.
  uint32_t numSamples = dataSize / 2;
  if(numSamples > audio.capacity) numSamples = audio.capacity;

  const size_t CHUNK = 1024;
  int16_t tmp[CHUNK];
  size_t written = 0;
  const int VOL = MASTER_VOLUME;   // local copy for the inner loop
  while(written < numSamples) {
    size_t want = numSamples - written;
    if(want > CHUNK) want = CHUNK;
    size_t got = f.read((uint8_t*)tmp, want * 2) / 2;
    if(got == 0) break;
    for(size_t i = 0; i < got; i++) {
      int32_t s = (int32_t)tmp[i];
      s = (s * VOL) / 100;
#if AUDIO_DITHER
      // TPDF dither: -255..+255 range, centred at 0 (one 8-bit LSB)
      s += (esp_random() & 0xFF) - (esp_random() & 0xFF);
#endif
      // Clamp before bit reduction (volume + dither can theoretically overshoot)
      if(s >  32767) s =  32767;
      if(s < -32768) s = -32768;
      // 16-bit signed [-32768..32767] → 8-bit unsigned [0..255]
      audio.buf[written + i] = (uint8_t)((s + 32768) >> 8);
    }
    written += got;
  }
  f.close();

  // ---- De-pop: fade-in/out + DC offset alignment ----
  // PWM duty stepping from silence to first sample (or last sample to
  // silence) creates a step discontinuity = audible pop. We crossfade
  // both ends toward 128 (silence midpoint in unsigned 8-bit) over
  // ~15ms (330 samples at 22050Hz). 15ms is short enough to be
  // imperceptible inside a sound, long enough to fully damp the pop.
  // For looping sounds we ALSO crossfade the loop seam.
  if(written > 800) {
    const size_t FADE = 330;   // ~15ms at 22050Hz
    // Fade in: blend buf[i] toward 128 at i=0, full strength at i=FADE
    for(size_t i = 0; i < FADE && i < written; i++) {
      float t = (float)i / FADE;
      audio.buf[i] = (uint8_t)(128.0f + (audio.buf[i] - 128.0f) * t);
    }
    // Fade out: full strength at len-FADE, blend toward 128 at len-1
    for(size_t i = 0; i < FADE && i < written; i++) {
      size_t idx = written - 1 - i;
      float t = (float)i / FADE;
      audio.buf[idx] = (uint8_t)(128.0f + (audio.buf[idx] - 128.0f) * t);
    }
    // Loop seam crossfade: when looping, the sample after written-1 is buf[0].
    // If those don't match, you get a click every loop. Average the ends.
    if(loop) {
      uint8_t startVal = audio.buf[0];
      uint8_t endVal   = audio.buf[written - 1];
      uint8_t blend    = (uint8_t)(((int)startVal + (int)endVal) / 2);
      audio.buf[0]           = blend;
      audio.buf[written - 1] = blend;
    }
  }

  // Wake the speaker (ramps PWM up smoothly) before starting playback.
  // This adds ~30ms latency before the first sample but eliminates the
  // pop from the PWM carrier suddenly engaging.
  audioWakeSpeaker();

  portENTER_CRITICAL(&audioMux);
  audio.len     = written;
  audio.pos     = 0;
  audio.loop    = loop;
  audio.playing = true;
  portEXIT_CRITICAL(&audioMux);

  return true;
}

bool audioBusy() { return audio.playing; }

void audioStop() {
  portENTER_CRITICAL(&audioMux);
  audio.playing = false;
  audio.loop    = false;
  portEXIT_CRITICAL(&audioMux);
  ledcWrite(LEDC_SPK_CH, 128);   // park at silence to avoid pop
}

// Wrappers — try WAV first, fall back to original beep pattern if
// the file is missing or audio isn't ready.
void playClick()      { if(!playSfx("top.wav"))         { beep(1200,20); } }
void playEncCW()      { if(!playSfx("clkWise.wav"))     { beep(1400,15); } }
void playEncCCW()     { if(!playSfx("cntClkWise.wav"))  { beep(1000,15); } }
void playBtnLeft()    { if(!playSfx("left.wav"))        { beep(900,30); } }
void playBtnRight()   { if(!playSfx("right.wav"))       { beep(1300,30); } }
void playBtnTop()     { if(!playSfx("top.wav"))         { beep(1200,30); } }
void playBtnTopHold() { if(!playSfx("topHold.wav"))     { beep(800,40);delay(30);beep(1400,40); } }
void playBtnBottom()  { if(!playSfx("bottom.wav"))      { beep(400,60); } }
void playScan()       { if(!playSfx("topHold.wav"))     { beep(800,40);delay(30);beep(1400,40); } }
void playFreeze()     { if(!playSfx("bottom.wav"))      { beep(400,60); } }
void playBoot()       { if(!playSfx("boot.wav"))        { beep(600,80);delay(40);beep(900,80);delay(40);beep(1200,120); } }
void playEnvScan()    { if(!playSfx("envScan.wav", true))     { beep(700,40);delay(20);beep(1000,40); } }
void playMeasScan()   { if(!playSfx("measureScan.wav", true)) { beep(900,40);delay(20);beep(1200,40); } }
void playThermScan()  { if(!playSfx("thermalScan.wav", true)) { beep(500,40);delay(20);beep(800,40); } }
void playWifiScan()   { if(!playSfx("wifiScan.wav"))           { beep(800,40);delay(30);beep(1400,40); } }
// Calibration tones stay synchronous (cal screen blocks anyway)
void playCalStart()   { beep(1000,60);delay(30);beep(1000,60); }
void playCalDone()    { beep(600,60);delay(20);beep(900,60);delay(20);beep(1200,60);delay(20);beep(1600,120); }

// True if the given mode has a continuous scan loop SFX. ENV, MEASURE,
// THERMAL all have looping scan sounds; other modes do not.
bool modeHasScanLoop(MacroMode m) {
  return (m == MODE_ENVIRONMENTAL || m == MODE_MEASURE || m == MODE_THERMAL);
}

// Triggered when entering a sensor mode that has a "scanning into life"
// signature sound. Modes without one fall through silently — the
// directional button click already played.
void playModeEntrySfx(MacroMode m) {
  switch(m) {
    case MODE_ENVIRONMENTAL: playEnvScan();   break;
    case MODE_MEASURE:       playMeasScan();  break;
    case MODE_THERMAL:       playThermScan(); break;
    default: break;
  }
}

// ================================================================
//  SOUND RECORDING GLOBALS
//
//  MAX_REC_SECONDS is the hard auto-stop limit. At 22050Hz mono 16-bit
//  that's 44100 bytes/sec. Default 10 minutes = ~26MB per recording.
//  Tweak freely; FAT32 file size cap is 4GB which is ~25 hours.
// ================================================================
#define MAX_REC_SECONDS 600

static File     recFile;
static bool     soundRecording = false;
static uint32_t recByteCount   = 0;
static uint16_t recIndex       = 0;
static unsigned long recStartMs = 0;

// Zero-crossing rate gives a rough dominant-frequency estimate. We
// count sign-changes per fast read and divide by the time window.
// Only valid for periodic signals; speech/noise gives a wide reading
// that's still useful as "voice ~150-300Hz, whistle ~1-3kHz" feedback.
static volatile uint32_t soundDomFreqHz = 0;

// ================================================================
//  LED UPDATE
// ================================================================
void ledInit() {
  ledcSetup(LEDC_LED_CH,LEDC_LED_FREQ,LEDC_LED_RES);
  ledcAttachPin(LED_GROUP,LEDC_LED_CH);
  ledcWrite(LEDC_LED_CH,0);
}
void ledUpdate() {
  unsigned long now=millis();
  uint8_t brightness=0;
  if(sys.calibrating){brightness=((now/150)%3==0)?255:0;}
  else switch(sys.currentMode){
    case MODE_ENVIRONMENTAL:{float p=(float)(now%3000)/3000.0f*TWO_PI;brightness=(uint8_t)((sinf(p)*0.5f+0.5f)*200.0f);break;}
    case MODE_MEASURE:brightness=200;break;
    case MODE_SOUND:
      // Blink LED while recording, VU meter otherwise
      if(soundRecording) brightness=((now/250)%2)?220:0;
      else brightness=(uint8_t)constrain(soundData.rmsLevel*255.0f,0,200);
      break;
    case MODE_BIO:{int p=(int)(now%1000);brightness=(p<80||(p>160&&p<240))?220:0;break;}
    case MODE_THERMAL:{float n=constrain((thermalData.maxTemp-20.0f)/20.0f,0,1);brightness=(uint8_t)(n*220.0f);break;}
    case MODE_WIRELESS:brightness=wirelessData.scanning?((now/100)%2?220:0):((now/800)%2?100:0);break;
    case MODE_FILES:brightness=100;break;
  }
  ledcWrite(LEDC_LED_CH,brightness);
}

// ================================================================
//  SD
// ================================================================
void sdCreateFolderTree() {
  if(!sys.sdMounted) return;
  const char* folders[]={
    "/tricorder","/tricorder/config","/tricorder/audio","/tricorder/sounds","/tricorder/firmware",
    "/tricorder/snapshots","/tricorder/snapshots/env","/tricorder/snapshots/measure",
    "/tricorder/snapshots/sound","/tricorder/snapshots/bio","/tricorder/snapshots/thermal",
    "/tricorder/snapshots/wifi","/tricorder/snapshots/wifi/all",
    "/tricorder/snapshots/wifi/individual","/tricorder/snapshots/ble",
    "/tricorder/snapshots/ble/all","/tricorder/snapshots/ble/individual","/logs"
  };
  const int nFolders = sizeof(folders)/sizeof(folders[0]);
  for(int i=0;i<nFolders;i++){
    if(!SD.exists(folders[i])){SD.mkdir(folders[i]);Serial.printf("[SD] Created: %s\n",folders[i]);}
  }
}

void sdInit() {
  if(!SD.begin(SD_CS)){
    Serial.println("[SD] FAILED — check SD_CS=GPIO19, card inserted, SPI wiring");
    return;
  }
  sys.sdMounted=true;
  Serial.printf("[SD] OK — cardType:%d  size:%.0fMB\n",SD.cardType(),SD.cardSize()/1048576.0f);
  sdCreateFolderTree();
}

// ================================================================
//  DIAGNOSTIC LOG — writes to /tricorder/config/diag.log
//  Always-on rolling log of important events for debugging.
//  Upload this file when asking for help.
// ================================================================
void diagLog(const char* tag, const char* fmt, ...) {
  if(!sys.sdMounted) return;
  File f=SD.open("/tricorder/config/diag.log",FILE_APPEND);
  if(!f) return;
  char msg[256];
  va_list args;
  va_start(args,fmt);
  vsnprintf(msg,sizeof(msg),fmt,args);
  va_end(args);
  f.printf("[%lu ms][%s] %s\n",millis(),tag,msg);
  f.close();
  Serial.printf("[%lu ms][%s] %s\n",millis(),tag,msg);
}

void diagLogBoot() {
  if(!sys.sdMounted) return;
  File f=SD.open("/tricorder/config/diag.log",FILE_APPEND);
  if(!f) return;
  f.println("\n================================================================");
  f.printf("[BOOT] %lu ms  Tricorder v2.9 diagnostic session\n",millis());
  f.printf("  PSRAM: %s (%.0f KB free)\n",
    psramFound()?"OK":"NOT FOUND",
    psramFound()?ESP.getFreePsram()/1024.0f:0);
  f.printf("  SD:    cardType=%d  size=%.0f MB\n",
    SD.cardType(), SD.cardSize()/1048576.0f);
  f.printf("  Heap free: %u bytes\n",ESP.getFreeHeap());
  f.println("================================================================");
  f.close();
}

void sdWriteSnapshot() {
  Serial.printf("[SD] Snapshot attempt — sdMounted=%d  mode=%s\n",
    sys.sdMounted?1:0, MODE_NAMES[sys.currentMode]);
  if(!sys.sdMounted){
    tft.fillRect(60,0,100,TFT_HEADER_H,TFT_DARK_TEAL);
    tft.setTextSize(1);tft.setTextColor(ST77XX_RED);
    tft.setCursor(62,5);tft.print("SD ERR [FRZ]");
    return;
  }

  // Route snapshot to the correct subfolder based on current mode.
  // Also writes a master log at /logs/log.csv for backward compat.
  char modePath[48];
  char modeHeader[200];
  char modeRow[256];

  switch(sys.currentMode) {
    case MODE_ENVIRONMENTAL:
      strcpy(modePath,"/tricorder/snapshots/env/env.csv");
      strcpy(modeHeader,"timestamp_ms,temp_c,humidity,pressure_hpa,gas_kohm,uv_raw,uvi");
      snprintf(modeRow,sizeof(modeRow),"%lu,%.1f,%.1f,%.1f,%.2f,%.0f,%.3f",
        millis(),envData.temperature,envData.humidity,envData.pressure,
        envData.gasKOhm,envData.uvRaw,envData.uvIndex);
      break;
    case MODE_MEASURE:
      strcpy(modePath,"/tricorder/snapshots/measure/measure.csv");
      strcpy(modeHeader,"timestamp_ms,dist_mm,heading_deg,mag_x,mag_y,mag_z,colorR,colorG,colorB,colorIR");
      snprintf(modeRow,sizeof(modeRow),"%lu,%d,%.1f,%.2f,%.2f,%.2f,%lu,%lu,%lu,%lu",
        millis(),measureData.distanceMM,measureData.heading,
        measureData.magX,measureData.magY,measureData.magZ,
        measureData.colorR,measureData.colorG,measureData.colorB,measureData.colorIR);
      break;
    case MODE_THERMAL:
      strcpy(modePath,"/tricorder/snapshots/thermal/thermal.csv");
      strcpy(modeHeader,"timestamp_ms,center_c,min_c,max_c,delta_c,pixels_64");
      {
        snprintf(modeRow,sizeof(modeRow),"%lu,%.1f,%.1f,%.1f,%.1f",
          millis(),thermalData.centerTemp,thermalData.minTemp,
          thermalData.maxTemp,thermalData.deltaTemp);
        // Append all 64 pixel values
        for(int i=0;i<64;i++){
          char px[8];snprintf(px,sizeof(px),",%.1f",thermalData.pixels[i]);
          strncat(modeRow,px,sizeof(modeRow)-strlen(modeRow)-1);
        }
      }
      break;
    case MODE_SOUND:
      strcpy(modePath,"/tricorder/snapshots/sound/sound.csv");
      strcpy(modeHeader,"timestamp_ms,rms,peak");
      snprintf(modeRow,sizeof(modeRow),"%lu,%.4f,%.4f",
        millis(),soundData.rmsLevel,soundData.peakLevel);
      break;
    case MODE_WIRELESS:
      strcpy(modePath,"/tricorder/snapshots/wifi/all/wifi.csv");
      strcpy(modeHeader,"timestamp_ms,ssid,rssi_dbm,channel");
      {
        bool isNew=!SD.exists(modePath);
        File f=SD.open(modePath,FILE_APPEND);
        if(!f){Serial.println("[SD] Wireless snapshot open failed");goto writeMaster;}
        if(isNew) f.println(modeHeader);
        for(int i=0;i<wirelessData.wifiCount;i++){
          char row[128];
          snprintf(row,sizeof(row),"%lu,%s,%d,%d",
            millis(),wirelessData.wifiSSID[i],wirelessData.wifiRSSI[i],wirelessData.wifiChannel[i]);
          f.println(row);Serial.println(row);
        }
        f.close();
      }
      goto writeMaster; // skip single-row write below
    default:
      // BIO/FILES — just write master log
      strcpy(modePath,"");
      strcpy(modeHeader,"");
      strcpy(modeRow,"");
      goto writeMaster;
  }

  // Write mode-specific CSV
  {
    bool isNew=!SD.exists(modePath);
    File f=SD.open(modePath,FILE_APPEND);
    if(!f){Serial.printf("[SD] Open failed: %s\n",modePath);goto writeMaster;}
    if(isNew) f.println(modeHeader);
    f.println(modeRow);f.close();
    Serial.printf("[SD] → %s\n%s\n",modePath,modeRow);
  }

  writeMaster:
  // Also append everything to the master log for easy export
  {
    bool isNew=!SD.exists("/logs/log.csv");
    File f=SD.open("/logs/log.csv",FILE_APPEND);
    if(!f){Serial.println("[SD] Master log open failed");return;}
    if(isNew)
      f.println("timestamp_ms,mode,temp_c,humidity,pressure_hpa,gas_kohm,"
                "uv_raw,uvi,dist_mm,heading,colorR,colorG,colorB,"
                "center_temp,min_temp,max_temp,rms");
    char row[300];
    snprintf(row,sizeof(row),
      "%lu,%s,%.1f,%.1f,%.1f,%.2f,%.0f,%.3f,%d,%.1f,%lu,%lu,%lu,%.1f,%.1f,%.1f,%.4f",
      millis(),MODE_NAMES[sys.currentMode],
      envData.temperature,envData.humidity,envData.pressure,
      envData.gasKOhm,envData.uvRaw,envData.uvIndex,
      measureData.distanceMM,measureData.heading,
      measureData.colorR,measureData.colorG,measureData.colorB,
      thermalData.centerTemp,thermalData.minTemp,thermalData.maxTemp,
      soundData.rmsLevel);
    f.println(row);f.close();
    Serial.printf("[SD] Master: %s\n",row);
  }
}

// ================================================================
//  WAV RECORDING
// ================================================================
void writeWavHeader(File& f, uint32_t dataSize) {
  const uint32_t sampleRate=22050;
  const uint16_t numCh=1,bps=16;
  uint32_t byteRate=sampleRate*numCh*bps/8;
  uint16_t blockAlign=numCh*bps/8;
  uint32_t chunkSize=36+dataSize;
  f.seek(0);
  f.write((const uint8_t*)"RIFF",4); f.write((uint8_t*)&chunkSize,4);
  f.write((const uint8_t*)"WAVE",4); f.write((const uint8_t*)"fmt ",4);
  uint32_t sub1=16; f.write((uint8_t*)&sub1,4);
  uint16_t afmt=1;  f.write((uint8_t*)&afmt,2);
  f.write((uint8_t*)&numCh,2); f.write((uint8_t*)&sampleRate,4);
  f.write((uint8_t*)&byteRate,4); f.write((uint8_t*)&blockAlign,2);
  f.write((uint8_t*)&bps,2);
  f.write((const uint8_t*)"data",4); f.write((uint8_t*)&dataSize,4);
}

void sdStartRecording() {
  if(!sys.sdMounted){Serial.println("[REC] SD not mounted");return;}
  char path[40];
  do{ snprintf(path,sizeof(path),"/tricorder/audio/rec%04d.wav",recIndex++); }
  while(SD.exists(path)&&recIndex<9999);
  recFile=SD.open(path,FILE_WRITE);
  if(!recFile){Serial.printf("[REC] Open failed: %s\n",path);return;}
  recByteCount=0;
  recStartMs=millis();
  writeWavHeader(recFile,0);
  soundRecording=true;
  Serial.printf("[REC] Start → %s (max %ds)\n",path,MAX_REC_SECONDS);
  tftDrawHeader();
  beep(1000,40);delay(20);beep(1400,60);
}

void sdStopRecording() {
  if(!soundRecording) return;
  soundRecording=false;
  writeWavHeader(recFile,recByteCount);
  recFile.close();
  Serial.printf("[REC] Stopped. %lu bytes = %.1fs\n",recByteCount,recByteCount/44100.0f);
  tftDrawHeader();
  beep(1400,60);delay(20);beep(800,80);
}

// ================================================================
//  FILE BROWSER
// ================================================================
void filesGoUp() {
  int len=strlen(filesData.currentPath);
  if(len<=1) return;
  for(int i=len-1;i>=0;i--){
    if(filesData.currentPath[i]=='/'){
      if(i==0) filesData.currentPath[1]='\0';
      else     filesData.currentPath[i]='\0';
      break;
    }
  }
  filesData.needsRefresh=true;sys.filesScrollIdx=0;
}

void filesReadDir() {
  filesData.count=0;
  if(!sys.sdMounted) return;
  File dir=SD.open(filesData.currentPath);
  if(!dir||!dir.isDirectory()){Serial.printf("[FILES] Cannot open: %s\n",filesData.currentPath);return;}
  if(strcmp(filesData.currentPath,"/")!=0&&strcmp(filesData.currentPath,"")!=0){
    strcpy(filesData.entries[filesData.count].name,"..");
    filesData.entries[filesData.count].isDir=true;
    filesData.entries[filesData.count].size=0;
    filesData.entries[filesData.count].valid=true;
    filesData.count++;
  }
  while(filesData.count<MAX_FILES){
    File entry=dir.openNextFile();if(!entry) break;
    String fname=entry.name();
    if(fname.startsWith(".")&&fname.length()>1){entry.close();continue;}
    int ls=fname.lastIndexOf('/');if(ls>=0) fname=fname.substring(ls+1);
    if(fname.length()==0){entry.close();continue;}
    strncpy(filesData.entries[filesData.count].name,fname.c_str(),32);
    filesData.entries[filesData.count].name[32]='\0';
    filesData.entries[filesData.count].isDir=entry.isDirectory();
    filesData.entries[filesData.count].size=entry.size();
    filesData.entries[filesData.count].valid=true;
    filesData.count++;entry.close();
  }
  dir.close();
  Serial.printf("[FILES] %d entries in %s\n",filesData.count,filesData.currentPath);
  filesData.needsRefresh=false;
}

void filesDrillIn(int idx) {
  if(idx<0||idx>=filesData.count) return;
  FileEntry* e=&filesData.entries[idx];
  if(!e->valid||!e->isDir) return;
  if(strcmp(e->name,"..")==0){filesGoUp();return;}
  int len=strlen(filesData.currentPath);
  if(len>0&&filesData.currentPath[len-1]!='/') strcat(filesData.currentPath,"/");
  strncat(filesData.currentPath,e->name,63-len);
  filesData.needsRefresh=true;sys.filesScrollIdx=0;
  Serial.printf("[FILES] → %s\n",filesData.currentPath);
}

// ================================================================
//  MUX
// ================================================================
void switchMux(uint8_t ch) {
  Wire.beginTransmission(MUX_ADDR);Wire.write(1<<ch);Wire.endTransmission();
  delayMicroseconds(150);
}
void muxDisable() {
  Wire.beginTransmission(MUX_ADDR);Wire.write(0x00);Wire.endTransmission();
}

// I2C bus recovery — call if reads start failing.
// Re-initialises Wire and pulls SCL high 9 times to clear any
// stuck slave that's holding SDA low after an aborted transaction.
void i2cRecover() {
  Wire.end();
  pinMode(I2C_SCL,OUTPUT);
  for(int i=0;i<9;i++){digitalWrite(I2C_SCL,HIGH);delayMicroseconds(5);digitalWrite(I2C_SCL,LOW);delayMicroseconds(5);}
  digitalWrite(I2C_SCL,HIGH);
  Wire.begin(I2C_SDA,I2C_SCL);Wire.setClock(100000);
  diagLog("I2C","Bus recovered");
}

// ================================================================
//  APDS9999 NATIVE DRIVER
//  Talks directly at 0x52 — Adafruit_APDS9960 lib was wrong chip.
//  Caller is responsible for selecting CH_APDS on the mux first.
// ================================================================
bool apdsRead(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(APDS_ADDR);
  Wire.write(reg);
  if(Wire.endTransmission(true)!=0) return false;
  Wire.requestFrom(APDS_ADDR,len);
  for(uint8_t i=0;i<len;i++){
    if(!Wire.available()) return false;
    buf[i]=Wire.read();
  }
  return true;
}
bool apdsWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(APDS_ADDR);
  Wire.write(reg);Wire.write(val);
  return Wire.endTransmission()==0;
}

// Initialise APDS9999. Returns true if PART_ID matches expected (0xC2).
bool apdsBegin() {
  uint8_t partId=0;
  if(!apdsRead(APDS_PART_ID,&partId,1)) return false;
  if(partId!=0xC2) {
    Serial.printf("[APDS] Unexpected PART_ID 0x%02X (expected 0xC2)\n",partId);
    // Still return true — chip is responding, may be a variant
  }
  // LS_GAIN=0x01 (3x), LS_MEAS_RATE=0x22 (18-bit res, 100ms),
  // MAIN_CTRL=0x06 (LS_EN + RGB mode)
  apdsWrite(APDS_LS_GAIN,0x01);
  apdsWrite(APDS_LS_MEAS_RATE,0x22);
  apdsWrite(APDS_MAIN_CTRL,0x06);
  delay(120);
  return true;
}

// Read all four channels. Each is 20-bit (3 bytes, little-endian, top nibble masked).
bool apdsReadColor(uint32_t* r, uint32_t* g, uint32_t* b, uint32_t* ir) {
  uint8_t buf[12];
  if(!apdsRead(APDS_LS_DATA_IR,buf,12)) return false;
  *ir=(uint32_t)buf[0]|((uint32_t)buf[1]<<8)|((uint32_t)(buf[2]&0x0F)<<16);
  *g =(uint32_t)buf[3]|((uint32_t)buf[4]<<8)|((uint32_t)(buf[5]&0x0F)<<16);
  *b =(uint32_t)buf[6]|((uint32_t)buf[7]<<8)|((uint32_t)(buf[8]&0x0F)<<16);
  *r =(uint32_t)buf[9]|((uint32_t)buf[10]<<8)|((uint32_t)(buf[11]&0x0F)<<16);
  return true;
}

// ================================================================
//  THERMAL
// ================================================================
#define INTERP_SIZE  16
#define CELL_W       10
#define CELL_H       5
#define THERMAL_Y    TFT_DATA_Y
#define THERMAL_STATS_Y (THERMAL_Y + INTERP_SIZE*CELL_H + 2)

float bilinearSample(float* src,float x,float y) {
  int x0=(int)x,y0=(int)y,x1=min(x0+1,7),y1=min(y0+1,7);
  float fx=x-x0,fy=y-y0;
  return src[y0*8+x0]*(1-fx)*(1-fy)+src[y0*8+x1]*fx*(1-fy)
        +src[y1*8+x0]*(1-fx)*fy    +src[y1*8+x1]*fx*fy;
}
void computeBilinear() {
  for(int row=0;row<INTERP_SIZE;row++)
    for(int col=0;col<INTERP_SIZE;col++){
      float sx=(INTERP_SIZE-1-row)*7.0f/(INTERP_SIZE-1);
      float sy=col*7.0f/(INTERP_SIZE-1);
      thermalData.interp[row*INTERP_SIZE+col]=bilinearSample(thermalData.pixels,sx,sy);
    }
}
// Nonlinear absolute thermal mapping — livable range 15-40C gets 80%
// of the full colour gradient. Outside that range is compressed.
// Approximate colour landmarks at your ambient (Laredo, TX):
//   ~15C = blue   ~23C = green (room)   ~37C = orange (body)   ~45C+ = red
uint16_t tempToAbsColor565(float t) {
  float n;
  if      (t <= 10.0f) n = 0.00f;
  else if (t <= 15.0f) n = 0.00f + (t-10.0f)/5.0f  * 0.05f;  // slow ramp in cold
  else if (t <= 40.0f) n = 0.05f + (t-15.0f)/25.0f * 0.80f;  // 15-40C = full gradient
  else if (t <= 60.0f) n = 0.85f + (t-40.0f)/20.0f * 0.10f;  // compress hot
  else if (t <= 80.0f) n = 0.95f + (t-60.0f)/20.0f * 0.05f;
  else                 n = 1.00f;
  n = constrain(n, 0.0f, 1.0f);
  uint8_t r,g,b;
  if      (n < 0.25f) { float f=n/0.25f;         r=0;              g=(uint8_t)(f*255);     b=255; }
  else if (n < 0.50f) { float f=(n-0.25f)/0.25f; r=0;              g=255;                  b=(uint8_t)((1-f)*255); }
  else if (n < 0.75f) { float f=(n-0.50f)/0.25f; r=(uint8_t)(f*255); g=255;               b=0; }
  else                { float f=(n-0.75f)/0.25f; r=255;            g=(uint8_t)((1-f)*255); b=0; }
  return tft.color565(r,g,b);
}
uint16_t tempToRelColor565(float t,float mn,float mx) {
  float n=(mx-mn<0.1f)?0.5f:constrain((t-mn)/(mx-mn),0,1);
  uint8_t r,g,b;
  if(n<0.25f)      {float f=n/0.25f;          r=0;g=(uint8_t)(f*255);b=255;}
  else if(n<0.5f)  {float f=(n-0.25f)/0.25f;  r=0;g=255;b=(uint8_t)((1-f)*255);}
  else if(n<0.75f) {float f=(n-0.5f)/0.25f;   r=(uint8_t)(f*255);g=255;b=0;}
  else             {float f=(n-0.75f)/0.25f;   r=255;g=(uint8_t)((1-f)*255);b=0;}
  return tft.color565(r,g,b);
}
void tftDrawThermal() {
  computeBilinear();
  for(int row=0;row<INTERP_SIZE;row++)
    for(int col=0;col<INTERP_SIZE;col++){
      float t=thermalData.interp[row*INTERP_SIZE+col];
      uint16_t c=sys.thermalAbsolute?tempToAbsColor565(t):tempToRelColor565(t,thermalData.minTemp,thermalData.maxTemp);
      tft.fillRect(col*CELL_W,THERMAL_Y+row*CELL_H,CELL_W,CELL_H,c);
    }
  tft.fillRect(0,THERMAL_STATS_Y,160,26,ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_CYAN);  tft.setCursor(2,THERMAL_STATS_Y+2);  tft.printf("Ctr:%.1fC",thermalData.centerTemp);
  tft.setTextColor(ST77XX_YELLOW);tft.setCursor(2,THERMAL_STATS_Y+14); tft.printf("Mode:%s",sys.thermalAbsolute?"ABS":"REL");
  tft.setTextColor(ST77XX_GREEN); tft.setCursor(80,THERMAL_STATS_Y+2); tft.printf("D:%.1fC",thermalData.deltaTemp);
  tft.setTextColor(ST77XX_WHITE); tft.setCursor(80,THERMAL_STATS_Y+14);tft.printf("%.1f-%.1f",thermalData.minTemp,thermalData.maxTemp);
}

// ================================================================
//  SOUND MODE — oscilloscope + stats
//
//  Layout (TFT 160×128, header takes 0..18):
//    y=22..88   live waveform, 66px tall, centered on y=55
//    y=92..127  three rows of stats (RMS / Peak / Freq + timer/bytes)
//
//  We keep a previous-frame copy of the waveform so we can erase it
//  pixel-by-pixel before redrawing — full-screen clear at 30fps would
//  flicker badly. The stats area is small enough to clear+redraw each
//  frame without flicker.
// ================================================================
#define SCOPE_Y       22
#define SCOPE_H       66
#define SCOPE_MID_Y   (SCOPE_Y + SCOPE_H/2)
#define SCOPE_W       160
#define SCOPE_STATS_Y 92

void tftDrawSoundScope() {
  static uint8_t prevYs[SCOPE_W];   // last frame's y-position per column

  // First entry / mode change: paint background and reset history.
  // Piggy-backs on tftUpdate's labelsDrawn=false signal which is set
  // any time the screen has been cleared by a mode change.
  if(!sys.labelsDrawn) {
    tft.fillRect(0, SCOPE_Y, SCOPE_W, SCOPE_H, ST77XX_BLACK);
    tft.drawFastHLine(0, SCOPE_MID_Y, SCOPE_W, 0x0841);
    for(int i = 0; i < SCOPE_W; i++) prevYs[i] = SCOPE_MID_Y;
    sys.labelsDrawn = true;
  }

  // Draw waveform — sample buffer is 1024 samples wide, scope is 160px,
  // so step = 1024/160 ≈ 6.4 samples per column. We pick the first sample
  // of each column for speed (good enough at 30fps).
  const int   DISP_LEN = sizeof(soundData.buffer) / sizeof(int16_t);
  const float step     = (float)DISP_LEN / SCOPE_W;
  const float HALF_H   = (SCOPE_H / 2 - 2);
  const float GAIN     = 4.0f;   // visual gain on top of mic gain

  for(int x = 0; x < SCOPE_W; x++) {
    int idx = (int)(x * step);
    if(idx >= DISP_LEN) idx = DISP_LEN - 1;
    float s = (soundData.buffer[idx] / 32768.0f) * GAIN;
    if(s >  1.0f) s =  1.0f;
    if(s < -1.0f) s = -1.0f;
    int y = SCOPE_MID_Y - (int)(s * HALF_H);
    // Erase previous pixel at this column (also redraw center line if we
    // happen to overlap it — cheaper than testing)
    int py = prevYs[x];
    if(py != y) {
      tft.drawPixel(x, py, (py == SCOPE_MID_Y) ? 0x0841 : ST77XX_BLACK);
    }
    tft.drawPixel(x, y, ST77XX_CYAN);
    prevYs[x] = y;
  }

  // ---- Stats area ----
  // Row 1: RMS + Peak (left) | Freq estimate (right)
  // Row 2: Recording state + timer/bytes
  // Row 3: Disk space hint or dB-style level meter
  tft.fillRect(0, SCOPE_STATS_Y, SCOPE_W, 128 - SCOPE_STATS_Y, ST77XX_BLACK);
  tft.setTextSize(1);

  // Row 1: RMS / Peak / Freq
  tft.setTextColor(ST77XX_GREEN);  tft.setCursor(2,  SCOPE_STATS_Y + 2);
  tft.printf("RMS:%.2f", soundData.rmsLevel);
  tft.setTextColor(ST77XX_YELLOW); tft.setCursor(64, SCOPE_STATS_Y + 2);
  tft.printf("Pk:%.2f", soundData.peakLevel);
  tft.setTextColor(ST77XX_CYAN);   tft.setCursor(118, SCOPE_STATS_Y + 2);
  tft.printf("%dHz", (int)soundDomFreqHz);

  // Row 2: recording state
  tft.setCursor(2, SCOPE_STATS_Y + 14);
  if(soundRecording) {
    unsigned long secs = (millis() - recStartMs) / 1000;
    unsigned long mins = secs / 60;
    secs = secs % 60;
    tft.setTextColor(ST77XX_RED);
    tft.printf("REC %lu:%02lu", mins, secs);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(78, SCOPE_STATS_Y + 14);
    if(recByteCount < 1024)
      tft.printf("%lu B", recByteCount);
    else if(recByteCount < 1024UL*1024UL)
      tft.printf("%lu KB", recByteCount / 1024);
    else
      tft.printf("%.1f MB", recByteCount / 1048576.0f);
  } else {
    tft.setTextColor(0x528A);
    tft.print("idle");
    tft.setTextColor(0x528A);
    tft.setCursor(78, SCOPE_STATS_Y + 14);
    tft.printf("max %dm", MAX_REC_SECONDS / 60);
  }

  // Row 3: simple amplitude bar — 30 cols wide, scales with peak
  int barW = constrain((int)(soundData.peakLevel * 156), 0, 156);
  tft.drawRect(2, SCOPE_STATS_Y + 26, 156, 4, 0x2104);
  tft.fillRect(3, SCOPE_STATS_Y + 27, barW > 1 ? barW - 1 : 0, 2,
    soundData.peakLevel > 0.85f ? ST77XX_RED :
    soundData.peakLevel > 0.5f  ? TFT_ORANGE :
                                  ST77XX_GREEN);
  if(barW < 154) {
    tft.fillRect(3 + (barW > 0 ? barW : 0), SCOPE_STATS_Y + 27,
                 154 - (barW > 0 ? barW : 0), 2, ST77XX_BLACK);
  }
}

// ================================================================
//  TFT HELPERS
// ================================================================
void tftValClear(uint8_t line){tft.fillRect(VAL_X-2,TFT_DATA_Y+line*17,VAL_W+2,11,ST77XX_BLACK);}
void tftLabel(uint8_t line,const char* label){tft.setTextSize(1);tft.setTextColor(ST77XX_WHITE);tft.setCursor(4,TFT_DATA_Y+line*17);tft.print(label);}
void tftVal(uint8_t line,uint16_t color,float val,const char* unit){tftValClear(line);tft.setTextSize(1);tft.setTextColor(color);tft.setCursor(VAL_X,TFT_DATA_Y+line*17);tft.print(val,1);tft.print(unit);}
void tftValInt(uint8_t line,uint16_t color,int val,const char* unit){tftValClear(line);tft.setTextSize(1);tft.setTextColor(color);tft.setCursor(VAL_X,TFT_DATA_Y+line*17);tft.print(val);tft.print(unit);}
void tftValStr(uint8_t line,uint16_t color,const char* str){tftValClear(line);tft.setTextSize(1);tft.setTextColor(color);tft.setCursor(VAL_X,TFT_DATA_Y+line*17);tft.print(str);}

void tftDrawHeader() {
  tft.fillRect(0,0,TFT_W,TFT_HEADER_H,TFT_DARK_TEAL);
  tft.setTextSize(1);tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(4,5);tft.print("MODE: ");tft.print(MODE_SHORT[sys.currentMode]);
  if(measureData.magCalibrated){tft.setTextColor(ST77XX_GREEN); tft.print(" CAL");}
  if(sys.snapshotFrozen)       {tft.setTextColor(ST77XX_YELLOW);tft.print(" [FRZ]");}
  if(soundRecording)           {tft.setTextColor(ST77XX_RED);   tft.print(" [REC]");}
  // WiFi status indicator
  if(wifiSt.mode == WIFI_AP_ACTIVE || wifiSt.mode == WIFI_AP_SETUP) {
    tft.setTextColor(ST77XX_GREEN); tft.print(" [AP]");
  } else if(wifiSt.mode == WIFI_STA_ACTIVE) {
    tft.setTextColor(ST77XX_GREEN); tft.print(" [WiFi]");
  } else if(wifiSt.mode == WIFI_STA_CONNECTING) {
    tft.setTextColor(TFT_ORANGE);   tft.print(" [...]");
  }
}

void tftDrawLabels() {
  tft.fillRect(0,TFT_DATA_Y,TFT_W,TFT_H-TFT_DATA_Y,ST77XX_BLACK);
  switch(sys.currentMode){
    case MODE_ENVIRONMENTAL:
      tftLabel(0,"Temp");tftLabel(1,"Humidity");tftLabel(2,"Pressure");tftLabel(3,"Gas");tftLabel(4,"UV");break;
    case MODE_MEASURE:
      tftLabel(0,"Dist");tftLabel(1,"Heading");tftLabel(2,"Mag X");tftLabel(3,"Mag Y");tftLabel(4,"Color");
      if(!measureData.magCalibrated){tft.setTextSize(1);tft.setTextColor(TFT_ORANGE);tft.setCursor(4,TFT_DATA_Y+5*17);tft.print("TOP=cal mag");}
      break;
    case MODE_SOUND:
      // Sound mode draws its own waveform-centric layout — no labels here
      break;
    case MODE_WIRELESS:case MODE_THERMAL:case MODE_FILES:break;
    case MODE_BIO:tft.setTextColor(ST77XX_YELLOW);tft.setCursor(4,TFT_DATA_Y);tft.print("Connect Bio Module");break;
  }
  sys.labelsDrawn=true;
}

// ================================================================
//  WIRELESS LIST
// ================================================================
void tftDrawWirelessList() {
  tft.fillRect(0,TFT_DATA_Y,TFT_W,TFT_H-TFT_DATA_Y,ST77XX_BLACK);
  if(sys.wirelessShowDetail){
    int i=sys.wirelessDetailIdx;tft.setTextSize(1);
    if(sys.wirelessShowBLE){
      tft.setTextColor(ST77XX_CYAN);tft.setCursor(4,TFT_DATA_Y);tft.print("BLE Device:");
      tft.setTextColor(ST77XX_WHITE);tft.setCursor(4,TFT_DATA_Y+11);tft.print(wirelessData.bleName[i][0]?wirelessData.bleName[i]:"(unnamed)");
      tft.setTextColor(ST77XX_CYAN);tft.setCursor(4,TFT_DATA_Y+25);tft.print("Address:");
      tft.setTextColor(ST77XX_WHITE);tft.setCursor(4,TFT_DATA_Y+36);tft.print(wirelessData.bleAddress[i]);
      tft.setTextColor(ST77XX_CYAN);tft.setCursor(4,TFT_DATA_Y+50);tft.print("Signal:");
      tft.setTextColor(ST77XX_GREEN);tft.setCursor(4,TFT_DATA_Y+61);tft.printf("%d dBm",wirelessData.bleRSSI[i]);
    } else {
      tft.setTextColor(ST77XX_CYAN);tft.setCursor(4,TFT_DATA_Y);tft.print("SSID:");
      tft.setTextColor(ST77XX_WHITE);tft.setCursor(4,TFT_DATA_Y+11);tft.print(wirelessData.wifiSSID[i]);
      tft.setTextColor(ST77XX_CYAN);tft.setCursor(4,TFT_DATA_Y+25);tft.print("Signal:");
      tft.setTextColor(ST77XX_GREEN);tft.setCursor(4,TFT_DATA_Y+36);tft.printf("%d dBm",wirelessData.wifiRSSI[i]);
      tft.setTextColor(ST77XX_CYAN);tft.setCursor(4,TFT_DATA_Y+50);tft.print("Channel:");
      tft.setTextColor(ST77XX_WHITE);tft.setCursor(4,TFT_DATA_Y+61);tft.print(wirelessData.wifiChannel[i]);
    }
    tft.setTextColor(ST77XX_YELLOW);tft.setCursor(4,TFT_DATA_Y+82);tft.print("[click] back to list");
    return;
  }
  if(wirelessData.scanning){tft.setTextColor(ST77XX_CYAN);tft.setCursor(4,TFT_DATA_Y+14);tft.print("SCANNING...");return;}
  int total=sys.wirelessShowBLE?min(wirelessData.bleCount,10):min(wirelessData.wifiCount,10);
  int* scrollIdx=sys.wirelessShowBLE?&sys.bleScrollIdx:&sys.wifiScrollIdx;
  int totalPages=(total==0)?1:(total+ITEMS_PER_PAGE-1)/ITEMS_PER_PAGE;
  int currentPage=(*scrollIdx)/ITEMS_PER_PAGE;
  tft.setTextSize(1);tft.setTextColor(ST77XX_WHITE);tft.setCursor(4,TFT_DATA_Y);
  if(sys.wirelessShowBLE) tft.printf("BLE:%d",wirelessData.bleCount);
  else                    tft.printf("WiFi:%d",wirelessData.wifiCount);
  tft.setTextColor(ST77XX_CYAN);tft.setCursor(80,TFT_DATA_Y);tft.printf("[%s]",sys.wirelessShowBLE?"BLE":"WiFi");
  tft.setCursor(130,TFT_DATA_Y);tft.printf("%d/%d",currentPage+1,totalPages);
  if(total==0){tft.setTextColor(ST77XX_YELLOW);tft.setCursor(4,TFT_DATA_Y+20);tft.print("Hold TOP to scan");return;}
  int pageStart=currentPage*ITEMS_PER_PAGE;
  for(int i=0;i<ITEMS_PER_PAGE;i++){
    int idx=pageStart+i;if(idx>=total) break;
    int y=TFT_DATA_Y+14+i*26;bool selected=(idx==*scrollIdx);
    if(selected) tft.fillRect(0,y-1,TFT_W,24,0x2104);
    if(sys.wirelessShowBLE){
      int rssi=wirelessData.bleRSSI[idx];uint16_t rc=(rssi>-60)?ST77XX_GREEN:(rssi>-75)?TFT_ORANGE:ST77XX_RED;
      tft.setTextColor(rc);tft.setCursor(4,y);tft.printf("%ddBm",rssi);
      tft.setTextColor(selected?ST77XX_CYAN:ST77XX_WHITE);tft.setCursor(56,y);
      char name[17]={0};strncpy(name,wirelessData.bleName[idx],16);if(!name[0])strcpy(name,"(unnamed)");tft.print(name);
      tft.setTextColor(0x7BEF);tft.setCursor(56,y+11);char addr[13]={0};strncpy(addr,wirelessData.bleAddress[idx],12);tft.print(addr);
    } else {
      int rssi=wirelessData.wifiRSSI[idx];uint16_t rc=(rssi>-60)?ST77XX_GREEN:(rssi>-75)?TFT_ORANGE:ST77XX_RED;
      tft.setTextColor(rc);tft.setCursor(4,y);tft.printf("%ddBm",rssi);
      tft.setTextColor(selected?ST77XX_CYAN:ST77XX_WHITE);tft.setCursor(56,y);
      char ssid[17]={0};strncpy(ssid,wirelessData.wifiSSID[idx],16);tft.print(ssid);
      tft.setTextColor(0x7BEF);tft.setCursor(56,y+11);tft.printf("ch%d",wirelessData.wifiChannel[idx]);
    }
  }
  tft.setTextColor(0x528A);tft.setCursor(4,TFT_H-10);tft.print("Hold TOP=scan  Tap=WiFi/BLE");
}

// ================================================================
//  FILES LIST
// ================================================================
void tftDrawFilesList() {
  tft.fillRect(0,TFT_DATA_Y,TFT_W,TFT_H-TFT_DATA_Y,ST77XX_BLACK);
  if(sys.filesShowDetail){
    int i=sys.filesDetailIdx;if(i>=filesData.count){sys.filesShowDetail=false;return;}
    FileEntry* e=&filesData.entries[i];tft.setTextSize(1);
    tft.setTextColor(ST77XX_CYAN);tft.setCursor(4,TFT_DATA_Y);tft.print(e->isDir?"[DIR]":"[FILE]");
    tft.setTextColor(ST77XX_WHITE);tft.setCursor(4,TFT_DATA_Y+14);tft.print(e->name);
    if(!e->isDir){
      tft.setTextColor(ST77XX_CYAN);tft.setCursor(4,TFT_DATA_Y+30);tft.print("Size:");
      tft.setTextColor(ST77XX_WHITE);tft.setCursor(4,TFT_DATA_Y+42);
      if(e->size<1024) tft.printf("%d bytes",e->size);
      else if(e->size<1048576) tft.printf("%.1f KB",e->size/1024.0f);
      else tft.printf("%.1f MB",e->size/1048576.0f);
    }
    tft.setTextColor(ST77XX_YELLOW);tft.setCursor(4,TFT_DATA_Y+82);tft.print("[click] back");
    return;
  }
  int total=filesData.count;
  int totalPages=(total==0)?1:(total+ITEMS_PER_PAGE-1)/ITEMS_PER_PAGE;
  int currentPage=sys.filesScrollIdx/ITEMS_PER_PAGE;
  tft.setTextSize(1);tft.setTextColor(ST77XX_CYAN);tft.setCursor(4,TFT_DATA_Y);
  char dispPath[20];strncpy(dispPath,filesData.currentPath,19);dispPath[19]='\0';tft.print(dispPath);
  tft.setCursor(130,TFT_DATA_Y);tft.printf("%d/%d",currentPage+1,totalPages);
  if(total==0){tft.setTextColor(ST77XX_RED);tft.setCursor(4,TFT_DATA_Y+20);tft.print("Empty or Error");return;}
  int pageStart=currentPage*ITEMS_PER_PAGE;
  for(int i=0;i<ITEMS_PER_PAGE;i++){
    int idx=pageStart+i;if(idx>=total) break;
    FileEntry* e=&filesData.entries[idx];if(!e->valid) continue;
    int y=TFT_DATA_Y+14+i*26;bool selected=(idx==sys.filesScrollIdx);
    if(selected) tft.fillRect(0,y-1,TFT_W,24,0x2104);
    tft.setTextColor(e->isDir?ST77XX_CYAN:ST77XX_WHITE);tft.setCursor(4,y);tft.print(e->isDir?"[D]":"[F]");
    tft.setTextColor(selected?ST77XX_CYAN:ST77XX_WHITE);tft.setCursor(30,y);
    char dispName[17]={0};strncpy(dispName,e->name,16);tft.print(dispName);
    if(!e->isDir){
      tft.setTextColor(0x7BEF);tft.setCursor(30,y+11);
      if(e->size<1024) tft.printf("%dB",e->size);
      else if(e->size<1048576) tft.printf("%.1fK",e->size/1024.0f);
      else tft.printf("%.1fM",e->size/1048576.0f);
    }
  }
  tft.setTextColor(0x528A);tft.setCursor(4,TFT_H-10);tft.print("TOP=up  click=open");
}

// ================================================================
//  VALUE UPDATE
// ================================================================
void tftDrawValues() {
  switch(sys.currentMode){
    case MODE_ENVIRONMENTAL:
      tftVal(0,TFT_ORANGE,  envData.temperature,"C");
      tftVal(1,ST77XX_CYAN, envData.humidity,   "%");
      tftVal(2,ST77XX_WHITE,envData.pressure,   " hPa");
      tftVal(3,ST77XX_GREEN,envData.gasKOhm,    " kOhm");
      tftValClear(4);tft.setTextSize(1);tft.setTextColor(ST77XX_YELLOW);
      tft.setCursor(VAL_X,TFT_DATA_Y+4*17);tft.printf("%.0fr %.2f",envData.uvRaw,envData.uvIndex);
      // Matrix icon name hint at the very bottom
      tft.fillRect(0,TFT_H-10,TFT_W,10,ST77XX_BLACK);
      tft.setTextColor(0x528A);tft.setCursor(4,TFT_H-9);
      {
        const char* names[5]={"Matrix: Temp","Matrix: Humidity","Matrix: Pressure","Matrix: Gas","Matrix: UV"};
        tft.print(names[sys.envIconIdx]);
      }
      break;
    case MODE_MEASURE:
      if(measureData.distanceValid) tftValInt(0,ST77XX_GREEN,measureData.distanceMM," mm");
      else tftValStr(0,ST77XX_RED,"no target");
      tftValClear(1);tft.setCursor(VAL_X,TFT_DATA_Y+1*17);tft.setTextColor(ST77XX_CYAN);
      tft.printf("%.1f%s",measureData.heading,measureData.magCalibrated?"":" *");
      tftVal(2,ST77XX_WHITE,measureData.magX," uT");
      tftVal(3,ST77XX_WHITE,measureData.magY," uT");
      // Color swatch — APDS9999 20-bit values, normalized for display.
      // We find the max of R/G/B and scale so the brightest channel = 255.
      // This gives a reasonable on-screen colour regardless of overall light.
      tftValClear(4);
      {
        if(!measureData.apdsOk) {
          tft.fillRect(VAL_X,TFT_DATA_Y+4*17,18,10,ST77XX_BLACK);
          tft.drawRect(VAL_X,TFT_DATA_Y+4*17,18,10,ST77XX_RED);
          tft.setTextSize(1);tft.setTextColor(ST77XX_RED);
          tft.setCursor(VAL_X+22,TFT_DATA_Y+4*17);tft.print("ERR");
        } else {
          uint32_t R=measureData.colorR, G=measureData.colorG, B=measureData.colorB;
          uint32_t mx = R; if(G>mx) mx=G; if(B>mx) mx=B;
          uint8_t r8,g8,b8;
          if(mx<10) {  // basically dark
            r8=g8=b8=0;
          } else {
            r8=(uint8_t)((uint64_t)R*255/mx);
            g8=(uint8_t)((uint64_t)G*255/mx);
            b8=(uint8_t)((uint64_t)B*255/mx);
          }
          tft.fillRect(VAL_X,TFT_DATA_Y+4*17,18,10,tft.color565(r8,g8,b8));
          tft.drawRect(VAL_X,TFT_DATA_Y+4*17,18,10,ST77XX_WHITE);
          tft.setTextSize(1);tft.setTextColor(ST77XX_WHITE);
          tft.setCursor(VAL_X+22,TFT_DATA_Y+4*17);
          // Show normalized RGB so user can see relative balance
          tft.printf("%d %d %d",r8,g8,b8);
        }
      }
      // Matrix icon name hint
      tft.fillRect(0,TFT_H-10,TFT_W,10,ST77XX_BLACK);
      tft.setTextColor(0x528A);tft.setCursor(4,TFT_H-9);
      {
        const char* names[3]={"Matrix: Distance","Matrix: Heading","Matrix: Color"};
        tft.print(names[sys.measureIconIdx]);
      }
      break;
    case MODE_SOUND:   tftDrawSoundScope();break;
    case MODE_WIRELESS:tftDrawWirelessList();break;
    case MODE_THERMAL: tftDrawThermal();break;
    case MODE_FILES:   tftDrawFilesList();break;
    case MODE_BIO:break;
  }
}

// ================================================================
//  TFT UPDATE
// ================================================================
void tftUpdate() {
  bool modeChanged=(sys.currentMode!=sys.lastMode);
  if(modeChanged){
    if(soundRecording){Serial.println("[REC] Mode change — stopping.");sdStopRecording();}
    tft.fillScreen(ST77XX_BLACK);
    sys.lastMode=sys.currentMode;sys.labelsDrawn=false;
    sys.wirelessShowDetail=false;sys.filesShowDetail=false;
    if(sys.currentMode==MODE_FILES){
      strcpy(filesData.currentPath,"/tricorder");
      filesData.needsRefresh=true;sys.filesScrollIdx=0;
    }
  }
  if(wifiSt.mode != WIFI_OFF_STATE) {
    tftDrawHeader();
    wifiDrawTFT();
    return;
  }
  if(sys.currentMode==MODE_FILES&&filesData.needsRefresh) filesReadDir();
  tftDrawHeader();
  if(sys.currentMode==MODE_THERMAL) {tftDrawThermal();return;}
  if(sys.currentMode==MODE_WIRELESS){tftDrawWirelessList();return;}
  if(sys.currentMode==MODE_FILES)   {tftDrawFilesList();return;}
  if(sys.currentMode==MODE_SOUND)   {tftDrawSoundScope();return;}
  if(!sys.labelsDrawn) tftDrawLabels();
  tftDrawValues();
}

// ================================================================
//  SENSOR READS
// ================================================================
void readEnvironmental() {
  switchMux(CH_BME);
  if(bme.performReading()){
    envData.temperature=bme.temperature;envData.humidity=bme.humidity;
    envData.pressure=bme.pressure/100.0f;envData.gasKOhm=bme.gas_resistance/1000.0f;
  }
  switchMux(CH_LTR);
  uint32_t raw=ltr.readUVS();envData.uvRaw=(float)raw;envData.uvIndex=envData.uvRaw/6900.0f;
  envData.valid=true;
}

void readMeasure() {
  switchMux(CH_VL53);
  VL53L4CX_MultiRangingData_t rd;uint8_t ready=0;
  tof.VL53L4CX_GetMeasurementDataReady(&ready);
  if(ready){
    tof.VL53L4CX_GetMultiRangingData(&rd);
    if(rd.NumberOfObjectsFound>0){measureData.distanceMM=rd.RangeData[0].RangeMilliMeter;measureData.distanceValid=true;}
    tof.VL53L4CX_ClearInterruptAndStartMeasurement();
  }
  switchMux(CH_LIS);
  sensors_event_t e;lis2mdl.getEvent(&e);
  measureData.rawMagX=e.magnetic.x;measureData.rawMagY=e.magnetic.y;measureData.rawMagZ=e.magnetic.z;
  measureData.magX=e.magnetic.x-measureData.magOffX;
  measureData.magY=e.magnetic.y-measureData.magOffY;
  measureData.magZ=e.magnetic.z-measureData.magOffZ;
  measureData.heading=atan2f(measureData.magY,measureData.magX)*(180.0f/M_PI);
  if(measureData.heading<0) measureData.heading+=360.0f;
  switchMux(CH_APDS);
  // Native APDS9999 read — no Adafruit lib involved
  uint32_t r=0,g=0,b=0,ir=0;
  if(apdsReadColor(&r,&g,&b,&ir)) {
    // Apply white balance gains (calibrated against white paper under
    // your room lighting). After this, white reads R≈G≈B; pure red
    // shows R high, G/B low; etc.
    measureData.colorR  = (uint32_t)(r * WB_GAIN_R);
    measureData.colorG  = (uint32_t)(g * WB_GAIN_G);
    measureData.colorB  = (uint32_t)(b * WB_GAIN_B);
    measureData.colorIR = ir;
    measureData.colorC  = measureData.colorG; // luminance reference
    measureData.apdsOk  = true;
  } else {
    measureData.apdsOk=false;
  }
  // Mirror to legacy fields so snapshot CSV format doesn't change
  measureData.regColorR=measureData.colorR;
  measureData.regColorG=measureData.colorG;
  measureData.regColorB=measureData.colorB;
  measureData.regColorC=measureData.colorC;
  measureData.valid=true;
}

void readThermal() {
  switchMux(CH_AMG);amg.readPixels(thermalData.pixels);
  float mn=9999,mx=-9999;
  for(int i=0;i<64;i++){if(thermalData.pixels[i]<mn)mn=thermalData.pixels[i];if(thermalData.pixels[i]>mx)mx=thermalData.pixels[i];}
  thermalData.centerTemp=thermalData.pixels[27];thermalData.minTemp=mn;thermalData.maxTemp=mx;thermalData.deltaTemp=mx-mn;thermalData.valid=true;
  matrixSelect();
  matrix.setRotation(2);  // thermal was correct in old orientation — keep it
  matrixClear();
  for(int y=0;y<8;y++) for(int x=0;x<8;x++){float n=(thermalData.pixels[y*8+x]-mn)/max(0.001f,mx-mn);if(n>0.5f)matrix.drawPixel(x,y,LED_ON);}
  matrixWriteDisplay();
  matrix.setRotation(3);  // restore for other modes
  matrixDeselect();
}

// Drain whatever PCM samples are sitting in the I2S DMA ring. Called
// every main-loop iteration while in MODE_SOUND so we never lose audio
// to overflow during recording. Non-blocking — uses 0-tick timeout.
//
// Side effects:
//   - writes PCM to the open recording file (if recording)
//   - rolls soundData.buffer with most-recent samples for the display
//   - updates rmsLevel, peakLevel, soundDomFreqHz running stats
//   - auto-stops recording when MAX_REC_SECONDS reached
void readSoundFast() {
  // Local scratch (DMA buffer is 1024 samples × 2 bytes); read as much
  // as is available right now, return immediately if empty.
  static int16_t scratch[1024];
  size_t bytesRead = 0;
  esp_err_t r = i2s_read(I2S_NUM_0, scratch, sizeof(scratch), &bytesRead, 0);
  if(r != ESP_OK || bytesRead == 0) return;
  int samples = bytesRead / sizeof(int16_t);

  // Roll into soundData.buffer (used by waveform display) — keep most-recent
  // samples at the tail. soundData.buffer is 1024 samples; if we read fewer,
  // shift left and append.
  const int DISP_LEN = sizeof(soundData.buffer) / sizeof(int16_t);
  if(samples >= DISP_LEN) {
    memcpy(soundData.buffer, scratch + (samples - DISP_LEN), DISP_LEN * 2);
  } else {
    int keep = DISP_LEN - samples;
    memmove(soundData.buffer, soundData.buffer + samples, keep * 2);
    memcpy(soundData.buffer + keep, scratch, samples * 2);
  }

  // Stats (gain-corrected, clamped to [0,1])
  const float GAIN = 10.0f;
  float sumSq = 0, peak = 0;
  int   crossings = 0;
  int16_t prev = scratch[0];
  for(int i = 0; i < samples; i++) {
    float s = (scratch[i] / 32768.0f) * GAIN;
    sumSq += s * s;
    if(fabsf(s) > peak) peak = fabsf(s);
    // Count zero-crossings (sign changes) for frequency estimate
    if((prev < 0 && scratch[i] >= 0) || (prev >= 0 && scratch[i] < 0)) crossings++;
    prev = scratch[i];
  }
  soundData.rmsLevel  = constrain(sqrtf(sumSq / samples), 0.0f, 1.0f);
  soundData.peakLevel = constrain(peak, 0.0f, 1.0f);
  soundData.valid     = true;

  // Dominant freq estimate: crossings / 2 = full cycles in this window.
  // Window duration in seconds = samples / 22050.
  if(samples > 0) {
    float windowSec = (float)samples / 22050.0f;
    soundDomFreqHz = (uint32_t)((crossings / 2.0f) / windowSec);
  }

  // Write PCM to open WAV file
  if(soundRecording && recFile && samples > 0) {
    recFile.write((const uint8_t*)scratch, samples * sizeof(int16_t));
    recByteCount += samples * sizeof(int16_t);
    // Auto-stop at limit
    if(millis() - recStartMs >= (unsigned long)MAX_REC_SECONDS * 1000) {
      Serial.printf("[REC] Auto-stop at %ds limit\n", MAX_REC_SECONDS);
      sdStopRecording();
    }
  }
}

// Called from the periodic sensor tick — just a no-op now since
// readSoundFast() handles everything continuously. Kept so the
// main loop's mode dispatch table doesn't need restructuring.
void readSound() {
  // intentionally empty — see readSoundFast()
}

void readWireless() {
  wirelessData.scanning=true;tftUpdate();
  Serial.println("[WIFI] Scanning...");
  int found=WiFi.scanNetworks(false,true);wirelessData.wifiCount=min(found,10);
  for(int i=0;i<wirelessData.wifiCount;i++){
    strncpy(wirelessData.wifiSSID[i],WiFi.SSID(i).c_str(),32);wirelessData.wifiSSID[i][32]='\0';
    wirelessData.wifiRSSI[i]=WiFi.RSSI(i);wirelessData.wifiChannel[i]=WiFi.channel(i);
  }
  WiFi.scanDelete();
  Serial.println("[BLE] Scanning...");
  BLEScan* bleScan=BLEDevice::getScan();
  bleScan->setActiveScan(true);bleScan->setInterval(100);bleScan->setWindow(99);
  BLEScanResults results=bleScan->start(2,false);wirelessData.bleCount=min((int)results.getCount(),10);
  for(int i=0;i<wirelessData.bleCount;i++){
    BLEAdvertisedDevice dev=results.getDevice(i);
    strncpy(wirelessData.bleName[i],   dev.getName().c_str(),              32);
    strncpy(wirelessData.bleAddress[i],dev.getAddress().toString().c_str(),17);
    wirelessData.bleName[i][32]='\0';wirelessData.bleAddress[i][17]='\0';wirelessData.bleRSSI[i]=dev.getRSSI();
  }
  bleScan->clearResults();wirelessData.scanning=false;wirelessData.valid=true;
  Serial.printf("[WIRELESS] WiFi:%d  BLE:%d\n",wirelessData.wifiCount,wirelessData.bleCount);
}

// ================================================================
//  MATRIX ANIMATIONS — v3.7
//
//  Same as v3.6 (180° rotation in drawFrame helpers so user-sketched
//  hex appears physically correct) PLUS:
//   - drawFrameFromRow renamed to drawFrameRowsBelow for clarity —
//     it draws source rows >= threshold, which AFTER the 180° render
//     rotation appears at the physical BOTTOM of the matrix and grows
//     UP as threshold decreases. This is the cumulative bottom-up
//     fill the user wants.
//   - At 0°C only the bulb-adjacent mercury dot lights.
//   - At 70°C the full mercury column fills up the tube.
//   - Below 0°C: only outline shows, no mercury.
//
//  ALSO: encoder scroll lag fix lives in handleEncoder() — see the
//  separate patch in the message body for that section.
// ================================================================

// Render a static frame from a 64-bit hex pattern.
// Pattern is read as user sketched it (byte 0 = top row, MSB = left
// column), then rotated 180° before drawing so it lands physically
// the right way up on this matrix orientation (setRotation(3) on
// our HT16K33 module places logical (0,0) at physical bottom-right,
// so we counter that with a 180° flip in software).
// Read hex frame literally. Byte 0 (most significant byte of the
// uint64) = top row. MSB of each byte = leftmost column.
// No rotation, no flips. The same px() that draws every other icon.
static void drawFrame(uint64_t img) {
  for(int r=0; r<8; r++) {
    uint8_t row = (uint8_t)((img >> ((7-r)*8)) & 0xFF);
    for(int c=0; c<8; c++) {
      if(row & (1 << (7-c))) px(7-c, 7-r);
    }
  }
}

// ---------- ENVIRONMENTAL ICONS ----------

// ---------- THERMOMETER ----------
// Outline drawn always. Mercury fills cumulatively bulb → diagonal up.
// All coordinates in user's 1-indexed (rowAxBcol) system, converted
// here to 0-indexed (col, row). After drawFrame's 180° rotation,
// these land at physical bottom-left (bulb) climbing diagonally up.
static void drawIconTemp(uint32_t /*f*/) {
  // 7 frames: index 0 = empty bulb (≤0°C), index 6 = full (≥60°C)
  static const uint64_t TEMP_FRAMES[7] = {
    0x07050b142850a0c0ULL,  // 0: ≤0°C — outline only
    0x07070b142850a0c0ULL,  // 1: 0-10°C
    0x07070f142850a0c0ULL,  // 2: 10-20°C
    0x07070f1c2850a0c0ULL,  // 3: 20-30°C
    0x07070f1c3850a0c0ULL,  // 4: 30-40°C
    0x07070f1c3870a0c0ULL,  // 5: 40-50°C
    0x07070f1c3870e0c0ULL,  // 6: ≥50°C — full mercury
  };
  float t = envData.temperature;
  int idx;
  if      (t <  0.0f) idx = 0;
  else if (t < 10.0f) idx = 1;
  else if (t < 20.0f) idx = 2;
  else if (t < 30.0f) idx = 3;
  else if (t < 40.0f) idx = 4;
  else if (t < 50.0f) idx = 5;
  else                idx = 6;
  drawFrame(TEMP_FRAMES[idx]);
}

// Humidity — uses px() directly, no rotation needed since this icon
// was working visually before. Two reserved interior pixels light
// only as overflow indicator above 100%.
static void drawIconHumidity(uint32_t /*f*/) {
  px(3,0); px(4,0);
  px(2,1); px(5,1);
  px(2,2); px(5,2);
  px(1,3); px(6,3);
  px(1,4); px(6,4);
  px(1,5); px(6,5);
  px(2,6); px(5,6);
  px(3,7); px(4,7);
  float h = constrain(envData.humidity / 100.0f, 0.0f, 1.0f);
  int fillRows = (int)(h * 6.0f + 0.5f);
  if(fillRows>=1){ px(3,6); px(4,6); }
  if(fillRows>=2){ for(int x=2;x<=5;x++) px(x,5); }
  if(fillRows>=3){ for(int x=2;x<=5;x++) px(x,4); }
  if(fillRows>=4){ for(int x=2;x<=5;x++) px(x,3); }
  if(fillRows>=5){ for(int x=3;x<=4;x++) px(x,2); }
  if(fillRows>=6){ for(int x=3;x<=4;x++) px(x,1); }
  if(envData.humidity > 100.0f) {
    px(3,1); px(4,1);
  }
}

// Pressure — horizontal bar, tick marks at ends.
static void drawIconPressure(uint32_t /*f*/) {
  // 13 frames covering the full pressure spectrum:
  //   frame 0:  very low (<975 hPa) — storm warning indicator
  //   frame 1:  blank transition (975-980)
  //   frames 2-10: graduated bar scale across 980-1030 hPa
  //                (5.55 hPa per step)
  //   frame 11: blank transition (1030-1035)
  //   frame 12: very high (>1035 hPa) — clear-weather indicator
  static const uint64_t PRESSURE_FRAMES[13] = {
    0x003c009966669900ULL,  // 0  storm
    0x0000000000000000ULL,  // 1  transition
    0x81c381c381c381c3ULL,  // 2  empty bars
    0x99c381c381c381c3ULL,  // 3
    0x99db81c381c381c3ULL,  // 4
    0x99db99c381c381c3ULL,  // 5
    0x99db99db81c381c3ULL,  // 6
    0x99db99db99c381c3ULL,  // 7
    0x99db99db99db81c3ULL,  // 8
    0x99db99db99db99c3ULL,  // 9
    0x99db99db99db99dbULL,  // 10 full bars
    0x0000000000000000ULL,  // 11 transition
    0x003c009966669900ULL,  // 12 clear
  };

  float p = envData.pressure;
  int idx;
  if      (p < 975.0f)  idx = 0;
  else if (p < 980.0f)  idx = 1;
  else if (p > 1035.0f) idx = 12;
  else if (p > 1030.0f) idx = 11;
  else {
    // Map 980..1030 hPa across frames 2..10 (9 frames, ~5.56 hPa each)
    float n = (p - 980.0f) / 50.0f;            // 0.0..1.0
    idx = 2 + (int)(n * 8.0f + 0.5f);          // 2..10
    if(idx < 2)  idx = 2;
    if(idx > 10) idx = 10;
  }
  drawFrame(PRESSURE_FRAMES[idx]);
}

// ---------- CLOUD / GAS ----------
// Outline drawn always. Particles fill cumulatively from bottom-left
// across, then up — like rainwater pooling in the cloud.
static void drawIconGas(uint32_t /*f*/) {
  // 10 frames: index 0 = empty cloud (cleanest air),
  //            index 9 = full cloud (worst air)
  static const uint64_t GAS_FRAMES[10] = {
    0x00007e8181864830ULL,
    0x00007e9581864830ULL,
    0x00007ed781864830ULL,
    0x00007eff81864830ULL,
    0x00007eff93864830ULL,
    0x00007effb7864830ULL,
    0x00007effff864830ULL,
    0x00007effffae4830ULL,
    0x00007efffffe4830ULL,
    0x00007efffffe7830ULL,
  };
  // Lower kOhm = more VOCs = higher index
  float g = envData.gasKOhm;
  float dirty = 1.0f - constrain((g - 5.0f) / 195.0f, 0.0f, 1.0f);
  int idx = (int)(dirty * 9.0f + 0.5f);
  if(idx < 0) idx = 0;
  if(idx > 9) idx = 9;
  drawFrame(GAS_FRAMES[idx]);
}

// UV — falling particles. Drives off uvRaw (LTR390 datasheet conversion
// is unreliable at our 3x gain + 16-bit resolution combo, so we use
// raw counts directly — about 1 particle per 3 raw counts.
static void drawIconUV(uint32_t f) {
  int particles = (int)(envData.uvRaw / 3.0f + 0.5f);
  if(particles < 0) particles = 0;
  if(particles > 16) particles = 16;
  for(int i=0; i<particles; i++) {
    uint32_t seed = (uint32_t)i * 2654435761u + (f / 30) * 374761393u;
    int col = (int)(seed % 8);
    int phaseOffset = (seed >> 8) % 8;
    int row = (int)((f / 2 + phaseOffset) % 8);
    px(col, row);
  }
}

void drawModeEnvironmental(uint32_t f) {
  switch(sys.envIconIdx) {
    case 0: drawIconTemp(f);     break;
    case 1: drawIconHumidity(f); break;
    case 2: drawIconPressure(f); break;
    case 3: drawIconGas(f);      break;
    case 4: drawIconUV(f);       break;
  }
}

// ---------- MEASURE ICONS ----------

static void drawIconDistance(uint32_t f) {
  for(int x=0;x<8;x++) px(x,7);
  if(!measureData.distanceValid) return;
  float meters = measureData.distanceMM / 1000.0f;
  int bandRow;
  if      (meters < 1.0f) bandRow = 6;
  else if (meters < 2.0f) bandRow = 5;
  else if (meters < 3.0f) bandRow = 4;
  else if (meters < 4.0f) bandRow = 3;
  else if (meters < 5.0f) bandRow = 2;
  else if (meters < 6.0f) bandRow = 1;
  else                    bandRow = 0;
  for(int x=0;x<8;x++) px(x, bandRow);
  int col = 3 + ((f/8) % 2);
  for(int y=bandRow; y<=7; y++) px(col, y);
}

static void drawIconHeading(uint32_t f) {
  px(3,0); px(4,0);
  px(7,3); px(7,4);
  px(3,7); px(4,7);
  px(0,3); px(0,4);
  px(6,1);
  px(6,6);
  px(1,6);
  px(1,1);
  float h = measureData.heading;
  if(!measureData.magCalibrated) {
    h = (f * 5) % 360;
  }
  float rad = (h - 90.0f) * 3.14159f / 180.0f;
  float ex = 3.5f + 3.0f * cosf(rad);
  float ey = 3.5f + 3.0f * sinf(rad);
  matrix.drawLine(3, 3, (int)(ex+0.5f), (int)(ey+0.5f), LED_ON);
  px(3,3); px(4,3); px(3,4); px(4,4);
}

static void drawIconColor(uint32_t /*f*/) {
  if(!measureData.apdsOk) {
    matrix.drawLine(0,0,7,7,LED_ON);
    matrix.drawLine(0,7,7,0,LED_ON);
    return;
  }
  uint32_t mx = measureData.colorR;
  if(measureData.colorG > mx) mx = measureData.colorG;
  if(measureData.colorB > mx) mx = measureData.colorB;
  if(mx < 10) mx = 10;
  int barR = (int)((uint64_t)measureData.colorR * 8 / mx);
  int barG = (int)((uint64_t)measureData.colorG * 8 / mx);
  int barB = (int)((uint64_t)measureData.colorB * 8 / mx);
  barR = constrain(barR, 0, 8);
  barG = constrain(barG, 0, 8);
  barB = constrain(barB, 0, 8);
  for(int y=0; y<barR; y++) { px(0, 7-y); px(1, 7-y); }
  for(int y=0; y<barG; y++) { px(3, 7-y); px(4, 7-y); }
  for(int y=0; y<barB; y++) { px(6, 7-y); px(7, 7-y); }
}

void drawModeMeasure(uint32_t f) {
  switch(sys.measureIconIdx) {
    case 0: drawIconDistance(f); break;
    case 1: drawIconHeading(f);  break;
    case 2: drawIconColor(f);    break;
  }
}

// ---------- SOUND ----------

void drawModeSound(uint32_t f) {
  if(!soundData.valid) return;
  for(int x=0; x<8; x++) {
    int16_t s = soundData.buffer[x * 128];
    float n = (float)s / 32768.0f;
    int rowOffset = (int)(n * 4.0f * 3.0f);
    rowOffset = constrain(rowOffset, -3, 3);
    int row = 4 - rowOffset;
    row = constrain(row, 0, 7);
    px(x, row);
  }
}

// ---------- BIO ----------
// User-supplied heartbeat shape, swept right-to-left across the
// matrix. Pattern shifts column-by-column every 2 frames.
void drawModeBio(uint32_t f) {
  static const uint64_t HEARTBEAT = 0x00080ccb28180800ULL;
  int shift = (f / 2) % 8;
  uint64_t shifted = 0;
  for(int r=0; r<8; r++) {
    uint8_t row = (uint8_t)((HEARTBEAT >> ((7-r)*8)) & 0xFF);
    uint8_t newRow;
    if(shift == 0) newRow = row;
    else newRow = (uint8_t)((row << shift) | (row >> (8 - shift)));
    shifted |= ((uint64_t)newRow) << ((7-r)*8);
  }
  drawFrame(shifted);
}

// ---------- WIRELESS ----------

void drawModeWireless(uint32_t f) {
  px(3,3); px(4,3); px(3,4); px(4,4);
  int radius = 1 + (f/4) % 3;
  for(int x=0;x<8;x++) for(int y=0;y<8;y++){
    int dx=abs(x-3) + (x>3?-1:0);
    int dy=abs(y-3) + (y>3?-1:0);
    int dist = dx+dy;
    if(dist == radius) px(x,y);
  }
}

// ---------- FILES ----------

void drawModeFiles(uint32_t f) {
  px(0,2); px(1,2); px(2,2);
  px(2,1); px(3,1); px(4,1); px(5,1); px(6,1); px(7,1);
  px(7,2);
  px(0,3); px(7,3);
  px(0,4); px(7,4);
  px(0,5); px(7,5);
  px(0,6); px(7,6);
  for(int x=0;x<8;x++) px(x,7);
  int phase = (f/8) % 4;
  for(int i=0; i<3; i++) {
    int col = 2 + i*2;
    int row = 4 + ((phase + i) % 2);
    px(col, row);
  }
}

// ================================================================
//  WIFI FEATURE IMPLEMENTATION (v3.5)
// ================================================================

static String httpUrlEncode(const String& s) {
  String out;
  for(size_t i=0; i<s.length(); i++) {
    char c = s[i];
    if(isalnum(c) || c=='-' || c=='_' || c=='.' || c=='/' || c=='~') out += c;
    else { char buf[4]; snprintf(buf, sizeof(buf), "%%%02X", (uint8_t)c); out += buf; }
  }
  return out;
}

static String httpHtmlEscape(const String& s) {
  String out;
  for(size_t i=0; i<s.length(); i++) {
    char c = s[i];
    if(c=='<')      out += "&lt;";
    else if(c=='>') out += "&gt;";
    else if(c=='&') out += "&amp;";
    else if(c=='"') out += "&quot;";
    else            out += c;
  }
  return out;
}

static String httpFmtSize(size_t bytes) {
  char buf[16];
  if(bytes < 1024)            snprintf(buf, sizeof(buf), "%u B", (unsigned)bytes);
  else if(bytes < 1024*1024)  snprintf(buf, sizeof(buf), "%.1f KB", bytes/1024.0);
  else                        snprintf(buf, sizeof(buf), "%.1f MB", bytes/1048576.0);
  return String(buf);
}

void wifiEnter() {
  // Build AP SSID from MAC (last 4 hex)
  uint8_t mac[6];
  WiFi.macAddress(mac);
  snprintf(wifiSt.apSsid, sizeof(wifiSt.apSsid),
    "Tricorder-%02X%02X", mac[4], mac[5]);

  wifiSt.mode    = WIFI_MENU;
  wifiSt.menuIdx = 0;
  wifiSt.startTime = millis();
  wifiSt.httpReqCount = 0;
  diagLog("WIFI", "Submenu entered. AP name will be: %s", wifiSt.apSsid);
  playClick();
}

void wifiExit() {
  if(wifiSt.mode == WIFI_OFF_STATE) return;
  diagLog("WIFI", "Exiting. Uptime: %lus, requests served: %d",
    (millis() - wifiSt.startTime)/1000, wifiSt.httpReqCount);

  httpServer.stop();
  WiFi.disconnect(true, true);
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);

  wifiSt.mode = WIFI_OFF_STATE;
  wifiSt.ntpSynced = false;
  wifiSt.ipStr[0] = '\0';
  wifiSt.connectedSsid[0] = '\0';
  wifiSt.setupFallback = false;
  playClick();
}

void wifiStartAP(bool setupMode) {
  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(wifiSt.apSsid);  // open network, no password
  if(!ok) {
    diagLog("WIFI", "softAP failed");
    wifiSt.mode = WIFI_OFF_STATE;
    return;
  }
  delay(100);
  IPAddress ip = WiFi.softAPIP();
  snprintf(wifiSt.ipStr, sizeof(wifiSt.ipStr), "%d.%d.%d.%d",
    ip[0], ip[1], ip[2], ip[3]);

  httpServer.on("/",        httpHandleRoot);
  httpServer.on("/browse",  httpHandleBrowse);
  httpServer.on("/file",    httpHandleFile);
  httpServer.on("/delete",  httpHandleDelete);
  httpServer.on("/api/status", httpHandleStatus);
  if(setupMode) {
    httpServer.on("/setup", HTTP_GET,  httpHandleSetup);
    httpServer.on("/setup", HTTP_POST, httpHandleSetupPost);
  }
  httpServer.onNotFound(httpHandleNotFound);
  httpServer.begin();

  wifiSt.mode = setupMode ? WIFI_AP_SETUP : WIFI_AP_ACTIVE;
  wifiSt.setupFallback = setupMode;
  diagLog("WIFI", "AP started. SSID:%s  IP:%s  setup=%d",
    wifiSt.apSsid, wifiSt.ipStr, setupMode?1:0);
  beep(1200, 60);
}

void wifiStartSTA() {
  char ssid[33] = {0};
  char pass[65] = {0};
  if(!wifiLoadCreds(ssid, sizeof(ssid), pass, sizeof(pass)) || ssid[0] == '\0') {
    diagLog("WIFI", "No credentials — falling back to AP setup mode");
    wifiStartAP(true);
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  wifiSt.mode = WIFI_STA_CONNECTING;
  wifiSt.connectAttemptStart = millis();
  strncpy(wifiSt.connectedSsid, ssid, 32);
  wifiSt.connectedSsid[32] = '\0';
  diagLog("WIFI", "STA connecting to %s...", ssid);
}

void wifiTickConnecting() {
  if(WiFi.status() == WL_CONNECTED) {
    IPAddress ip = WiFi.localIP();
    snprintf(wifiSt.ipStr, sizeof(wifiSt.ipStr), "%d.%d.%d.%d",
      ip[0], ip[1], ip[2], ip[3]);

    httpServer.on("/",        httpHandleRoot);
    httpServer.on("/browse",  httpHandleBrowse);
    httpServer.on("/file",    httpHandleFile);
    httpServer.on("/delete",  httpHandleDelete);
    httpServer.on("/api/status", httpHandleStatus);
    httpServer.onNotFound(httpHandleNotFound);
    httpServer.begin();

    // NTP sync — Central time, auto-DST
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    setenv("TZ", "CST6CDT,M3.2.0,M11.1.0", 1);
    tzset();
    wifiSt.mode = WIFI_STA_ACTIVE;
    diagLog("WIFI", "STA connected. IP:%s RSSI:%d", wifiSt.ipStr, WiFi.RSSI());
    beep(1400, 60);
    return;
  }
  // 15s timeout
  if(millis() - wifiSt.connectAttemptStart > 15000) {
    diagLog("WIFI", "STA connect timeout — falling back to AP setup");
    WiFi.disconnect(true);
    delay(100);
    wifiStartAP(true);
  }
}

void wifiTickActive() {
  httpServer.handleClient();
  // Probe NTP once a second until synced
  static unsigned long lastNtpProbe = 0;
  if(!wifiSt.ntpSynced && wifiSt.mode == WIFI_STA_ACTIVE
     && millis() - lastNtpProbe > 1000) {
    lastNtpProbe = millis();
    struct tm tinfo;
    if(getLocalTime(&tinfo, 50)) {
      wifiSt.ntpSynced = true;
      diagLog("WIFI", "NTP synced: %04d-%02d-%02d %02d:%02d:%02d",
        tinfo.tm_year+1900, tinfo.tm_mon+1, tinfo.tm_mday,
        tinfo.tm_hour, tinfo.tm_min, tinfo.tm_sec);
    }
  }
}

bool wifiLoadCreds(char* ssid, size_t ssidLen, char* pass, size_t passLen) {
  if(!sys.sdMounted) return false;
  File f = SD.open("/tricorder/config/wifi.json", FILE_READ);
  if(!f) return false;
  String content = f.readString();
  f.close();
  // Crude but adequate JSON parsing for two known keys
  int sIdx = content.indexOf("\"ssid\"");
  int pIdx = content.indexOf("\"pass\"");
  if(sIdx < 0 || pIdx < 0) return false;
  int sStart = content.indexOf('"', content.indexOf(':', sIdx)) + 1;
  int sEnd   = content.indexOf('"', sStart);
  int pStart = content.indexOf('"', content.indexOf(':', pIdx)) + 1;
  int pEnd   = content.indexOf('"', pStart);
  if(sStart <= 0 || sEnd <= 0 || pStart <= 0 || pEnd <= 0) return false;
  String s = content.substring(sStart, sEnd);
  String p = content.substring(pStart, pEnd);
  strncpy(ssid, s.c_str(), ssidLen-1); ssid[ssidLen-1] = '\0';
  strncpy(pass, p.c_str(), passLen-1); pass[passLen-1] = '\0';
  return true;
}

bool wifiSaveCreds(const char* ssid, const char* pass) {
  if(!sys.sdMounted) return false;
  if(!SD.exists("/tricorder/config")) SD.mkdir("/tricorder/config");
  if(SD.exists("/tricorder/config/wifi.json"))
    SD.remove("/tricorder/config/wifi.json");
  File f = SD.open("/tricorder/config/wifi.json", FILE_WRITE);
  if(!f) return false;
  f.printf("{\n  \"ssid\": \"%s\",\n  \"pass\": \"%s\"\n}\n", ssid, pass);
  f.close();
  diagLog("WIFI", "Saved creds for SSID: %s", ssid);
  return true;
}

void httpSendDirListing(const String& path) {
  wifiSt.httpReqCount++;
  if(!sys.sdMounted) {
    httpServer.send(503, "text/plain", "SD not mounted");
    return;
  }
  File dir = SD.open(path);
  if(!dir || !dir.isDirectory()) {
    if(dir) dir.close();
    httpServer.send(404, "text/plain", "Not a directory: " + path);
    return;
  }

  String html;
  html.reserve(4096);
  html += F("<!DOCTYPE html><html><head><meta charset='utf-8'>"
            "<meta name='viewport' content='width=device-width,initial-scale=1'>"
            "<title>Tricorder Files</title><style>"
            "body{font-family:system-ui,sans-serif;background:#0a0e1a;color:#cce;"
            "max-width:720px;margin:0 auto;padding:1em;}"
            "h1{color:#5cf;border-bottom:1px solid #234;padding-bottom:.3em;}"
            ".path{color:#8af;font-family:monospace;margin-bottom:1em;}"
            "table{width:100%;border-collapse:collapse;}"
            "th,td{text-align:left;padding:.4em .6em;border-bottom:1px solid #234;}"
            "th{color:#5cf;}"
            "a{color:#8af;text-decoration:none;}a:hover{color:#cf5;}"
            ".dir{color:#fc8;}.size{color:#888;text-align:right;font-family:monospace;}"
            ".del{color:#f66;font-size:.85em;margin-left:.6em;}"
            ".del:hover{color:#fff;background:#a22;padding:0 .3em;border-radius:3px;}"
            ".bar{margin-top:1em;color:#666;font-size:.9em;}"
            "</style></head><body>");
  html += F("<h1>Tricorder SD</h1>");
  html += "<div class='path'>" + httpHtmlEscape(path) + "</div>";
  html += F("<table><tr><th>Name</th><th>Size</th></tr>");

  if(path != "/" && path.length() > 0) {
    int slash = path.lastIndexOf('/');
    String parent = (slash <= 0) ? "/" : path.substring(0, slash);
    html += "<tr><td><a class='dir' href='/browse?path="
         + httpUrlEncode(parent) + "'>../</a></td><td></td></tr>";
  }

  while(true) {
    File entry = dir.openNextFile();
    if(!entry) break;
    String name = String(entry.name());
    int ls = name.lastIndexOf('/');
    if(ls >= 0) name = name.substring(ls+1);
    if(name.length() == 0) { entry.close(); continue; }

    String fullPath = (path == "/") ? ("/" + name) : (path + "/" + name);
    if(entry.isDirectory()) {
      html += "<tr><td><a class='dir' href='/browse?path="
           + httpUrlEncode(fullPath) + "'>" + httpHtmlEscape(name) + "/</a></td>"
           + "<td class='size'>—</td></tr>";
    } else {
      // JS confirm + GET to /delete. Page reloads back to current dir on success.
      String confirmJs = "return confirm('Delete " + name + "?');";
      html += "<tr><td><a href='/file?path=" + httpUrlEncode(fullPath) + "'>"
           + httpHtmlEscape(name) + "</a>"
           + "<a class='del' href='/delete?path=" + httpUrlEncode(fullPath)
           + "&back=" + httpUrlEncode(path) + "' onclick=\"" + confirmJs + "\">[del]</a>"
           + "</td>"
           + "<td class='size'>" + httpFmtSize(entry.size()) + "</td></tr>";
    }
    entry.close();
  }
  dir.close();

  html += F("</table><div class='bar'>"
            "<a href='/api/status'>status JSON</a> · ");
  html += String(wifiSt.httpReqCount) + " requests served";
  html += F("</div></body></html>");
  httpServer.send(200, "text/html", html);
}

void httpHandleRoot() {
  httpSendDirListing("/");
}

void httpHandleBrowse() {
  String path = httpServer.hasArg("path") ? httpServer.arg("path") : "/";
  if(!path.startsWith("/") || path.indexOf("..") >= 0) {
    httpServer.send(400, "text/plain", "Bad path");
    return;
  }
  httpSendDirListing(path);
}

void httpHandleFile() {
  wifiSt.httpReqCount++;
  if(!httpServer.hasArg("path")) {
    httpServer.send(400, "text/plain", "Missing path");
    return;
  }
  String path = httpServer.arg("path");
  if(!path.startsWith("/") || path.indexOf("..") >= 0) {
    httpServer.send(400, "text/plain", "Bad path");
    return;
  }
  if(!sys.sdMounted) {
    httpServer.send(503, "text/plain", "SD not mounted");
    return;
  }
  File f = SD.open(path, FILE_READ);
  if(!f || f.isDirectory()) {
    if(f) f.close();
    httpServer.send(404, "text/plain", "Not found: " + path);
    return;
  }

  const char* mime = "application/octet-stream";
  String lower = path; lower.toLowerCase();
  if      (lower.endsWith(".csv"))  mime = "text/csv";
  else if (lower.endsWith(".json")) mime = "application/json";
  else if (lower.endsWith(".log"))  mime = "text/plain";
  else if (lower.endsWith(".txt"))  mime = "text/plain";
  else if (lower.endsWith(".wav"))  mime = "audio/wav";
  else if (lower.endsWith(".html")) mime = "text/html";

  httpServer.sendHeader("Content-Disposition",
    "inline; filename=\"" + path.substring(path.lastIndexOf('/')+1) + "\"");
  httpServer.streamFile(f, mime);
  f.close();
}

// Delete a file from the SD card. GET /delete?path=...&back=...
// Path validation: must start with /, no .. traversal, must be a file
// (not a directory — recursive delete is disabled to prevent oops).
// On success, redirects back to the parent dir listing.
void httpHandleDelete() {
  wifiSt.httpReqCount++;
  if(!sys.sdMounted) {
    httpServer.send(503, "text/plain", "SD not mounted");
    return;
  }
  if(!httpServer.hasArg("path")) {
    httpServer.send(400, "text/plain", "Missing path");
    return;
  }
  String path = httpServer.arg("path");
  if(!path.startsWith("/") || path.indexOf("..") >= 0) {
    httpServer.send(400, "text/plain", "Bad path");
    return;
  }

  // Open the entry to confirm it's a regular file before deleting.
  File f = SD.open(path, FILE_READ);
  if(!f) {
    httpServer.send(404, "text/plain", "Not found: " + path);
    return;
  }
  if(f.isDirectory()) {
    f.close();
    httpServer.send(400, "text/plain", "Refusing to delete directory: " + path);
    return;
  }
  f.close();

  bool ok = SD.remove(path);
  diagLog("WIFI", "Delete %s -> %s", path.c_str(), ok ? "OK" : "FAILED");
  if(!ok) {
    httpServer.send(500, "text/plain", "Delete failed: " + path);
    return;
  }

  // Redirect back to the parent dir, or to the explicit back path if given.
  String back;
  if(httpServer.hasArg("back")) {
    back = httpServer.arg("back");
    if(!back.startsWith("/") || back.indexOf("..") >= 0) back = "/";
  } else {
    int slash = path.lastIndexOf('/');
    back = (slash <= 0) ? "/" : path.substring(0, slash);
  }
  httpServer.sendHeader("Location", "/browse?path=" + httpUrlEncode(back));
  httpServer.send(303, "text/plain", "Deleted");
}

void httpHandleStatus() {
  wifiSt.httpReqCount++;
  char json[512];
  struct tm tinfo;
  char timeStr[32] = "not synced";
  if(wifiSt.ntpSynced && getLocalTime(&tinfo, 10)) {
    strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &tinfo);
  }
  snprintf(json, sizeof(json),
    "{\n"
    "  \"uptime_ms\": %lu,\n"
    "  \"wall_time\": \"%s\",\n"
    "  \"mode\": \"%s\",\n"
    "  \"heap_free\": %u,\n"
    "  \"psram_free\": %u,\n"
    "  \"sd_size_mb\": %.1f,\n"
    "  \"sd_used_mb\": %.1f,\n"
    "  \"wifi_mode\": \"%s\",\n"
    "  \"wifi_ssid\": \"%s\",\n"
    "  \"wifi_ip\": \"%s\",\n"
    "  \"http_requests\": %d\n"
    "}\n",
    millis(), timeStr, MODE_NAMES[sys.currentMode],
    ESP.getFreeHeap(), ESP.getFreePsram(),
    SD.cardSize()/1048576.0, SD.usedBytes()/1048576.0,
    (wifiSt.mode == WIFI_AP_ACTIVE || wifiSt.mode == WIFI_AP_SETUP) ? "AP" : "STA",
    (wifiSt.mode == WIFI_AP_ACTIVE || wifiSt.mode == WIFI_AP_SETUP) ? wifiSt.apSsid : wifiSt.connectedSsid,
    wifiSt.ipStr,
    wifiSt.httpReqCount);
  httpServer.send(200, "application/json", json);
}

void httpHandleSetup() {
  wifiSt.httpReqCount++;
  String html = F(
    "<!DOCTYPE html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>Tricorder Setup</title><style>"
    "body{font-family:system-ui,sans-serif;background:#0a0e1a;color:#cce;"
    "max-width:480px;margin:0 auto;padding:1em;}"
    "h1{color:#5cf;}label{display:block;margin:1em 0 .3em;color:#8af;}"
    "input{width:100%;padding:.6em;font-size:1em;background:#1a2030;"
    "color:#cce;border:1px solid #345;border-radius:4px;box-sizing:border-box;}"
    "button{margin-top:1.5em;padding:.8em 1.5em;font-size:1em;"
    "background:#28a;color:white;border:none;border-radius:4px;cursor:pointer;}"
    "button:hover{background:#39b;}"
    ".note{color:#888;font-size:.9em;margin-top:1em;}"
    "</style></head><body>"
    "<h1>WiFi Setup</h1>"
    "<p>Enter your home WiFi credentials. They will be saved to "
    "<code>/tricorder/config/wifi.json</code> and used for future "
    "STA-mode connections.</p>"
    "<form method='POST' action='/setup'>"
    "<label>SSID</label>"
    "<input name='ssid' required maxlength='32' autocomplete='off'>"
    "<label>Password</label>"
    "<input name='pass' type='password' maxlength='64' autocomplete='off'>"
    "<button type='submit'>Save</button>"
    "</form>"
    "<div class='note'>After saving, exit WiFi mode and re-enter via "
    "Join WiFi to connect to your network.</div>"
    "</body></html>");
  httpServer.send(200, "text/html", html);
}

void httpHandleSetupPost() {
  wifiSt.httpReqCount++;
  if(!httpServer.hasArg("ssid") || !httpServer.hasArg("pass")) {
    httpServer.send(400, "text/plain", "Missing fields");
    return;
  }
  String ssid = httpServer.arg("ssid");
  String pass = httpServer.arg("pass");
  if(ssid.length() == 0 || ssid.length() > 32 || pass.length() > 64) {
    httpServer.send(400, "text/plain", "Invalid lengths");
    return;
  }
  if(!wifiSaveCreds(ssid.c_str(), pass.c_str())) {
    httpServer.send(500, "text/plain", "Save failed");
    return;
  }
  String html = "<!DOCTYPE html><html><body style='font-family:sans-serif;"
                "background:#0a0e1a;color:#cce;padding:2em;text-align:center;'>"
                "<h1 style='color:#5cf;'>Saved</h1>"
                "<p>Credentials written for SSID: <b>" + httpHtmlEscape(ssid) + "</b></p>"
                "<p>Exit WiFi mode on the device, then re-enter via Join WiFi.</p>"
                "</body></html>";
  httpServer.send(200, "text/html", html);
}

void httpHandleNotFound() {
  httpServer.send(404, "text/plain", "Not found: " + httpServer.uri());
}

void wifiDrawTFT() {
  tft.fillRect(0, TFT_DATA_Y, TFT_W, TFT_H - TFT_DATA_Y, ST77XX_BLACK);
  tft.setTextSize(1);

  switch(wifiSt.mode) {
    case WIFI_MENU: {
      tft.setTextColor(ST77XX_CYAN);
      tft.setCursor(4, TFT_DATA_Y);
      tft.print("WIFI MODE");
      const char* opts[3] = {"Host AP (no internet)", "Join WiFi (NTP+net)", "Exit"};
      for(int i=0; i<3; i++) {
        int y = TFT_DATA_Y + 20 + i*16;
        if(i == wifiSt.menuIdx) {
          tft.fillRect(0, y-2, TFT_W, 14, 0x2104);
          tft.setTextColor(ST77XX_YELLOW);
        } else {
          tft.setTextColor(ST77XX_WHITE);
        }
        tft.setCursor(8, y);
        tft.print(opts[i]);
      }
      tft.setTextColor(0x528A);
      tft.setCursor(4, TFT_H-10);
      tft.print("turn=select  click=go");
      break;
    }
    case WIFI_AP_ACTIVE:
    case WIFI_AP_SETUP: {
      tft.setTextColor(ST77XX_GREEN);
      tft.setCursor(4, TFT_DATA_Y);
      tft.print(wifiSt.mode == WIFI_AP_SETUP ? "AP SETUP MODE" : "HOSTING AP");
      tft.setTextColor(ST77XX_CYAN);
      tft.setCursor(4, TFT_DATA_Y+16); tft.print("SSID:");
      tft.setTextColor(ST77XX_WHITE);
      tft.setCursor(4, TFT_DATA_Y+28); tft.print(wifiSt.apSsid);
      tft.setTextColor(ST77XX_CYAN);
      tft.setCursor(4, TFT_DATA_Y+44); tft.print("Browse:");
      tft.setTextColor(ST77XX_YELLOW);
      tft.setCursor(4, TFT_DATA_Y+56); tft.print("http://");
      tft.print(wifiSt.ipStr);
      if(wifiSt.mode == WIFI_AP_SETUP) {
        tft.setTextColor(TFT_ORANGE);
        tft.setCursor(4, TFT_DATA_Y+72); tft.print("Visit /setup to");
        tft.setCursor(4, TFT_DATA_Y+82); tft.print("configure home WiFi");
      } else {
        tft.setTextColor(0x7BEF);
        tft.setCursor(4, TFT_DATA_Y+72);
        tft.printf("Reqs: %d", wifiSt.httpReqCount);
        tft.setCursor(4, TFT_DATA_Y+82);
        tft.printf("Up: %lus", (millis() - wifiSt.startTime)/1000);
      }
      tft.setTextColor(0x528A);
      tft.setCursor(4, TFT_H-10);
      tft.print("LEFT/RIGHT=exit");
      break;
    }
    case WIFI_STA_CONNECTING: {
      tft.setTextColor(TFT_ORANGE);
      tft.setCursor(4, TFT_DATA_Y);
      tft.print("CONNECTING...");
      tft.setTextColor(ST77XX_WHITE);
      tft.setCursor(4, TFT_DATA_Y+16); tft.print("SSID:");
      tft.setCursor(4, TFT_DATA_Y+28); tft.print(wifiSt.connectedSsid);
      tft.setTextColor(ST77XX_CYAN);
      tft.setCursor(4, TFT_DATA_Y+50);
      tft.printf("Wait: %lus / 15s", (millis() - wifiSt.connectAttemptStart)/1000);
      break;
    }
    case WIFI_STA_ACTIVE: {
      tft.setTextColor(ST77XX_GREEN);
      tft.setCursor(4, TFT_DATA_Y);
      tft.print("CONNECTED");
      tft.setTextColor(ST77XX_CYAN);
      tft.setCursor(4, TFT_DATA_Y+16); tft.print("SSID:");
      tft.setTextColor(ST77XX_WHITE);
      tft.setCursor(4, TFT_DATA_Y+28); tft.print(wifiSt.connectedSsid);
      tft.setTextColor(ST77XX_CYAN);
      tft.setCursor(4, TFT_DATA_Y+44); tft.print("IP:");
      tft.setTextColor(ST77XX_YELLOW);
      tft.setCursor(4, TFT_DATA_Y+56); tft.print(wifiSt.ipStr);
      tft.setTextColor(ST77XX_CYAN);
      tft.setCursor(4, TFT_DATA_Y+72); tft.print("Time:");
      tft.setTextColor(wifiSt.ntpSynced ? ST77XX_GREEN : 0x7BEF);
      tft.setCursor(4, TFT_DATA_Y+82);
      if(wifiSt.ntpSynced) {
        struct tm tinfo;
        if(getLocalTime(&tinfo, 10)) {
          char buf[32];
          strftime(buf, sizeof(buf), "%H:%M:%S %m/%d", &tinfo);
          tft.print(buf);
        }
      } else {
        tft.print("syncing...");
      }
      tft.setTextColor(0x528A);
      tft.setCursor(4, TFT_H-10);
      tft.printf("Reqs:%d  RSSI:%ddBm", wifiSt.httpReqCount, WiFi.RSSI());
      break;
    }
    default: break;
  }
}

// ================================================================
//  MAG CALIBRATION
// ================================================================
void runMagCalibration() {
  sys.calibrating=true;
  tft.fillScreen(ST77XX_BLACK);tft.setTextColor(ST77XX_YELLOW);tft.setTextSize(1);
  tft.setCursor(4,20);tft.print("MAG CALIBRATION");tft.setCursor(4,36);tft.print("Rotate in all dirs");tft.setCursor(4,52);tft.print("10 seconds...");
  playCalStart();
  diagLog("CAL","Mag calibration started");
  float xMn=9999,xMx=-9999,yMn=9999,yMx=-9999,zMn=9999,zMx=-9999;
  // Log samples to SD for later analysis
  File calFile;
  if(sys.sdMounted){
    calFile=SD.open("/tricorder/config/cal_samples.csv",FILE_WRITE);
    if(calFile) calFile.println("timestamp_ms,raw_x,raw_y,raw_z");
  }
  int sampleCount=0;
  unsigned long start=millis();
  while(millis()-start<10000){
    switchMux(CH_LIS);sensors_event_t e;lis2mdl.getEvent(&e);
    if(e.magnetic.x<xMn)xMn=e.magnetic.x;if(e.magnetic.x>xMx)xMx=e.magnetic.x;
    if(e.magnetic.y<yMn)yMn=e.magnetic.y;if(e.magnetic.y>yMx)yMx=e.magnetic.y;
    if(e.magnetic.z<zMn)zMn=e.magnetic.z;if(e.magnetic.z>zMx)zMx=e.magnetic.z;
    if(calFile) calFile.printf("%lu,%.2f,%.2f,%.2f\n",millis(),e.magnetic.x,e.magnetic.y,e.magnetic.z);
    sampleCount++;
    int rem=10-(int)((millis()-start)/1000);
    tft.fillRect(0,64,160,14,ST77XX_BLACK);tft.setCursor(4,68);tft.setTextColor(ST77XX_CYAN);
    tft.printf("%ds  Y:%.0f..%.0f",rem,yMn,yMx);delay(20);
  }
  if(calFile) calFile.close();
  measureData.magOffX=(xMx+xMn)/2.0f;measureData.magOffY=(yMx+yMn)/2.0f;measureData.magOffZ=(zMx+zMn)/2.0f;
  measureData.magCalibrated=true;sys.calibrating=false;
  // Write full cal result to diag log
  diagLog("CAL","Samples:%d  X range:%.2f..%.2f (span %.2f)  Y range:%.2f..%.2f (span %.2f)  Z range:%.2f..%.2f (span %.2f)",
    sampleCount,xMn,xMx,xMx-xMn,yMn,yMx,yMx-yMn,zMn,zMx,zMx-zMn);
  diagLog("CAL","Offsets X=%.2f Y=%.2f Z=%.2f",measureData.magOffX,measureData.magOffY,measureData.magOffZ);
  if((xMx-xMn)<10.0f) diagLog("CAL","WARNING: X span <10uT — insufficient rotation on X axis");
  if((yMx-yMn)<10.0f) diagLog("CAL","WARNING: Y span <10uT — insufficient rotation on Y axis");
  if((zMx-zMn)<10.0f) diagLog("CAL","WARNING: Z span <10uT — insufficient rotation on Z axis");
  if(fabsf(measureData.magOffX)>500||fabsf(measureData.magOffY)>500)
    diagLog("CAL","WARNING: offsets >500uT — sensor interference or sensor stuck");
  tft.fillScreen(ST77XX_BLACK);tft.setTextColor(ST77XX_GREEN);tft.setCursor(4,30);tft.print("CAL COMPLETE");
  tft.setTextColor(ST77XX_WHITE);tft.setCursor(4,48);tft.printf("OffX:%.1f",measureData.magOffX);
  tft.setCursor(4,60);tft.printf("OffY:%.1f",measureData.magOffY);tft.setCursor(4,72);tft.printf("OffZ:%.1f",measureData.magOffZ);
  tft.setTextColor(ST77XX_CYAN);tft.setCursor(4,90);tft.print("Samples: "+String(sampleCount));
  tft.setCursor(4,100);tft.printf("X span:%.0f Y:%.0f",xMx-xMn,yMx-yMn);
  playCalDone();delay(3500);sys.lastMode=MODE_COUNT;
}

// ================================================================
//  SERIAL OUTPUT
// ================================================================
void serialPrint() {
  Serial.printf("\n[%lu ms] %s%s%s\n",millis(),MODE_NAMES[sys.currentMode],
    sys.snapshotFrozen?" [FROZEN]":"",soundRecording?" [REC]":"");
  switch(sys.currentMode){
    case MODE_ENVIRONMENTAL:
      Serial.printf("  Temp:%.1fC  Hum:%.1f%%  Press:%.1fhPa  Gas:%.1fkOhm\n",envData.temperature,envData.humidity,envData.pressure,envData.gasKOhm);
      Serial.printf("  UV raw:%.0f  UVI:%.4f\n",envData.uvRaw,envData.uvIndex);break;
    case MODE_MEASURE:
      if(measureData.distanceValid) Serial.printf("  Dist:%dmm\n",measureData.distanceMM);
      else Serial.println("  Dist: no target");
      Serial.printf("  Heading:%.1f  Cal: X=%.2f Y=%.2f Z=%.2f uT\n",measureData.heading,measureData.magX,measureData.magY,measureData.magZ);
      Serial.printf("           Raw: X=%.2f Y=%.2f Z=%.2f uT\n",measureData.rawMagX,measureData.rawMagY,measureData.rawMagZ);
      // Sanity check: Earth's field is ~25-60uT. Values >>100uT = interference or bad cal.
      if(fabsf(measureData.rawMagX)>500||fabsf(measureData.rawMagY)>500)
        Serial.println("  [MAG] WARNING: raw >500uT — magnetic interference or bad calibration");
      Serial.printf("  APDS R=%lu G=%lu B=%lu IR=%lu  (%s)\n",
        measureData.colorR,measureData.colorG,measureData.colorB,measureData.colorIR,
        measureData.apdsOk?"OK":"NO RESP");
      if(!measureData.apdsOk) Serial.println("  [COLOR] APDS9999 not responding");
      break;
    case MODE_THERMAL:
      Serial.printf("  Mode:%s  Ctr:%.1f  Min:%.1f  Max:%.1f  D:%.1f\n",sys.thermalAbsolute?"ABS":"REL",thermalData.centerTemp,thermalData.minTemp,thermalData.maxTemp,thermalData.deltaTemp);break;
    case MODE_SOUND:
      Serial.printf("  RMS:%.4f  Peak:%.4f  Rec:%s  Bytes:%lu\n",soundData.rmsLevel,soundData.peakLevel,soundRecording?"ON":"off",recByteCount);break;
    case MODE_WIRELESS:
      Serial.printf("  View:%s  WiFi:%d  BLE:%d\n",sys.wirelessShowBLE?"BLE":"WiFi",wirelessData.wifiCount,wirelessData.bleCount);break;
    case MODE_FILES:
      Serial.printf("  Path:%s  Entries:%d\n",filesData.currentPath,filesData.count);break;
    case MODE_BIO:Serial.println("  [Bio not connected]");break;
  }
}

// ================================================================
//  ENCODER LAG FIX — replaces handleEncoder() in v3.4 sketch.
//
//  Problem: when scrolling encoder in ENV/MEASURE mode, sys.envIconIdx
//  changes immediately and the matrix updates within 100ms, but the
//  TFT bottom label ("Matrix: Temperature" → "Matrix: Humidity")
//  doesn't refresh until the next sensor tick (up to 500ms later).
//  This makes scrolling feel sluggish even though the matrix itself
//  is responsive.
//
//  Fix: call tftUpdate() right after the icon index changes, same as
//  the WIRELESS/FILES branches already do.
// ================================================================
struct { bool lastCLK=HIGH,lastSW=HIGH; unsigned long lastMs=0; } enc;

void handleEncoder() {
  bool clk=digitalRead(ENC_CLK),dt=digitalRead(ENC_DT),sw=digitalRead(ENC_SW);
  unsigned long now=millis();
  // Debounce: 150ms — increased from 30ms to suppress USB ground-loop noise.
  // Hardware fix: 100nF ceramic caps CLK→GND and DT→GND at encoder pins.
  if(clk==HIGH&&enc.lastCLK==LOW&&now-enc.lastMs>150){
    enc.lastMs=now;int delta=(dt==HIGH)?1:-1;
    bool scrolled=false;
    bool needsTftUpdate=false;
    // WiFi submenu takes priority when active
    if(wifiSt.mode == WIFI_MENU) {
      wifiSt.menuIdx = (wifiSt.menuIdx + delta + 3) % 3;
      scrolled = true; needsTftUpdate = true;
    } else if(sys.currentMode==MODE_WIRELESS&&!sys.wirelessShowDetail){
      int total=sys.wirelessShowBLE?min(wirelessData.bleCount,10):min(wirelessData.wifiCount,10);
      int* si=sys.wirelessShowBLE?&sys.bleScrollIdx:&sys.wifiScrollIdx;
      if(total>0){*si=constrain(*si+delta,0,total-1);scrolled=true;needsTftUpdate=true;}
    } else if(sys.currentMode==MODE_FILES&&!sys.filesShowDetail){
      if(filesData.count>0){sys.filesScrollIdx=constrain(sys.filesScrollIdx+delta,0,filesData.count-1);scrolled=true;needsTftUpdate=true;}
    } else if(sys.currentMode==MODE_ENVIRONMENTAL){
      sys.envIconIdx=(sys.envIconIdx+delta+5)%5;
      scrolled=true;needsTftUpdate=true;  // ← refresh TFT label immediately
    } else if(sys.currentMode==MODE_MEASURE){
      sys.measureIconIdx=(sys.measureIconIdx+delta+3)%3;
      scrolled=true;needsTftUpdate=true;  // ← refresh TFT label immediately
    }
    if(scrolled) { if(delta > 0) playEncCW(); else playEncCCW(); }
    if(needsTftUpdate) tftUpdate();
  }
  enc.lastCLK=clk;
  if(sw==LOW&&enc.lastSW==HIGH&&now-enc.lastMs>200){
    enc.lastMs=now;
    // WiFi submenu confirmation takes priority
    if(wifiSt.mode == WIFI_MENU) {
      switch(wifiSt.menuIdx) {
        case 0: wifiStartAP(false); break;
        case 1: wifiStartSTA();     break;
        case 2: wifiExit();         break;
      }
      tftDrawHeader();
      tftUpdate();
    } else if(sys.currentMode==MODE_WIRELESS){
      int total=sys.wirelessShowBLE?wirelessData.bleCount:wirelessData.wifiCount;
      if(sys.wirelessShowDetail){sys.wirelessShowDetail=false;playClick();tftUpdate();}
      else if(total>0){int* si=sys.wirelessShowBLE?&sys.bleScrollIdx:&sys.wifiScrollIdx;sys.wirelessDetailIdx=*si;sys.wirelessShowDetail=true;playClick();tftUpdate();}
    } else if(sys.currentMode==MODE_FILES){
      if(sys.filesShowDetail){sys.filesShowDetail=false;playClick();tftUpdate();}
      else if(filesData.count>0){
        FileEntry* e=&filesData.entries[sys.filesScrollIdx];
        if(e->valid){
          if(e->isDir){filesDrillIn(sys.filesScrollIdx);playClick();tftUpdate();}
          else{sys.filesDetailIdx=sys.filesScrollIdx;sys.filesShowDetail=true;playClick();tftUpdate();}
        }
      }
    } else {Serial.println("[ENC SW] Pressed");playClick();}
  }
  enc.lastSW=sw;
}

// ================================================================
//  BUTTONS
// ================================================================
struct {
  bool lastTop=HIGH,lastLeft=HIGH,lastRight=HIGH,lastBottom=HIGH;
  unsigned long topPressStart=0;bool topHolding=false;unsigned long lastMs=0;
} btn;

void handleButtons() {
  unsigned long now=millis();
  if(now-btn.lastMs<200) return;
  bool bTop=digitalRead(BTN_TOP),bLeft=digitalRead(BTN_LEFT),bRight=digitalRead(BTN_RIGHT),bBottom=digitalRead(BTN_BOTTOM);

  // If WiFi is up, LEFT/RIGHT both tear it down before doing anything else.
  // This guarantees WiFi never persists across mode changes.
  if(wifiSt.mode != WIFI_OFF_STATE) {
    if((bRight==LOW && btn.lastRight==HIGH) ||
       (bLeft ==LOW && btn.lastLeft ==HIGH)) {
      Serial.println("[WIFI] Mode change — tearing down WiFi");
      wifiExit();
      // fall through to existing mode-change logic below
    }
  }

  if(bRight==LOW&&btn.lastRight==HIGH){
    sys.currentMode=(MacroMode)((sys.currentMode+1)%MODE_COUNT);
    sys.snapshotFrozen=false;btn.lastMs=now;
    Serial.printf("[BTN RIGHT] -> %s\n",MODE_NAMES[sys.currentMode]);
    playBtnRight();
    sys.scanLoopWanted = modeHasScanLoop(sys.currentMode);
  }
  btn.lastRight=bRight;

  if(bLeft==LOW&&btn.lastLeft==HIGH){
    sys.currentMode=(MacroMode)((sys.currentMode+MODE_COUNT-1)%MODE_COUNT);
    sys.snapshotFrozen=false;btn.lastMs=now;
    Serial.printf("[BTN LEFT] -> %s\n",MODE_NAMES[sys.currentMode]);
    playBtnLeft();
    sys.scanLoopWanted = modeHasScanLoop(sys.currentMode);
  }
  btn.lastLeft=bLeft;

  // TOP — tap (<1s) vs hold (≥1s)
  if(bTop==LOW){
    if(!btn.topHolding){btn.topHolding=true;btn.topPressStart=now;}
    else if(now-btn.topPressStart>=1000&&sys.currentMode==MODE_WIRELESS&&!sys.wirelessShowDetail){
      btn.topHolding=false;
      Serial.println("[BTN TOP HOLD] Wireless scan...");
      playWifiScan();
      readWireless();tftUpdate();
    }
    // Hook A: TOP-hold in FILES mode opens WiFi submenu
    else if(now-btn.topPressStart>=1000 && sys.currentMode==MODE_FILES
            && !sys.filesShowDetail && wifiSt.mode==WIFI_OFF_STATE) {
      btn.topHolding=false;
      Serial.println("[BTN TOP HOLD] Enter WiFi submenu");
      playBtnTopHold();
      wifiEnter();
      tftDrawHeader();
      tftUpdate();
    }
  } else {
    if(btn.topHolding&&now-btn.topPressStart<1000){
      btn.lastMs=now;
      if(sys.currentMode==MODE_THERMAL){
        sys.thermalAbsolute=!sys.thermalAbsolute;
        Serial.printf("[BTN TOP] Thermal: %s\n",sys.thermalAbsolute?"ABS":"REL");playBtnTop();
      } else if(sys.currentMode==MODE_WIRELESS&&!sys.wirelessShowDetail){
        sys.wirelessShowBLE=!sys.wirelessShowBLE;
        Serial.printf("[BTN TOP] Wireless: %s\n",sys.wirelessShowBLE?"BLE":"WiFi");playBtnTop();tftUpdate();
      } else if(sys.currentMode==MODE_MEASURE){
        Serial.println("[BTN TOP] Mag cal...");runMagCalibration();
      } else if(sys.currentMode==MODE_FILES&&!sys.filesShowDetail){
        filesGoUp();Serial.printf("[BTN TOP] Up → %s\n",filesData.currentPath);playBtnTop();tftUpdate();
      } else if(sys.currentMode==MODE_SOUND){
        if(soundRecording){Serial.println("[BTN TOP] Stop rec");sdStopRecording();}
        else              {Serial.println("[BTN TOP] Start rec");sdStartRecording();}
      } else {
        Serial.println("[BTN TOP] Pressed");playBtnTop();
      }
    }
    btn.topHolding=false;
  }
  btn.lastTop=bTop;

  // BOTTOM — freeze + CSV snapshot
  if(bBottom==LOW&&btn.lastBottom==HIGH){
    sys.snapshotFrozen=!sys.snapshotFrozen;btn.lastMs=now;
    if(sys.snapshotFrozen){
      playBtnBottom();
      sdWriteSnapshot(); // writes per-mode CSV + master log
      // Full diagnostic dump for current mode
      diagLog("DUMP","=== SNAPSHOT %s ===",MODE_NAMES[sys.currentMode]);
      switch(sys.currentMode) {
        case MODE_MEASURE:
          diagLog("MEAS","Dist:%dmm (valid=%d)",measureData.distanceMM,measureData.distanceValid);
          diagLog("MEAS","Heading:%.1f deg  Calibrated:%d",measureData.heading,measureData.magCalibrated);
          diagLog("MEAS","Mag raw   X=%.2f Y=%.2f Z=%.2f uT",measureData.rawMagX,measureData.rawMagY,measureData.rawMagZ);
          diagLog("MEAS","Mag offset X=%.2f Y=%.2f Z=%.2f",measureData.magOffX,measureData.magOffY,measureData.magOffZ);
          diagLog("MEAS","Mag calib X=%.2f Y=%.2f Z=%.2f uT",measureData.magX,measureData.magY,measureData.magZ);
          diagLog("MEAS","Expected magnitude in Texas ~25-30uT. Actual:%.2f",
            sqrtf(measureData.magX*measureData.magX+measureData.magY*measureData.magY+measureData.magZ*measureData.magZ));
          diagLog("COLOR","APDS9999 R=%lu G=%lu B=%lu IR=%lu  (%s)",
            measureData.colorR,measureData.colorG,measureData.colorB,
            measureData.colorIR, measureData.apdsOk?"OK":"NO RESPONSE");
          if(!measureData.apdsOk) diagLog("COLOR","APDS9999 not responding — check CH5 wiring");
          break;
        case MODE_ENVIRONMENTAL:
          diagLog("ENV","T:%.2fC H:%.2f%% P:%.2fhPa Gas:%.2fkOhm UVraw:%.0f UVI:%.4f",
            envData.temperature,envData.humidity,envData.pressure,envData.gasKOhm,
            envData.uvRaw,envData.uvIndex);
          break;
        case MODE_THERMAL:
          diagLog("THERM","Ctr:%.1f Min:%.1f Max:%.1f Delta:%.1f Mode:%s",
            thermalData.centerTemp,thermalData.minTemp,thermalData.maxTemp,
            thermalData.deltaTemp,sys.thermalAbsolute?"ABS":"REL");
          break;
        case MODE_SOUND:
          diagLog("SND","RMS:%.4f Peak:%.4f Freq~%dHz Recording:%s Bytes:%lu",
            soundData.rmsLevel,soundData.peakLevel,(int)soundDomFreqHz,
            soundRecording?"ON":"off",recByteCount);
          break;
        case MODE_WIRELESS:
          diagLog("WIFI","Count:%d  View:%s",wirelessData.wifiCount,sys.wirelessShowBLE?"BLE":"WiFi");
          for(int i=0;i<wirelessData.wifiCount;i++)
            diagLog("WIFI","  [%d] %s  %ddBm  ch%d",i,wirelessData.wifiSSID[i],wirelessData.wifiRSSI[i],wirelessData.wifiChannel[i]);
          break;
        default:break;
      }
      tftDrawHeader();
    } else {
      playBtnBottom();
      tftDrawHeader();
      diagLog("FRZ","Unfrozen");
    }
  }
  btn.lastBottom=bBottom;
}

// ================================================================
//  PDM MIC
//
//  DMA buffers sized for ~745ms of headroom: 16 buffers × 1024 samples
//  × 2 bytes = ~32KB total, ~32KB/sample-rate-44KB/s ≈ 0.74s. This
//  gives the main loop plenty of slack to handle SD writes / WiFi
//  ticks without losing samples to overflow.
// ================================================================
void pdmMicInit() {
  const i2s_config_t cfg={
    .mode=(i2s_mode_t)(I2S_MODE_MASTER|I2S_MODE_RX|I2S_MODE_PDM),
    .sample_rate=22050,.bits_per_sample=I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format=I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format=I2S_COMM_FORMAT_STAND_PCM_SHORT,
    .intr_alloc_flags=ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count=16,.dma_buf_len=1024,
    .use_apll=false,.tx_desc_auto_clear=false,.fixed_mclk=0
  };
  const i2s_pin_config_t pins={
    .mck_io_num=I2S_PIN_NO_CHANGE,.bck_io_num=I2S_PIN_NO_CHANGE,
    .ws_io_num=PDM_CLK_PIN,.data_out_num=I2S_PIN_NO_CHANGE,.data_in_num=PDM_DATA_PIN
  };
  esp_err_t r=i2s_driver_install(I2S_NUM_0,&cfg,0,NULL);
  if(r==ESP_OK) i2s_set_pin(I2S_NUM_0,&pins);
  Serial.printf("[PDM] %s\n",r==ESP_OK?"OK":"FAILED");
}

// ================================================================
//  SENSOR INIT
// ================================================================
void initSensors() {
  Serial.println("[SENSORS] Init...");
  switchMux(CH_APDS);
  if(apdsBegin()){
    Serial.println("  APDS9999 OK (native driver, addr 0x52)");
  } else {
    Serial.println("  APDS9999 FAILED — no response at 0x52");
  }
  switchMux(CH_LIS);
  if(lis2mdl.begin()) Serial.println("  LIS2MDL OK");else Serial.println("  LIS2MDL FAILED");
  switchMux(CH_LTR);
  if(ltr.begin()){ltr.setMode(LTR390_MODE_UVS);ltr.setGain(LTR390_GAIN_3);ltr.setResolution(LTR390_RESOLUTION_16BIT);Serial.println("  LTR390 OK");}
  else Serial.println("  LTR390 FAILED");
  switchMux(CH_BME);
  if(bme.begin()){bme.setTemperatureOversampling(BME680_OS_8X);bme.setHumidityOversampling(BME680_OS_2X);bme.setPressureOversampling(BME680_OS_4X);bme.setIIRFilterSize(BME680_FILTER_SIZE_3);bme.setGasHeater(320,150);Serial.println("  BME688 OK");}
  else Serial.println("  BME688 FAILED");
  switchMux(CH_AMG);
  if(amg.begin()) Serial.println("  AMG8833 OK");else Serial.println("  AMG8833 FAILED");
  switchMux(CH_VL53);tof.VL53L4CX_Off();
  if(tof.InitSensor(0x52)==0){tof.VL53L4CX_StartMeasurement();Serial.println("  VL53L4CX OK");}
  else Serial.println("  VL53L4CX FAILED");
  muxDisable();
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
  Serial.begin(115200);delay(500);
  Serial.println("\n=== TRICORDER v3.8.2 ===");
  Serial.println("  BOTTOM = freeze + CSV snapshot (/logs/log.csv)");
  Serial.println("  SOUND mode: TOP tap = WAV record toggle (/tricorder/audio/)");
  Serial.println("  FILES mode: TOP hold 1s = WiFi submenu (HTTP file browser)");
  Serial.println("  AUDIO: WAV SFX from /tricorder/sounds/, beep fallback");
  Serial.printf("PSRAM: %s (%.0fKB)\n",psramFound()?"OK":"NOT FOUND",psramFound()?ESP.getFreePsram()/1024.0f:0);

  pinMode(BTN_RIGHT,INPUT_PULLUP);pinMode(BTN_LEFT,INPUT_PULLUP);
  pinMode(BTN_TOP,  INPUT_PULLUP);pinMode(BTN_BOTTOM,INPUT_PULLUP);
  pinMode(ENC_SW,   INPUT_PULLUP);pinMode(ENC_CLK,INPUT_PULLUP);pinMode(ENC_DT,INPUT_PULLUP);
  pinMode(LED_GROUP,OUTPUT);

  ledInit();ledcWrite(LEDC_LED_CH,200);

  SPI.begin(SPI_SCK,SPI_MISO,SPI_MOSI);
  tft.initR(INITR_BLACKTAB);tft.setRotation(3);tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_CYAN);tft.setTextSize(1);tft.setCursor(20,55);tft.print("TRICORDER BOOTING...");
  Serial.println("[TFT] OK");

  Wire.begin(I2C_SDA,I2C_SCL);Wire.setClock(100000);
  matrixSelect();matrix.begin(0x70);matrix.setRotation(3);matrix.setBrightness(5);
  matrixClear();matrixWriteDisplay();Serial.println("[MATRIX] OK");matrixDeselect();

  audioInit();
  sdInit();         // prints [SD] OK + card size, or FAILED
  diagLogBoot();    // writes boot info to /tricorder/config/diag.log
  initSensors();
  playBoot();
  pdmMicInit();

  WiFi.mode(WIFI_STA);WiFi.disconnect();
  BLEDevice::init("Tricorder");

  ledcWrite(LEDC_LED_CH,0);
  tft.fillScreen(ST77XX_BLACK);tftDrawHeader();tftDrawLabels();
  Serial.println("[BOOT] Ready.");
  Serial.println("  LEFT/RIGHT=mode  BOTTOM=freeze+snapshot");
  Serial.println("  SOUND: TOP=rec  WIRELESS: TOP tap=WiFi/BLE  hold=scan");
  Serial.println("  THERMAL: TOP=ABS/REL  FILES: TOP=up  click=drill  TOPhold=WiFi");

  // Once boot.wav finishes, the main loop will kick off the scan SFX
  // for the starting mode (ENV by default).
  sys.scanLoopWanted = modeHasScanLoop(sys.currentMode);
}

// ================================================================
//  LOOP
// ================================================================
void loop() {
  handleEncoder();
  handleButtons();

  // Continuously drain the PDM mic in SOUND mode so recordings don't
  // lose chunks between sensor ticks. Non-blocking: returns immediately
  // if the I2S DMA ring is empty.
  if(sys.currentMode == MODE_SOUND && wifiSt.mode == WIFI_OFF_STATE) {
    readSoundFast();
  }

  // Sound scope refresh — runs at ~30fps, independent of the 500ms
  // sensor cycle, so the waveform actually feels live.
  static unsigned long lastScopeRefresh = 0;
  if(sys.currentMode == MODE_SOUND && wifiSt.mode == WIFI_OFF_STATE
     && millis() - lastScopeRefresh > 33) {
    lastScopeRefresh = millis();
    tftDrawSoundScope();
  }

  // Scan loop maintenance:
  // sys.scanLoopWanted stays true for the lifetime of any mode that has
  // a scan SFX. Whenever audio goes idle (a click finished, the previous
  // scan loop was preempted, etc.) and we're not frozen, restart the
  // scan loop. Mode change overwrites the wanted flag for the new mode.
  // Freeze sets it false; unfreeze sets it true again.
  if(sys.scanLoopWanted && !audioBusy() && !sys.snapshotFrozen) {
    playModeEntrySfx(sys.currentMode);
  }

  // Hook D: WiFi service ticks
  if(wifiSt.mode == WIFI_STA_CONNECTING) {
    wifiTickConnecting();
  } else if(wifiSt.mode == WIFI_AP_ACTIVE
         || wifiSt.mode == WIFI_AP_SETUP
         || wifiSt.mode == WIFI_STA_ACTIVE) {
    wifiTickActive();
  }

  if(millis()-sys.lastLedUpdate>30){ledUpdate();sys.lastLedUpdate=millis();}

  if(millis()-sys.lastMatrixUpdate>100){
    sys.matrixFrame++;
    if(sys.currentMode!=MODE_THERMAL){
      matrixSelect();matrixClear();
      // Hook G: when WiFi is active (not just menu), show wireless rings
      if(wifiSt.mode != WIFI_OFF_STATE && wifiSt.mode != WIFI_MENU) {
        drawModeWireless(sys.matrixFrame);
      } else {
        switch(sys.currentMode){
          case MODE_ENVIRONMENTAL:drawModeEnvironmental(sys.matrixFrame);break;
          case MODE_MEASURE:      drawModeMeasure(sys.matrixFrame);break;
          case MODE_SOUND:        drawModeSound(sys.matrixFrame);break;
          case MODE_BIO:          drawModeBio(sys.matrixFrame);break;
          case MODE_WIRELESS:     drawModeWireless(sys.matrixFrame);break;
          case MODE_FILES:        drawModeFiles(sys.matrixFrame);break;
          default:break;
        }
      }
      matrixWriteDisplay();matrixDeselect();
    }
    sys.lastMatrixUpdate=millis();
  }

  // Periodic refresh of WiFi UI (clock, request count, connecting countdown)
  static unsigned long lastWifiTftRefresh = 0;
  if(wifiSt.mode != WIFI_OFF_STATE && wifiSt.mode != WIFI_MENU
     && millis() - lastWifiTftRefresh > 1000) {
    lastWifiTftRefresh = millis();
    tftDrawHeader();
    wifiDrawTFT();
  }

  if(!sys.snapshotFrozen&&millis()-sys.lastSensorUpdate>sys.sensorIntervalMs){
    // Skip sensor reads while WiFi is active to keep HTTP responsive
    // and reduce CPU load.
    if(wifiSt.mode == WIFI_OFF_STATE) {
      switch(sys.currentMode){
        case MODE_ENVIRONMENTAL:readEnvironmental();break;
        case MODE_MEASURE:      readMeasure();break;
        case MODE_SOUND:        readSound();break;  // also writes PCM if recording
        case MODE_THERMAL:      readThermal();break;
        case MODE_BIO:case MODE_WIRELESS:case MODE_FILES:break;
      }
      tftUpdate();serialPrint();
    }
    sys.lastSensorUpdate=millis();
  }
}
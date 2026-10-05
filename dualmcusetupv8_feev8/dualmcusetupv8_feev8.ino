// =====================================================================
//  ESP32-S3 voice/audio node
//  Receives motion-state flags from F4, plays SD-card WAV clips.
// =====================================================================

#include "ESP_I2S.h"
#include "esp_heap_caps.h"
#include "SD_MMC.h"
#include "FS.h"
#include "communications.h"
#include "comms_protocol.h"

// ==================== Pin definitions ====================
#define I2S_MIC_BCLK  41
#define I2S_MIC_WS    42
#define I2S_MIC_DATA   2

#define I2S_SPK_BCLK  21
#define I2S_SPK_LRC   14
#define I2S_SPK_DIN   47

#define BUTTON_PIN    12

// ---- UART to F4 ----
#define UART_TX_PIN   19
#define UART_RX_PIN   20

// ---- SD card ----
#define SD_CLK_PIN    39
#define SD_CMD_PIN    38
#define SD_D0_PIN     40

// ==================== Audio settings ====================
#define SAMPLE_RATE   16000
#define RECORD_SEC    2.5
#define BUFFER_SIZE   (SAMPLE_RATE * RECORD_SEC)

#define PLAYBACK_DIR  "/playback"

// ==================== Audio amplification ====================
// 1 = unity (no change)
// 2 = +6 dB
// 4 = +12 dB
// 8 = +18 dB (typically starts clipping on loud clips)
#define AUDIO_GAIN   8

int16_t *record_buffer = nullptr;
size_t   record_samples = 0;

I2SClass i2sMic;
I2SClass i2sSpk;

// ==================== Shared state ====================
struct RobotStatus {
    uint8_t  flags        = 0;
    uint32_t timestamp_ms = 0;
};

RobotStatus       robotStatus;
SemaphoreHandle_t statusMutex;

volatile uint8_t  pendingCmd = CMD_NONE;
SemaphoreHandle_t cmdMutex;

// ==================== Prototypes ====================
void commsTask(void* param);
void audioTask(void* param);
void record();
void play();
void playWavFile(const char* path);
const char* selectClip(uint8_t flags);

// ---- Helper: queue a command for F4 (unused for now) ----
void setPendingCommand(uint8_t cmd) {
    xSemaphoreTake(cmdMutex, portMAX_DELAY);
    pendingCmd = cmd;
    xSemaphoreGive(cmdMutex);
}

// ---- Helper: read latest F4 status ----
RobotStatus getRobotStatus() {
    RobotStatus s;
    xSemaphoreTake(statusMutex, portMAX_DELAY);
    s = robotStatus;
    xSemaphoreGive(statusMutex);
    return s;
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("S3 voice node starting...");
    Serial.printf("[AUDIO] gain = %u\n", (unsigned)AUDIO_GAIN);

    // ---- RGB LED (releases GPIO38 if the boot ROM claimed it) ----
    #ifdef RGB_BUILTIN
        pinMode(RGB_BUILTIN, OUTPUT);
        rgbLedWrite(RGB_BUILTIN, 0, 0, 0);
    #endif

    // ---- SD card ----
    if (!SD_MMC.setPins(39, 38, 40)) {
        Serial.println("[SD] setPins failed");
        while (1);
    }
    if (!SD_MMC.begin("/sdcard", true)) {
        Serial.println("[SD] mount failed — halting for debug");
        while (1);
    }
    Serial.println("[SD] card ready.");

    // ---- PSRAM buffer ----
    size_t buffer_bytes = BUFFER_SIZE * sizeof(int16_t);
    record_buffer = (int16_t*)heap_caps_malloc(buffer_bytes, MALLOC_CAP_SPIRAM);
    if (!record_buffer) {
        record_buffer = (int16_t*)heap_caps_malloc(buffer_bytes, MALLOC_CAP_8BIT);
        if (!record_buffer) {
            Serial.println("FATAL: No memory!");
            while (1);
        }
    }
    Serial.printf("Buffer: %u KB from %s\n",
                  (unsigned)(buffer_bytes / 1024),
                  esp_ptr_external_ram(record_buffer) ? "PSRAM" : "DRAM");

    // ---- I2S mic ----
    i2sMic.setPins(I2S_MIC_BCLK, I2S_MIC_WS, -1, I2S_MIC_DATA);
    if (!i2sMic.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT,
                      I2S_SLOT_MODE_MONO, I2S_STD_SLOT_LEFT)) {
        Serial.println("Mic init failed");
    }

    // ---- I2S speaker ----
    i2sSpk.setPins(I2S_SPK_BCLK, I2S_SPK_LRC, I2S_SPK_DIN, -1);
    if (!i2sSpk.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_32BIT,
                      I2S_SLOT_MODE_STEREO)) {
        Serial.println("Speaker init failed");
        while (1);
    }

    // ---- Button ----
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // ---- UART to F4 ----
    commSerial.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    Serial.printf("UART to F4: TX=%d RX=%d @115200\n", UART_TX_PIN, UART_RX_PIN);

    // ---- Mutexes ----
    cmdMutex    = xSemaphoreCreateMutex();
    statusMutex = xSemaphoreCreateMutex();

    // ---- Tasks pinned to core 0 ----
    xTaskCreatePinnedToCore(commsTask, "Comms", 4096, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(audioTask, "Audio", 4096, NULL, 2, NULL, 0);

    Serial.println("Ready.");
}

// ==================== Main loop ====================
void loop() {
    delay(100);
}

// ==================== Record / Play (mic — retained, unused) ====================
void record() {
    record_samples = 0;
    while (record_samples < BUFFER_SIZE) {
        int16_t s;
        i2sMic.readBytes((char*)&s, sizeof(s));
        record_buffer[record_samples++] = s;
    }
}

void play() {
    if (record_samples == 0) return;
    for (size_t i = 0; i < record_samples; i++) {
        int16_t raw = record_buffer[i];

        // ---- Apply gain with hard clipping ----
        int32_t amplified = (int32_t)raw * AUDIO_GAIN;
        if (amplified > 32767)  amplified = 32767;
        if (amplified < -32768) amplified = -32768;
        int16_t s = (int16_t)amplified;

        int32_t w = ((int32_t)s) << 16;
        i2sSpk.write((uint8_t*)&w, sizeof(w));
        i2sSpk.write((uint8_t*)&w, sizeof(w));
    }
}

// ==================== WAV Playback ====================
// Assumes a canonical 44-byte WAV header. Streams PCM samples to i2sSpk
// with AUDIO_GAIN applied per sample.
void playWavFile(const char* path) {
    File f = SD_MMC.open(path, FILE_READ);
    if (!f) {
        Serial.printf("[AUDIO] open failed: %s\n", path);
        return;
    }

    uint8_t hdr[44];
    if (f.read(hdr, 44) != 44) {
        Serial.printf("[AUDIO] short header: %s\n", path);
        f.close();
        return;
    }

    if (memcmp(hdr, "RIFF", 4) != 0 || memcmp(hdr + 8, "WAVE", 4) != 0) {
        Serial.printf("[AUDIO] not a WAV: %s\n", path);
        f.close();
        return;
    }

    uint16_t channels      = *(uint16_t*)(hdr + 22);
    uint32_t sampleRate    = *(uint32_t*)(hdr + 24);
    uint16_t bitsPerSample = *(uint16_t*)(hdr + 34);

    if (channels != 1 || bitsPerSample != 16 || sampleRate != SAMPLE_RATE) {
        Serial.printf("[AUDIO] warning: %s — %uHz %uch %ubit (expected %uHz 1ch 16bit)\n",
                      path, (unsigned)sampleRate, channels, bitsPerSample,
                      (unsigned)SAMPLE_RATE);
        // Best-effort playback anyway.
    }

    // Stream samples: 16-bit mono → left-justified 32-bit stereo
    uint8_t buf[512];
    size_t  got;
    while ((got = f.read(buf, sizeof(buf))) > 0) {
        size_t samples = got / 2;
        for (size_t i = 0; i < samples; i++) {
            int16_t raw = (int16_t)((uint16_t)buf[i * 2] |
                                    ((uint16_t)buf[i * 2 + 1] << 8));

            // ---- Apply gain with hard clipping ----
            int32_t amplified = (int32_t)raw * AUDIO_GAIN;
            if (amplified > 32767)  amplified = 32767;
            if (amplified < -32768) amplified = -32768;
            int16_t s = (int16_t)amplified;

            int32_t w = ((int32_t)s) << 16;   // left-justify into 32-bit slot
            i2sSpk.write((uint8_t*)&w, sizeof(w));   // left
            i2sSpk.write((uint8_t*)&w, sizeof(w));   // right
        }
    }
    f.close();
}

// ==================== Clip selection ====================
const char* selectClip(uint8_t flags) {
    // Priority order — halt wins over obstacle, obstacle wins over motion.
    if (flags & ST_HALTED)                    return PLAYBACK_DIR "/stopping.wav";
    if (flags & ST_OBSTACLE_F)                return PLAYBACK_DIR "/obstacle detected.wav";
    if (flags & ST_OBSTACLE_L)                return PLAYBACK_DIR "/obstacle detected.wav";
    if (flags & ST_OBSTACLE_R)                return PLAYBACK_DIR "/obstacle detected.wav";
    if (flags & ST_FORWARD)                   return PLAYBACK_DIR "/going forward.wav";
    if (flags & ST_BACKWARD)                  return PLAYBACK_DIR "/going backwards.wav";
    if (flags & ST_TURN_LEFT)                 return PLAYBACK_DIR "/turning left.wav";
    if (flags & ST_TURN_RIGHT)                return PLAYBACK_DIR "/turning right.wav";
    return nullptr;   // idle → silent
}

// ==================== Comms Task (S3 <- F4) ====================
void commsTask(void* param) {
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20);

    commState state = WAIT_START;
    uint8_t   rxType = 0, rxLen = 0, rxIdx = 0;
    uint8_t   rxBuf[16], hdr[3];

    uint32_t pktsOK = 0, pktsBad = 0;
    uint32_t lastPrint = 0;
    const uint32_t PRINT_PERIOD_MS = 1000;

    for (;;) {
        uint32_t now = millis();

        while (commSerial.available()) {
            uint8_t b = (uint8_t)commSerial.read();

            switch (state) {
                case WAIT_START:
                    if (b == PKT_START_BYTE) { hdr[0] = b; state = WAIT_TYPE; }
                    break;
                case WAIT_TYPE:
                    rxType = b; hdr[1] = b; state = WAIT_LEN;
                    break;
                case WAIT_LEN:
                    rxLen = b; hdr[2] = b; rxIdx = 0;
                    if (rxLen == 0)                  state = WAIT_CHECKSUM;
                    else if (rxLen <= sizeof(rxBuf)) state = WAIT_PAYLOAD;
                    else                             state = WAIT_START;
                    break;
                case WAIT_PAYLOAD:
                    rxBuf[rxIdx++] = b;
                    if (rxIdx >= rxLen) state = WAIT_CHECKSUM;
                    break;
                case WAIT_CHECKSUM: {
                    uint8_t calc = computeChecksum(hdr, 3);
                    for (uint8_t i = 0; i < rxLen; i++) calc ^= rxBuf[i];

                    if (calc == b) {
                        pktsOK++;
                        if (rxType == PKT_TYPE_STATUS && rxLen == 1) {
                            uint8_t flags = rxBuf[0];
                            xSemaphoreTake(statusMutex, portMAX_DELAY);
                            robotStatus.flags        = flags;
                            robotStatus.timestamp_ms = now;
                            xSemaphoreGive(statusMutex);

                            Serial.printf("[F4] flags=0x%02X", flags);
                            if (flags == 0)             Serial.print(" (idle)");
                            if (flags & ST_FORWARD)     Serial.print(" FWD");
                            if (flags & ST_BACKWARD)    Serial.print(" BACK");
                            if (flags & ST_TURN_LEFT)   Serial.print(" TL");
                            if (flags & ST_TURN_RIGHT)  Serial.print(" TR");
                            if (flags & ST_OBSTACLE_L)  Serial.print(" OBST_L");
                            if (flags & ST_OBSTACLE_R)  Serial.print(" OBST_R");
                            if (flags & ST_OBSTACLE_F)  Serial.print(" OBST_F");
                            if (flags & ST_HALTED)      Serial.print(" HALTED");
                            Serial.println();
                        }
                    } else {
                        pktsBad++;
                    }
                    state = WAIT_START;
                    break;
                }
                default:
                    state = WAIT_START;
                    break;
            }
        }

        if (now - lastPrint >= PRINT_PERIOD_MS) {
            lastPrint = now;
            Serial.printf("[COMMS] %lu ok, %lu bad | flags=0x%02X | heap=%lu\n",
                          (unsigned long)pktsOK, (unsigned long)pktsBad,
                          robotStatus.flags,
                          (unsigned long)xPortGetFreeHeapSize());
            pktsOK = 0; pktsBad = 0;
        }

        vTaskDelayUntil(&lastWake, period);
    }
}

// ==================== Audio Task ====================
// Debounced edge-triggered playback:
//   - When flags change, reset a timer.
//   - After DEBOUNCE_MS of stable flags, if they differ from the last
//     flags we played a clip for, play the matching WAV.
//   - Idle (0x00) is silent.
void audioTask(void* param) {
    const TickType_t period = pdMS_TO_TICKS(50);
    TickType_t lastWake = xTaskGetTickCount();

    const uint32_t DEBOUNCE_MS = 400;

    uint8_t  lastObserved   = 0xFF;   // force first evaluation
    uint8_t  lastPlayed     = 0xFF;
    uint32_t lastChangeMs   = 0;

    // Boot chime
    playWavFile(PLAYBACK_DIR "/starting.wav");
    lastPlayed = 0x00;

    for (;;) {
        uint8_t flags = 0;
        xSemaphoreTake(statusMutex, portMAX_DELAY);
        flags = robotStatus.flags;
        xSemaphoreGive(statusMutex);

        uint32_t now = millis();

        // Reset debounce timer whenever flags change
        if (flags != lastObserved) {
            lastObserved = flags;
            lastChangeMs = now;
        }

        // Fire playback once flags have settled AND they're new
        if ((now - lastChangeMs) >= DEBOUNCE_MS && flags != lastPlayed) {
            const char* clip = selectClip(flags);
            if (clip) {
                Serial.printf("[AUDIO] play %s\n", clip);
                playWavFile(clip);
            } else {
                Serial.println("[AUDIO] idle — silent");
            }
            lastPlayed = flags;
        }

        vTaskDelayUntil(&lastWake, period);
    }
}
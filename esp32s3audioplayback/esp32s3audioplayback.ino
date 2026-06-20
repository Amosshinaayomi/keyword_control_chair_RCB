// #include "ESP_I2S.h"

// // ---------- Pin definitions ----------
// // Microphone (I2S0)
// #define I2S_MIC_BCLK  41
// #define I2S_MIC_WS    42
// #define I2S_MIC_DATA  2

// // Speaker (I2S1)
// #define I2S_SPK_BCLK  21
// #define I2S_SPK_LRC   14
// #define I2S_SPK_DIN   47

// // Button
// #define BUTTON_PIN    12   // or 22, 0, etc.

// // ---------- Audio settings ----------
// #define SAMPLE_RATE   16000
// #define RECORD_SEC    10               // max recording time
// #define BUFFER_SIZE   (SAMPLE_RATE * RECORD_SEC)

// int16_t record_buffer[BUFFER_SIZE];
// size_t record_samples = 0;

// I2SClass i2sMic;
// I2SClass i2sSpk;

// void setup() {
//   Serial.begin(115200);
//   delay(1000);
//   Serial.println("Record/playback demo with ESP_I2S");

//   pinMode(BUTTON_PIN, INPUT_PULLUP);

//   // ---------- Microphone: 16 kHz, 16-bit, mono, left slot ----------
//   i2sMic.setPins(I2S_MIC_BCLK, I2S_MIC_WS, -1, I2S_MIC_DATA);
//   if (!i2sMic.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT,
//                     I2S_SLOT_MODE_MONO, I2S_STD_SLOT_LEFT)) {
//     Serial.println("Mic init failed!");
//     while (1) {}
//   }

//   // ---------- Speaker: 16 kHz, 32-bit slot, stereo ----------
//   // (32-bit slot width matches what the MAX98357 needs)
//   i2sSpk.setPins(I2S_SPK_BCLK, I2S_SPK_LRC, I2S_SPK_DIN, -1);
//   if (!i2sSpk.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_32BIT,
//                     I2S_SLOT_MODE_STEREO)) {
//     Serial.println("Speaker init failed!");
//     while (1) {}
//   }

//   Serial.println("Ready. Hold button to record, release to play.");
// }

// void loop() {
//   static bool was_pressed = false;
//   bool pressed = !digitalRead(BUTTON_PIN);

//   if (pressed && !was_pressed) {
//     // ---------- Record ----------
//     record_samples = 0;
//     Serial.println("Recording...");

//     while (digitalRead(BUTTON_PIN) == LOW && record_samples < BUFFER_SIZE) {
//       size_t bytes_read;
//       int16_t sample;
//       i2sMic.readBytes((char*)&sample, sizeof(sample));
//       record_buffer[record_samples++] = sample;
//     }
//     Serial.printf("Recorded %d samples (%.1f s)\n", record_samples, (float)record_samples / SAMPLE_RATE);
//   }
//   else if (!pressed && was_pressed) {
//     // ---------- Playback ----------
//     if (record_samples == 0) { was_pressed = pressed; delay(10); return; }

//     Serial.printf("Playing %d samples...\n", record_samples);
//     for (size_t i = 0; i < record_samples; i++) {
//       int16_t raw = record_buffer[i];

//       // Apply 8× digital gain for volume boost
//       int32_t amplified = (int32_t)raw * 8;
//       if (amplified > 32767) amplified = 32767;
//       if (amplified < -32768) amplified = -32768;
//       int16_t sample = (int16_t)amplified;

//       // Left-align 16-bit sample in 32-bit word
//       int32_t word = ((int32_t)sample) << 16;

//       // Write stereo: left channel then right channel (both identical)
//       // The I2S peripheral will output left first (standard I2S)
//       i2sSpk.write((uint8_t*)&word, sizeof(word));   // left
//       i2sSpk.write((uint8_t*)&word, sizeof(word));   // right
//     }
//     Serial.println("Playback done.");
//   }

//   was_pressed = pressed;
//   delay(10);
// }


#include "ESP_I2S.h"
#include "esp_heap_caps.h"   // for PSRAM allocation

// ---------- Pin definitions ----------
// Microphone (I2S0)
#define I2S_MIC_BCLK  41
#define I2S_MIC_WS    42
#define I2S_MIC_DATA  2

// Speaker (I2S1)
#define I2S_SPK_BCLK  21
#define I2S_SPK_LRC   14
#define I2S_SPK_DIN   47

// Button
#define BUTTON_PIN    12   // or 22, 0, etc.

// ---------- Audio settings ----------
#define SAMPLE_RATE   16000
#define RECORD_SEC    10                // you can increase even up to 60
#define BUFFER_SIZE   (SAMPLE_RATE * RECORD_SEC)   // samples

// ---------- Global pointer (will point to PSRAM) ----------
int16_t *record_buffer = nullptr;
size_t record_samples = 0;

I2SClass i2sMic;
I2SClass i2sSpk;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Record/playback demo with PSRAM");

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // ---------- Allocate recording buffer in PSRAM ----------
  size_t buffer_bytes = BUFFER_SIZE * sizeof(int16_t);
  record_buffer = (int16_t*) heap_caps_malloc(buffer_bytes, MALLOC_CAP_SPIRAM);
  if (!record_buffer) {
    Serial.println("PSRAM allocation failed – falling back to internal DRAM");
    record_buffer = (int16_t*) heap_caps_malloc(buffer_bytes, MALLOC_CAP_8BIT);
    if (!record_buffer) {
      Serial.println("FATAL: No memory for buffer!");
      while (1) {}
    }
  }
  Serial.printf("Buffer allocated: %d KB from %s\n",
                buffer_bytes / 1024,
                esp_ptr_external_ram(record_buffer) ? "PSRAM" : "internal DRAM");

  // ---------- Microphone: 16 kHz, 16-bit, mono, left slot ----------
  i2sMic.setPins(I2S_MIC_BCLK, I2S_MIC_WS, -1, I2S_MIC_DATA);
  if (!i2sMic.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT,
                    I2S_SLOT_MODE_MONO, I2S_STD_SLOT_LEFT)) {
    Serial.println("Mic init failed!");
    while (1) {}
  }

  // ---------- Speaker: 16 kHz, 32-bit slot, stereo ----------
  i2sSpk.setPins(I2S_SPK_BCLK, I2S_SPK_LRC, I2S_SPK_DIN, -1);
  if (!i2sSpk.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_32BIT,
                    I2S_SLOT_MODE_STEREO)) {
    Serial.println("Speaker init failed!");
    while (1) {}
  }

  Serial.println("Ready. Hold button to record, release to play.");
}

void loop() {
  static bool was_pressed = false;
  bool pressed = !digitalRead(BUTTON_PIN);

  if (pressed && !was_pressed) {
    // ---------- Record ----------
    record_samples = 0;
    Serial.println("Recording...");

    while (digitalRead(BUTTON_PIN) == LOW && record_samples < BUFFER_SIZE) {
      int16_t sample;
      i2sMic.readBytes((char*)&sample, sizeof(sample));
      record_buffer[record_samples++] = sample;
    }
    Serial.printf("Recorded %d samples (%.1f s)\n", record_samples, (float)record_samples / SAMPLE_RATE);
  }
  else if (!pressed && was_pressed) {
    // ---------- Playback with gain ----------
    if (record_samples == 0) { was_pressed = pressed; delay(10); return; }

    Serial.printf("Playing %d samples...\n", record_samples);
    for (size_t i = 0; i < record_samples; i++) {
      int16_t raw = record_buffer[i];

      // 8× digital gain
      int32_t amplified = (int32_t)raw * 8;
      if (amplified > 32767) amplified = 32767;
      if (amplified < -32768) amplified = -32768;
      int16_t sample = (int16_t)amplified;

      int32_t word = ((int32_t)sample) << 16;   // left‑align in 32‑bit slot

      // Stereo: left channel, then right channel
      i2sSpk.write((uint8_t*)&word, sizeof(word));
      i2sSpk.write((uint8_t*)&word, sizeof(word));
    }
    Serial.println("Playback done.");
  }

  was_pressed = pressed;
  delay(10);
}
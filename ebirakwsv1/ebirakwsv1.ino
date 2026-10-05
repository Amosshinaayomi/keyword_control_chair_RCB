// ==========================================
// Edge Impulse KWS – Button Trigger + Playback
// ==========================================

// CRUCIAL: Forces TensorFlow to use your 8MB PSRAM
#define EI_CLASSIFIER_ALLOC_PSRAM 1
// Increase scratchpad if your model is large (adjust if needed)
#define EI_CLASSIFIER_TENSOR_ARENA_SIZE 350000
// Keep filterbank quantized for speed
#define EIDSP_QUANTIZE_FILTERBANK   0

/* Includes ---------------------------------------------------------------- */
#include <ebiramotionkws_inferencing.h>

#include "ESP_I2S.h"
#include "esp_heap_caps.h"

// ==================== Pin definitions ====================
#define I2S_MIC_BCLK  41
#define I2S_MIC_WS    42
#define I2S_MIC_DATA  2

#define I2S_SPK_BCLK  21
#define I2S_SPK_LRC   14
#define I2S_SPK_DIN   47

#define BUTTON_PIN    12

// ==================== Audio settings ====================
#define SAMPLE_RATE   16000
#define RECORD_SEC    1               // Model uses 1‑second windows
#define BUFFER_SIZE   (SAMPLE_RATE * RECORD_SEC)

// ==================== Global buffer (PSRAM) ====================
int16_t *record_buffer = nullptr;
size_t record_samples = 0;

I2SClass i2sMic;
I2SClass i2sSpk;

// ==================== Forward declarations ====================
void record();
void play();
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr);

// ==================== Setup ====================
void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("\n🎤 Edge Impulse KWS – Button Trigger");

  // ---- Allocate recording buffer in PSRAM ----
  size_t buffer_bytes = BUFFER_SIZE * sizeof(int16_t);
  record_buffer = (int16_t*)heap_caps_malloc(buffer_bytes, MALLOC_CAP_SPIRAM);
  if (!record_buffer) {
    record_buffer = (int16_t*)heap_caps_malloc(buffer_bytes, MALLOC_CAP_8BIT);
    if (!record_buffer) {
      Serial.println("FATAL: No memory for buffer!");
      while (1);
    }
  }
  Serial.printf("Buffer allocated: %d KB from %s\n",
                buffer_bytes / 1024,
                esp_ptr_external_ram(record_buffer) ? "PSRAM" : "internal DRAM");

  // ---- Microphone: 16 kHz, 16‑bit, mono, left slot ----
  i2sMic.setPins(I2S_MIC_BCLK, I2S_MIC_WS, -1, I2S_MIC_DATA);
  if (!i2sMic.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT,
                    I2S_SLOT_MODE_MONO, I2S_STD_SLOT_LEFT)) {
    Serial.println("Mic init failed!");
    while (1);
  }

  // ---- Speaker: 16 kHz, 32‑bit slot, stereo ----
  i2sSpk.setPins(I2S_SPK_BCLK, I2S_SPK_LRC, I2S_SPK_DIN, -1);
  if (!i2sSpk.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_32BIT,
                    I2S_SLOT_MODE_STEREO)) {
    Serial.println("Speaker init failed!");
    while (1);
  }

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // ---- Model info ----
  ei_printf("Inferencing settings:\n");
  ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
  ei_printf("\tSample length: %d ms.\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT / 16);
  ei_printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) / sizeof(ei_classifier_inferencing_categories[0]));

  Serial.println("Ready. Press the button to record 1 second, hear playback, then see inference.");
}

// ==================== Main Loop ====================
void loop() {
  static bool lastButtonState = HIGH;
  bool buttonState = digitalRead(BUTTON_PIN);

  // Detect button press (falling edge)
  if (buttonState == LOW && lastButtonState == HIGH) {
    // 1. Debounce / wait for the button click sound to pass
    Serial.println("\nPreparing... (1s delay to clear button click)");
    delay(1000);

    // 2. Record exactly 1 second
    Serial.println("🎤 Recording... (1 second)");
    rgbLedWrite(RGB_BUILTIN, 255, 255, 255);
    record();
    rgbLedWrite(RGB_BUILTIN, 0, 0, 0);
    // 3. Play it back so you can verify what the AI heard
    Serial.println("Playing back...");
    play();

    // 4. Run inference on the same buffer
    Serial.println("Running inference...");
    signal_t signal;
    signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
    signal.get_data = &microphone_audio_signal_get_data;
    ei_impulse_result_t result = { 0 };

    EI_IMPULSE_ERROR r = run_classifier(&signal, &result, false);
    if (r != EI_IMPULSE_OK) {
      ei_printf("ERR: Failed to run classifier (%d)\n", r);
    } else {
      ei_printf("Predictions (DSP: %d ms., Classification: %d ms.):\n",
                result.timing.dsp, result.timing.classification);

      float max_val = 0;
      int max_idx = 0;
      for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("    %s: ", result.classification[ix].label);
        ei_printf_float(result.classification[ix].value);
        ei_printf("\sn");
        if (result.classification[ix].value > max_val) {
          max_val = result.classification[ix].value;
          max_idx = ix;
        }
      }

      if (max_val > 0.60) {
        ei_printf("\nACTION TRIGGERED: %s\n", result.classification[max_idx].label);
        // ---- Insert your robot motor code here ----
      } else {
        ei_printf("\nLow confidence, ignoring.\n");
      }
    }
    Serial.println("Ready for next press.\n");
  }

  lastButtonState = buttonState;
  delay(10);
}

// ==================== Record (blocking, 1 second) ====================
void record() {
  record_samples = 0;
  while (record_samples < BUFFER_SIZE) {
    int16_t sample;
    i2sMic.readBytes((char*)&sample, sizeof(sample));
    record_buffer[record_samples++] = sample;
  }
}

// ==================== Playback (blocking, 1 second) ====================
void play() {
  if (record_samples == 0) return;
  for (size_t i = 0; i < record_samples; i++) {
    int16_t raw = record_buffer[i];
    int32_t amplified = (int32_t)raw * 8; // 8x gain
    if (amplified > 32767) amplified = 32767;
    if (amplified < -32768) amplified = -32768;
    int16_t sample = (int16_t)amplified;

    int32_t word = ((int32_t)sample) << 16; // left‑align in 32‑bit slot
    i2sSpk.write((uint8_t*)&word, sizeof(word)); // left
    i2sSpk.write((uint8_t*)&word, sizeof(word)); // right
  }
}

// ==================== Data getter for Edge Impulse ====================
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr) {
  numpy::int16_to_float(&record_buffer[offset], out_ptr, length);
  return 0;
}

// ==================== Sensor validation ====================
#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_MICROPHONE
#error "Invalid model for current sensor."
#endif
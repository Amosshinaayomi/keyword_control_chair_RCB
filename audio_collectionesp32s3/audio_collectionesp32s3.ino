#include "ESP_I2S.h"
#include "SD_MMC.h"
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
#define RECORD_SEC    10
#define BUFFER_SIZE   (SAMPLE_RATE * RECORD_SEC)

int16_t *record_buffer = nullptr;
size_t record_samples = 0;

I2SClass i2sMic;
I2SClass i2sSpk;

// ==================== File system state ====================
String currentPath = "/";

// ==================== CLI & recording state ====================
enum State { IDLE, RECORDING, PLAYING, SAVE_ASK, SAVE_GET_FILENAME };
State state = IDLE;
bool buttonWasPressed = false;
bool promptShown = false;

// ==================== Forward declarations ====================
void processCommand(String cmd);
void saveWav(String path);
void record();
void play();

// ==================== Setup ====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n📁 Audio CLI starting...");

  // ---- Mount SD card ----
  if (!SD_MMC.setPins(39, 38, 40)) {
    Serial.println("SD_MMC pin set failed!");
    while (1);
  }
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("SD card mount failed!");
    while (1);
  }
  Serial.println("SD card ready.");

  // ---- Allocate PSRAM buffer ----
  size_t buffer_bytes = BUFFER_SIZE * sizeof(int16_t);
  record_buffer = (int16_t*)heap_caps_malloc(buffer_bytes, MALLOC_CAP_SPIRAM);
  if (!record_buffer) {
    record_buffer = (int16_t*)heap_caps_malloc(buffer_bytes, MALLOC_CAP_8BIT);
    if (!record_buffer) {
      Serial.println("FATAL: No memory!");
      while (1);
    }
  }
  Serial.printf("Buffer: %d KB from %s\n", buffer_bytes / 1024,
                esp_ptr_external_ram(record_buffer) ? "PSRAM" : "DRAM");

  // ---- Init microphone (I2S0, 16-bit mono) ----
  i2sMic.setPins(I2S_MIC_BCLK, I2S_MIC_WS, -1, I2S_MIC_DATA);
  if (!i2sMic.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT,
                    I2S_SLOT_MODE_MONO, I2S_STD_SLOT_LEFT)) {
    Serial.println("Mic init failed!");
    while (1);
  }

  // ---- Init speaker (I2S1, 32-bit slot, stereo) ----
  i2sSpk.setPins(I2S_SPK_BCLK, I2S_SPK_LRC, I2S_SPK_DIN, -1);
  if (!i2sSpk.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_32BIT,
                    I2S_SLOT_MODE_STEREO)) {
    Serial.println("Speaker init failed!");
    while (1);
  }

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.println("Ready. Hold button to record, release to play.\n");
  promptShown = false;
}

// ==================== Main loop ====================
void loop() {
  // --- Button handling (state transitions) ---
  bool buttonPressed = !digitalRead(BUTTON_PIN);

  // Detect button press (falling edge)
  if (buttonPressed && !buttonWasPressed && state == IDLE) {
    state = RECORDING;
    record();                     // blocking
    state = PLAYING;
    play();                       // blocking
    // After playback, ask to save
    state = SAVE_ASK;
    Serial.println("\nSave recording? (y/n)");
    promptShown = false;
  }

  buttonWasPressed = buttonPressed;

  // --- Serial processing (depends on state) ---
  switch (state) {

    case IDLE:
      if (!promptShown) {
        Serial.print(currentPath + "> ");
        promptShown = true;
      }
      if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() > 0) {
          processCommand(line);
        }
        promptShown = false;   // re‑print prompt after command
      }
      break;

    case SAVE_ASK:
      if (Serial.available()) {
        char answer = Serial.read();
        // flush rest of line
        while (Serial.available()) Serial.read();
        if (answer == 'y' || answer == 'Y') {
          state = SAVE_GET_FILENAME;
          Serial.print("Enter filename (without path): ");
        } else {
          Serial.println("Discarded.");
          state = IDLE;
          promptShown = false;
        }
      }
      break;

    case SAVE_GET_FILENAME:
      if (Serial.available()) {
        String fname = Serial.readStringUntil('\n');
        fname.trim();
        if (fname.length() > 0) {
          String fullPath = currentPath;
          if (!fullPath.endsWith("/")) fullPath += "/";
          fullPath += fname + ".wav";
          saveWav(fullPath);
          state = IDLE;
          promptShown = false;
        } else {
          Serial.println("Invalid filename, try again.");
        }
      }
      break;

    // RECORDING and PLAYING are handled by blocking functions,
    // no serial processing needed.
    default:
      break;
  }

  delay(5); // small yield
}

// ==================== Record (blocking) ====================
void record() {
  record_samples = 0;
  Serial.println("🎤 Recording... (hold button)");
  while (digitalRead(BUTTON_PIN) == LOW && record_samples < BUFFER_SIZE) {
    int16_t sample;
    i2sMic.readBytes((char*)&sample, sizeof(sample));
    record_buffer[record_samples++] = sample;
  }
  Serial.printf("Recorded %d samples (%.1f s)\n", record_samples,
                (float)record_samples / SAMPLE_RATE);
}

// ==================== Playback with gain (blocking) ====================
void play() {
  if (record_samples == 0) return;
  Serial.println("🔊 Playing...");
  for (size_t i = 0; i < record_samples; i++) {
    int16_t raw = record_buffer[i];
    // 8× digital gain
    int32_t amplified = (int32_t)raw * 8;
    if (amplified > 32767) amplified = 32767;
    if (amplified < -32768) amplified = -32768;
    int16_t sample = (int16_t)amplified;

    int32_t word = ((int32_t)sample) << 16;   // left‑align in 32‑bit slot
    i2sSpk.write((uint8_t*)&word, sizeof(word));   // left channel
    i2sSpk.write((uint8_t*)&word, sizeof(word));   // right channel
  }
  delay(50);
  Serial.println("Playback finished.");
}

// ==================== Save WAV to SD ====================
void saveWav(String fullPath) {
  if (record_samples == 0) return;

  File f = SD_MMC.open(fullPath, FILE_WRITE);
  if (!f) {
    Serial.println("Could not open file for writing!");
    return;
  }

  // WAV header (16‑bit PCM, mono, 16 kHz)
  uint32_t dataSize = record_samples * 2;
  uint32_t fileSize = dataSize + 36;
  uint16_t audioFormat = 1;     // PCM
  uint16_t numChannels = 1;     // mono
  uint32_t byteRate = SAMPLE_RATE * numChannels * 2;   // 32000
  uint16_t blockAlign = numChannels * 2;                // 2
  uint16_t bitsPerSample = 16;

  auto writeLE = [&](uint32_t val, int bytes) {
    for (int i = 0; i < bytes; i++) {
      f.write((uint8_t)(val & 0xFF));
      val >>= 8;
    }
  };

  f.write((const uint8_t*)"RIFF", 4);
  writeLE(fileSize, 4);
  f.write((const uint8_t*)"WAVE", 4);
  f.write((const uint8_t*)"fmt ", 4);
  writeLE(16, 4);                 // subchunk1 size
  writeLE(audioFormat, 2);
  writeLE(numChannels, 2);
  writeLE(SAMPLE_RATE, 4);
  writeLE(byteRate, 4);
  writeLE(blockAlign, 2);
  writeLE(bitsPerSample, 2);
  f.write((const uint8_t*)"data", 4);
  writeLE(dataSize, 4);

  // PCM data
  for (size_t i = 0; i < record_samples; i++) {
    int16_t s = record_buffer[i];
    f.write((uint8_t*)&s, 2);
  }

  f.close();
  Serial.printf("💾 Saved as %s\n", fullPath.c_str());
}

// ==================== CLI command processor ====================
void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  // Split command and argument
  String command, argument;
  int spaceIdx = cmd.indexOf(' ');
  if (spaceIdx > 0) {
    command = cmd.substring(0, spaceIdx);
    argument = cmd.substring(spaceIdx + 1);
    argument.trim();
  } else {
    command = cmd;
  }

  if (command == "ls") {
    File dir = SD_MMC.open(currentPath);
    if (!dir || !dir.isDirectory()) {
      Serial.println("Cannot open directory.");
      return;
    }
    File entry = dir.openNextFile();
    if (!entry) {
      Serial.println("(empty)");
    }
    while (entry) {
      Serial.print(entry.name());
      if (entry.isDirectory()) {
        Serial.println("/");
      } else {
        Serial.print("\t");
        Serial.println(entry.size());
      }
      entry = dir.openNextFile();
    }
    dir.close();
  }
    else if (command == "cd") {
        String newPath;
        if (argument.length() == 0) {
            newPath = "/";   // go to root
        } else if (argument.startsWith("/")) {
            newPath = argument;
        } else if (argument == "..") {
            int lastSlash = currentPath.lastIndexOf('/');
            if (lastSlash == 0 && currentPath.length() > 1) {
                newPath = "/";
            } else if (lastSlash > 0) {
                newPath = currentPath.substring(0, lastSlash);
            } else {
                newPath = currentPath;
            }
        } else {
            newPath = currentPath;
            if (!newPath.endsWith("/")) newPath += "/";
            newPath += argument;
        }
        File testDir = SD_MMC.open(newPath);
        if (!testDir || !testDir.isDirectory()) {
            Serial.println("Directory does not exist.");
        } else {
            currentPath = newPath;
        }
        testDir.close();
    }
  else if (command == "mkdir") {
    if (argument.length() == 0) {
      Serial.println("Usage: mkdir <dir>");
      return;
    }
    String fullPath = currentPath;
    if (!fullPath.endsWith("/")) fullPath += "/";
    fullPath += argument;
    if (SD_MMC.mkdir(fullPath)) {
      Serial.println("Directory created.");
    } else {
      Serial.println("Failed to create directory.");
    }
  }
  else if (command == "pwd") {
    Serial.println(currentPath);
  }
  else {
    Serial.println("Unknown command. Available: ls, cd, mkdir, pwd");
  }
}
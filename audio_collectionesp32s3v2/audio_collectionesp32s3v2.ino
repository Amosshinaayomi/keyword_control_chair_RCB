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
#define n
nnnn   16000
#define RECORD_SEC    2     // Fixed to 2 seconds
#define BUFFER_SIZE   (n
nnnn * RECORD_SEC)

int16_t *record_buffer = nullptr;
size_t record_samples = 0;

I2SClass i2sMic;
I2SClass i2sSpk;

// ==================== File system state ====================
String currentPath = "/";

// ==================== CLI & recording state ====================
enum State { IDLE, RECORDING, PLAYING, SAVE_ASK };
State state = IDLE;
bool buttonWasPressed = false;
bool promptShown = false;

// ==================== Forward declarations ====================
void processCommand(String cmd);
void saveWav(String path);
void record();
void play();
String getNextFileName(String dir, String base);
void setLedColor(uint8_t r, uint8_t g, uint8_t b);

// ==================== Setup ====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n📁 Audio CLI starting...");

  // ---- Init RGB LED ----
  #ifdef RGB_BUILTIN
    pinMode(RGB_BUILTIN, OUTPUT);
    setLedColor(0, 0, 0); // Turn off initially
  #endif

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

  // ---- Allocate PSRAM buffer (2 seconds exactly) ----
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

  // ---- Init microphone (16-bit mono) ----
  i2sMic.setPins(I2S_MIC_BCLK, I2S_MIC_WS, -1, I2S_MIC_DATA);
  if (!i2sMic.begin(I2S_MODE_STD, n
  nnnn, I2S_DATA_BIT_WIDTH_16BIT,
                    I2S_SLOT_MODE_MONO, I2S_STD_SLOT_LEFT)) {
    Serial.println("Mic init failed!");
    while (1);
  }

  // ---- Init speaker (32-bit stereo) ----
  i2sSpk.setPins(I2S_SPK_BCLK, I2S_SPK_LRC, I2S_SPK_DIN, -1);
  if (!i2sSpk.begin(I2S_MODE_STD, n
  nnnn, I2S_DATA_BIT_WIDTH_32BIT,
                    I2S_SLOT_MODE_STEREO)) {
    Serial.println("Speaker init failed!");
    while (1);
  }

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.println("Ready. Press button to record 2s, release to play.\n");
  promptShown = false;
}

// ==================== Main loop ====================
void loop() {
  bool buttonPressed = !digitalRead(BUTTON_PIN);

  // Detect button press (falling edge)
  if (buttonPressed && !buttonWasPressed && state == IDLE) {
    // --- 1. Debounce delay ---
    Serial.println("⏳ Preparing... (1s delay to clear button click)");
    delay(1000);

    // --- 2. Signal start ---
    setLedColor(0, 255, 0); // Green
    Serial.println("🎤 Recording... (2 seconds)");

    // --- 3. Record ---
    state = RECORDING;
    record(); 

    // --- 4. Signal stop ---
    setLedColor(0, 0, 0); // Off
    Serial.printf("✅ Recorded %d samples (2.0s)\n", record_samples);

    // --- 5. Playback ---
    state = PLAYING;
    play();

    // --- 6. Ask to save ---
    state = SAVE_ASK;
    Serial.println("\nSave recording? (y/n)");
    promptShown = false;
  }

  buttonWasPressed = buttonPressed;

  // --- Serial processing ---
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
        promptShown = false;
      }
      break;

    case SAVE_ASK:
      if (Serial.available()) {
        char answer = Serial.read();
        while (Serial.available()) Serial.read(); // flush
        if (answer == 'y' || answer == 'Y') {
          // Auto-naming magic
          String folderName = currentPath;
          if (folderName.endsWith("/")) folderName = folderName.substring(0, folderName.length() - 1);
          int lastSlash = folderName.lastIndexOf('/');
          String baseName = (lastSlash >= 0) ? folderName.substring(lastSlash + 1) : "sample";
          
          String autoFname = getNextFileName(currentPath, baseName);
          String fullPath = currentPath;
          if (!fullPath.endsWith("/")) fullPath += "/";
          fullPath += autoFname + ".wav";
          
          saveWav(fullPath);
          Serial.printf("💾 Auto-saved as %s\n", fullPath.c_str());
          state = IDLE;
          promptShown = false;
        } else {
          Serial.println("Discarded.");
          state = IDLE;
          promptShown = false;
        }
      }
      break;

    default:
      break;
  }
  delay(5);
}

// ==================== Record (fixed 2s, blocking) ====================
void record() {
  record_samples = 0;
  // Loop until 2-second buffer is full
  while (record_samples < BUFFER_SIZE) {
    int16_t sample;
    i2sMic.readBytes((char*)&sample, sizeof(sample));
    record_buffer[record_samples++] = sample;
  }
}

// ==================== Playback with gain ====================
void play() {
  if (record_samples == 0) return;
  Serial.println("🔊 Playing...");
  for (size_t i = 0; i < record_samples; i++) {
    int16_t raw = record_buffer[i];
    int32_t amplified = (int32_t)raw * 8; // 8x gain
    if (amplified > 32767) amplified = 32767;
    if (amplified < -32768) amplified = -32768;
    int16_t sample = (int16_t)amplified;

    int32_t word = ((int32_t)sample) << 16; // left-align in 32-bit slot
    i2sSpk.write((uint8_t*)&word, sizeof(word));   // left
    i2sSpk.write((uint8_t*)&word, sizeof(word));   // right
  }
  delay(50);
  Serial.println("Playback finished.");
}

// ==================== Auto-Namer ====================
String getNextFileName(String dir, String base) {
    int maxIndex = 0;
    File root = SD_MMC.open(dir);
    if (!root) return base + "1";
    File file = root.openNextFile();
    while (file) {
        String fname = file.name();
        if (fname.startsWith(base) && fname.endsWith(".wav")) {
            String numStr = fname.substring(base.length());
            numStr.replace(".wav", "");
            if (numStr.length() > 0) {
                int idx = numStr.toInt();
                if (idx > maxIndex) maxIndex = idx;
            }
        }
        file = root.openNextFile();
    }
    root.close();
    return base + String(maxIndex + 1);
}

// ==================== Save WAV to SD ====================
void saveWav(String fullPath) {
  if (record_samples == 0) return;

  File f = SD_MMC.open(fullPath, FILE_WRITE);
  if (!f) {
    Serial.println("Could not open file for writing!");
    return;
  }

  uint32_t dataSize = record_samples * 2;
  uint32_t fileSize = dataSize + 36;
  uint16_t audioFormat = 1;
  uint16_t numChannels = 1;
  uint32_t byteRate = n
  nnnn * numChannels * 2;
  uint16_t blockAlign = numChannels * 2;
  uint16_t bitsPerSample = 16;

  auto writeLE = [&](uint32_t val, int bytes) {
    for (int i = 0; i < bytes; i++) {
      f.write((uint8_t)(val & 0xFF));
      val >>= 8;
    }
  };

  f.write((const uint8_t*)"RIFF", 4); writeLE(fileSize, 4);
  f.write((const uint8_t*)"WAVE", 4);
  f.write((const uint8_t*)"fmt ", 4); writeLE(16, 4);
  writeLE(audioFormat, 2); writeLE(numChannels, 2);
  writeLE(n
  nnnn, 4); writeLE(byteRate, 4);
  writeLE(blockAlign, 2); writeLE(bitsPerSample, 2);
  f.write((const uint8_t*)"data", 4); writeLE(dataSize, 4);

  for (size_t i = 0; i < record_samples; i++) {
    int16_t s = record_buffer[i];
    f.write((uint8_t*)&s, 2);
  }
  f.close();
}

// ==================== RGB LED Control (using your built-in macro) ====================
void setLedColor(uint8_t r, uint8_t g, uint8_t b) {
  #ifdef RGB_BUILTIN
    rgbLedWrite(RGB_BUILTIN, r, g, b);
  #endif
}

// ==================== CLI command processor ====================
void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

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
    while (entry) {
      Serial.print(entry.name());
      if (entry.isDirectory()) Serial.println("/");
      else { Serial.print("\t"); Serial.println(entry.size()); }
      entry = dir.openNextFile();
    }
    dir.close();
  }
  else if (command == "cd") {
    String newPath;
    if (argument.length() == 0) newPath = "/";
    else if (argument.startsWith("/")) newPath = argument;
    else if (argument == "..") {
      int lastSlash = currentPath.lastIndexOf('/');
      if (lastSlash == 0 && currentPath.length() > 1) newPath = "/";
      else if (lastSlash > 0) newPath = currentPath.substring(0, lastSlash);
      else newPath = currentPath;
    } else {
      newPath = currentPath;
      if (!newPath.endsWith("/")) newPath += "/";
      newPath += argument;
    }
    File testDir = SD_MMC.open(newPath);
    if (!testDir || !testDir.isDirectory()) Serial.println("Directory does not exist.");
    else currentPath = newPath;
    testDir.close();
  }
  else if (command == "mkdir") {
    if (argument.length() == 0) { Serial.println("Usage: mkdir <dir>"); return; }
    String fullPath = currentPath;
    if (!fullPath.endsWith("/")) fullPath += "/";
    fullPath += argument;
    if (SD_MMC.mkdir(fullPath)) Serial.println("Directory created.");
    else Serial.println("Failed to create directory.");
  }
  else if (command == "pwd") { Serial.println(currentPath); }
  else { Serial.println("Unknown command. Available: ls, cd, mkdir, pwd"); }
}
// #include "ESP_I2S.h"
// #include "SD_MMC.h"
// #include "esp_heap_caps.h"
// #include <WiFi.h>

// // ==================== WiFi credentials ====================
// const char *ssid = "audiocollectorunits3";
// const char *password = "123456789";

// // ==================== Pin definitions ====================
// #define I2S_MIC_BCLK  41
// #define I2S_MIC_WS    42
// #define I2S_MIC_DATA  2
// #define I2S_SPK_BCLK  21
// #define I2S_SPK_LRC   14
// #define I2S_SPK_DIN   47
// #define BUTTON_PIN    12

// // ==================== Audio settings ====================
// #define SAMPLE_RATE   16000
// #define RECORD_SEC    2
// #define BUFFER_SIZE   (SAMPLE_RATE * RECORD_SEC)

// int16_t *record_buffer = nullptr;
// size_t record_samples = 0;
// bool recording_available = false; // Flag to know if we have audio to save

// I2SClass i2sMic;
// I2SClass i2sSpk;

// // ==================== File system & Web state ====================
// String currentPath = "/";
// bool promptShown = false;

// WiFiServer uiServer(8080);

// // ==================== Forward declarations ====================
// void processCommand(String cmd);
// void saveWav(String path);
// void record();
// void play();
// String getNextFileName(String dir, String base);
// void setLedColor(uint8_t r, uint8_t g, uint8_t b);

// // ==================== Web Server Handlers ====================
// void handleClient(WiFiClient &client);

// // ==================== Setup ====================
// void setup() {
//   Serial.begin(115200);
//   delay(1000);
//   Serial.println("\n📻 Portable Audio CLI starting...");

//   // ---- Init RGB LED ----
//   #ifdef RGB_BUILTIN
//     pinMode(RGB_BUILTIN, OUTPUT);
//     setLedColor(0, 0, 0);
//   #endif

//   // ---- Mount SD card ----
//   if (!SD_MMC.setPins(39, 38, 40)) {
//     Serial.println("SD_MMC pin set failed!");
//     while (1);
//   }
//   if (!SD_MMC.begin("/sdcard", true)) {
//     Serial.println("SD card mount failed!");
//     while (1);
//   }
//   Serial.println("SD card ready.");

//   // ---- Allocate PSRAM buffer (2 seconds exactly) ----
//   size_t buffer_bytes = BUFFER_SIZE * sizeof(int16_t);
//   record_buffer = (int16_t*)heap_caps_malloc(buffer_bytes, MALLOC_CAP_SPIRAM);
//   if (!record_buffer) {
//     record_buffer = (int16_t*)heap_caps_malloc(buffer_bytes, MALLOC_CAP_8BIT);
//     if (!record_buffer) {
//       Serial.println("FATAL: No memory!");
//       while (1);
//     }
//   }
//   Serial.printf("Buffer: %d KB from %s\n", buffer_bytes / 1024, 
//                 esp_ptr_external_ram(record_buffer) ? "PSRAM" : "DRAM");

//   // ---- Init microphone ----
//   i2sMic.setPins(I2S_MIC_BCLK, I2S_MIC_WS, -1, I2S_MIC_DATA);
//   if (!i2sMic.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT,
//                     I2S_SLOT_MODE_MONO, I2S_STD_SLOT_LEFT)) {
//     Serial.println("Mic init failed!");
//     while (1);
//   }

//   // ---- Init speaker ----
//   i2sSpk.setPins(I2S_SPK_BCLK, I2S_SPK_LRC, I2S_SPK_DIN, -1);
//   if (!i2sSpk.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_32BIT,
//                     I2S_SLOT_MODE_STEREO)) {
//     Serial.println("Speaker init failed!");
//     while (1);
//   }

//   // ---- Button setup ----
//   pinMode(BUTTON_PIN, INPUT_PULLUP);

//   // ---- Connect to WiFi ----
//   WiFi.begin(ssid, password);
//   WiFi.setSleep(false);
//   Serial.print("WiFi connecting");
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     Serial.print(".");
//   }
//   Serial.println("\nWiFi connected!");
//   Serial.print("IP Address: ");
//   Serial.println(WiFi.localIP());

//   // ---- Start Web Server on Port 8080 ----
//   uiServer.begin();
//   Serial.println("Web UI ready at http://" + WiFi.localIP().toString() + ":8080");
//   promptShown = false;
// }

// // ==================== Main Loop ====================
// void loop() {
//   // 1. Handle Serial CLI (exactly as before)
//   if (!promptShown) {
//     Serial.print(currentPath + "> ");
//     promptShown = true;
//   }
//   if (Serial.available()) {
//     String line = Serial.readStringUntil('\n');
//     line.trim();
//     if (line.length() > 0) {
//       processCommand(line);
//     }
//     promptShown = false;
//   }

//   // 2. Handle Web Client
//   WiFiClient client = uiServer.available();
//   if (client) {
//     handleClient(client);
//     client.stop();
//   }

//   // 3. Handle Button Press (Blocking Sequence)
//   static bool lastButtonState = HIGH;
//   bool buttonState = digitalRead(BUTTON_PIN);
//   if (buttonState == LOW && lastButtonState == HIGH) {
//     Serial.println("\n⏳ Preparing... (1s delay to clear button click)");
//     delay(1000); // Debounce the mechanical click

//     setLedColor(0, 255, 0); // Green LED on
//     Serial.println("🎤 Recording... (2 seconds)");

//     record(); // BLOCKING RECORD

//     setLedColor(0, 0, 0); // LED off
//     recording_available = true; // Mark the buffer as valid
//     Serial.printf("✅ Recorded %d samples (2.0s)\n", record_samples);

//     Serial.println("🔊 Playing back...");
//     play(); // BLOCKING PLAYBACK

//     Serial.println("✅ Ready! Visit the Web UI to save this file, or press the button to re-record.");
//   }
//   lastButtonState = buttonState;

//   delay(5);
// }

// // ==================== Record (fixed 2s, blocking) ====================
// void record() {
//   record_samples = 0;
//   while (record_samples < BUFFER_SIZE) {
//     int16_t sample;
//     i2sMic.readBytes((char*)&sample, sizeof(sample));
//     record_buffer[record_samples++] = sample;
//   }
// }

// // ==================== Playback ====================
// void play() {
//   if (record_samples == 0) return;
//   for (size_t i = 0; i < record_samples; i++) {
//     int16_t raw = record_buffer[i];
//     int32_t amplified = (int32_t)raw * 8; // 8x gain
//     if (amplified > 32767) amplified = 32767;
//     if (amplified < -32768) amplified = -32768;
//     int16_t sample = (int16_t)amplified;
//     int32_t word = ((int32_t)sample) << 16; // left-align
//     i2sSpk.write((uint8_t*)&word, sizeof(word));
//     i2sSpk.write((uint8_t*)&word, sizeof(word));
//   }
// }

// // ==================== Web Client Handler ====================
// void handleClient(WiFiClient &client) {
//   String header = "";
//   while (client.connected() && client.available()) {
//     char c = client.read();
//     header += c;
//     if (header.endsWith("\r\n\r\n")) break;
//   }
//   if (header.length() == 0) return;

//   // --- SAVE AUDIO ---
//   if (header.indexOf("GET /save_audio") >= 0) {
//     String response;
//     if (!recording_available) {
//       response = "❌ No recording available! Press the button first.";
//     } else {
//       String folderName = currentPath;
//       if (folderName.endsWith("/")) folderName = folderName.substring(0, folderName.length() - 1);
//       int lastSlash = folderName.lastIndexOf('/');
//       String baseName = (lastSlash >= 0) ? folderName.substring(lastSlash + 1) : "sample";
      
//       String autoFname = getNextFileName(currentPath, baseName);
//       String fullPath = currentPath;
//       if (!fullPath.endsWith("/")) fullPath += "/";
//       fullPath += autoFname + ".wav";
      
//       saveWav(fullPath);
//       recording_available = false; // Reset so it can't be saved twice
//       response = "✅ Saved as: " + autoFname + ".wav";
//     }
//     client.println("HTTP/1.1 200 OK");
//     client.println("Content-type:text/plain");
//     client.println("Connection: close");
//     client.println();
//     client.print(response);
//     return;
//   }

//   // --- NAVIGATE DIRECTORIES (CHANGE DIR) ---
//   if (header.indexOf("GET /cd") >= 0) {
//     String newPath = "/";
//     int pathStart = header.indexOf("path=");
//     if (pathStart > 0) {
//       int pathEnd = header.indexOf(" ", pathStart);
//       if (pathEnd < 0) pathEnd = header.indexOf(" HTTP", pathStart);
//       newPath = header.substring(pathStart + 5, pathEnd);
//       newPath.trim();
      
//       // Sanitize path
//       if (newPath.length() == 0) newPath = "/";
//       else if (newPath.startsWith("/")) { /* keep as is */ }
//       else {
//         if (!currentPath.endsWith("/")) newPath = currentPath + "/" + newPath;
//         else newPath = currentPath + newPath;
//       }
      
//       File testDir = SD_MMC.open(newPath);
//       if (!testDir || !testDir.isDirectory()) {
//         client.println("HTTP/1.1 303 See Other");
//         client.println("Location: /?status=Error%20Directory%20not%20found");
//         client.println("Connection: close");
//         client.println();
//         return;
//       }
//       currentPath = newPath;
//       testDir.close();
//     }
//     client.println("HTTP/1.1 303 See Other");
//     client.println("Location: /");
//     client.println("Connection: close");
//     client.println();
//     return;
//   }

//   // --- MAKE DIRECTORY ---
//   if (header.indexOf("GET /mkdir") >= 0) {
//     String dirName = "";
//     int dirStart = header.indexOf("dir=");
//     if (dirStart > 0) {
//       int dirEnd = header.indexOf(" ", dirStart);
//       if (dirEnd < 0) dirEnd = header.indexOf(" HTTP", dirStart);
//       dirName = header.substring(dirStart + 4, dirEnd);
//       dirName.trim();
//     }
//     if (dirName.length() > 0) {
//       String fullPath = currentPath;
//       if (!fullPath.endsWith("/")) fullPath += "/";
//       fullPath += dirName;
//       if (SD_MMC.mkdir(fullPath)) {
//         client.println("HTTP/1.1 303 See Other");
//         client.println("Location: /?status=Folder%20Created");
//         client.println("Connection: close");
//         client.println();
//         return;
//       }
//     }
//     client.println("HTTP/1.1 303 See Other");
//     client.println("Location: /?status=Failed%20to%20create%20folder");
//     client.println("Connection: close");
//     client.println();
//     return;
//   }

//   // --- SERVE HTML PAGE ---
//   if (header.indexOf("GET / ") >= 0) {
//     String ip = WiFi.localIP().toString();
//     String folder = currentPath;
//     if (folder == "/") folder = "/ (Root)";

//     String html = R"rawliteral(
// <!DOCTYPE html>
// <html>
// <head>
//   <meta charset="UTF-8">
//   <meta name="viewport" content="width=device-width, initial-scale=1.0">
//   <title>Audio Collector</title>
//   <style>
//     body { font-family: Arial; background: #0d1117; color: #c9d1d9; text-align: center; padding: 20px; }
//     .container { max-width: 600px; margin: 0 auto; background: #161b22; padding: 20px; border-radius: 12px; border: 1px solid #30363d; }
//     .folder-box { background: #0d1117; border: 1px solid #30363d; border-radius: 8px; padding: 10px; margin: 15px 0; text-align: left; }
//     input { width: 70%; padding: 8px; border-radius: 6px; border: 1px solid #30363d; background: #0d1117; color: #c9d1d9; }
//     .btn { padding: 8px 16px; font-weight: bold; border: none; border-radius: 6px; cursor: pointer; margin: 5px; }
//     .btn-primary { background: #1f6feb; color: white; }
//     .btn-success { background: #238636; color: white; }
//     .btn-danger { background: #da3633; color: white; }
//     #status { margin-top: 12px; font-size: 14px; color: #8b949e; }
//     .waveform { height: 80px; background: #0d1117; border-radius: 6px; margin: 15px 0; border: 1px dashed #30363d; display: flex; align-items: center; justify-content: center; color: #484f58; }
//   </style>
// </head>
// <body>
// <div class="container">
//   <h2>🎙️ Portable Audio Collector</h2>
//   <div class="folder-box">
//     Current Folder: <b style="color:#58a6ff">)rawliteral";
//     html += folder;
//     html += R"rawliteral(</b>
//   </div>
  
//   <div class="waveform">📻 Press button on device to record & playback</div>

//   <button class="btn btn-success" onclick="saveAudio()">💾 Save Last Recording</button>
//   <div id="status">Click 'Save' to store the last recorded 2-second audio.</div>

//   <hr style="border-color:#30363d; margin: 20px 0;">
//   <h4>Directory Controls</h4>
//   <div>
//     <input type="text" id="dirInput" placeholder="folder_name">
//     <button class="btn btn-primary" onclick="mkdir()">📂 Make Folder</button>
//   </div>
//   <div style="margin-top: 10px;">
//     <input type="text" id="cdInput" placeholder="../ or subfolder">
//     <button class="btn btn-primary" onclick="cd()">📁 Change Dir</button>
//   </div>
// </div>
// <script>
//   const urlParams = new URLSearchParams(window.location.search);
//   const status = urlParams.get('status');
//   if (status) { document.getElementById('status').innerHTML = decodeURIComponent(status); }

//   function saveAudio() {
//     document.getElementById('status').innerHTML = 'Saving...';
//     fetch('/save_audio').then(r => r.text()).then(t => {
//       document.getElementById('status').innerHTML = t;
//     }).catch(() => {
//       document.getElementById('status').innerHTML = '❌ Error saving audio!';
//     });
//   }
//   function mkdir() {
//     const d = document.getElementById('dirInput').value.trim();
//     if (d) window.location.href = '/mkdir?dir=' + encodeURIComponent(d);
//   }
//   function cd() {
//     const p = document.getElementById('cdInput').value.trim();
//     window.location.href = '/cd?path=' + encodeURIComponent(p || '/');
//   }
// </script>
// </body>
// </html>
// )rawliteral";

//     client.println("HTTP/1.1 200 OK");
//     client.println("Content-type:text/html");
//     client.println("Connection: close");
//     client.println();
//     client.print(html);
//     return;
//   }
// }

// // ==================== Save WAV to SD ====================
// void saveWav(String fullPath) {
//   if (record_samples == 0) return;

//   File f = SD_MMC.open(fullPath, FILE_WRITE);
//   if (!f) {
//     Serial.println("Could not open file for writing!");
//     return;
//   }

//   uint32_t dataSize = record_samples * 2;
//   uint32_t fileSize = dataSize + 36;
//   uint16_t audioFormat = 1;
//   uint16_t numChannels = 1;
//   uint32_t byteRate = SAMPLE_RATE * numChannels * 2;
//   uint16_t blockAlign = numChannels * 2;
//   uint16_t bitsPerSample = 16;

//   auto writeLE = [&](uint32_t val, int bytes) {
//     for (int i = 0; i < bytes; i++) {
//       f.write((uint8_t)(val & 0xFF));
//       val >>= 8;
//     }
//   };

//   f.write((const uint8_t*)"RIFF", 4); writeLE(fileSize, 4);
//   f.write((const uint8_t*)"WAVE", 4);
//   f.write((const uint8_t*)"fmt ", 4); writeLE(16, 4);
//   writeLE(audioFormat, 2); writeLE(numChannels, 2);
//   writeLE(SAMPLE_RATE, 4); writeLE(byteRate, 4);
//   writeLE(blockAlign, 2); writeLE(bitsPerSample, 2);
//   f.write((const uint8_t*)"data", 4); writeLE(dataSize, 4);

//   for (size_t i = 0; i < record_samples; i++) {
//     int16_t s = record_buffer[i];
//     f.write((uint8_t*)&s, 2);
//   }
//   f.close();
// }

// // ==================== CLI command processor ====================
// void processCommand(String cmd) {
//   cmd.trim();
//   if (cmd.length() == 0) return;

//   String command, argument;
//   int spaceIdx = cmd.indexOf(' ');
//   if (spaceIdx > 0) {
//     command = cmd.substring(0, spaceIdx);
//     argument = cmd.substring(spaceIdx + 1);
//     argument.trim();
//   } else {
//     command = cmd;
//   }

//   if (command == "ls") {
//     File dir = SD_MMC.open(currentPath);
//     if (!dir || !dir.isDirectory()) { Serial.println("Cannot open directory."); return; }
//     File entry = dir.openNextFile();
//     if (!entry) Serial.println("(empty)");
//     while (entry) {
//       Serial.print(entry.name());
//       if (entry.isDirectory()) Serial.println("/");
//       else { Serial.print("\t"); Serial.println(entry.size()); }
//       entry = dir.openNextFile();
//     }
//     dir.close();
//   }
//   else if (command == "cd") {
//     String newPath;
//     if (argument.length() == 0) newPath = "/";
//     else if (argument.startsWith("/")) newPath = argument;
//     else if (argument == "..") {
//       int lastSlash = currentPath.lastIndexOf('/');
//       if (lastSlash == 0 && currentPath.length() > 1) newPath = "/";
//       else if (lastSlash > 0) newPath = currentPath.substring(0, lastSlash);
//       else newPath = currentPath;
//     } else {
//       newPath = currentPath;
//       if (!newPath.endsWith("/")) newPath += "/";
//       newPath += argument;
//     }
//     File testDir = SD_MMC.open(newPath);
//     if (!testDir || !testDir.isDirectory()) Serial.println("Directory does not exist.");
//     else currentPath = newPath;
//     testDir.close();
//   }
//   else if (command == "mkdir") {
//     if (argument.length() == 0) { Serial.println("Usage: mkdir <dir>"); return; }
//     String fullPath = currentPath;
//     if (!fullPath.endsWith("/")) fullPath += "/";
//     fullPath += argument;
//     if (SD_MMC.mkdir(fullPath)) Serial.println("Directory created.");
//     else Serial.println("Failed to create directory.");
//   }
//   else if (command == "pwd") { Serial.println(currentPath); }
//   else { Serial.println("Unknown command. Available: ls, cd, mkdir, pwd"); }
// }

// // ==================== Auto-Namer ====================
// String getNextFileName(String dir, String base) {
//     int maxIndex = 0;
//     File root = SD_MMC.open(dir);
//     if (!root) return base + "1";
//     File file = root.openNextFile();
//     while (file) {
//         String fname = file.name();
//         if (fname.startsWith(base) && fname.endsWith(".wav")) {
//             String numStr = fname.substring(base.length());
//             numStr.replace(".wav", "");
//             if (numStr.length() > 0) {
//                 int idx = numStr.toInt();
//                 if (idx > maxIndex) maxIndex = idx;
//             }
//         }
//         file = root.openNextFile();
//     }
//     root.close();
//     return base + String(maxIndex + 1);
// }

// void setLedColor(uint8_t r, uint8_t g, uint8_t b) {
//   #ifdef RGB_BUILTIN
//     rgbLedWrite(RGB_BUILTIN, r, g, b);
//   #endif
// }

#include "ESP_I2S.h"
#include "SD_MMC.h"
#include "esp_heap_caps.h"
#include <WiFi.h>
#include <vector> // Required for resolvePath()

// ==================== WiFi credentials ====================
const char *ssid = "audiocollectorunits3";
const char *password = "123456789";

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
#define RECORD_SEC    2.5
#define BUFFER_SIZE   (SAMPLE_RATE * RECORD_SEC)

int16_t *record_buffer = nullptr;
size_t record_samples = 0;
bool recording_available = false;

I2SClass i2sMic;
I2SClass i2sSpk;

String currentPath = "/";
bool promptShown = false;
WiFiServer uiServer(8080);

void processCommand(String cmd);
void saveWav(String path);
void record();
void play();
String getNextFileName(String dir, String base);
void setLedColor(uint8_t r, uint8_t g, uint8_t b);
void handleClient(WiFiClient &client);

// ==================== Helpers: URL Decode & Path Resolution ====================
String urlDecode(String input) {
  String decoded = "";
  char a, b;
  for (size_t i = 0; i < input.length(); i++) {
    if (input[i] == '%') {
      if (i + 2 < input.length()) {
        a = input[i + 1];
        b = input[i + 2];
        if (isxdigit(a) && isxdigit(b)) {
          char hex[3] = {a, b, '\0'};
          decoded += (char)strtol(hex, NULL, 16);
          i += 2;
        } else {
          decoded += '%';
        }
      } else {
        decoded += '%';
      }
    } else {
      decoded += input[i];
    }
  }
  return decoded;
}

String resolvePath(String base, String target) {
  if (target.length() == 0 || target == "/") return "/";
  if (target.startsWith("/")) {
    while (target.endsWith("/")) target.remove(target.length() - 1);
    return target.length() == 0 ? "/" : target;
  }

  std::vector<String> components;
  String temp = base;
  if (temp == "/") temp = "";
  if (temp.length() > 0 && temp.endsWith("/")) temp.remove(temp.length() - 1);
  int start = 0;
  int slash = temp.indexOf('/');
  while (slash != -1) {
    components.push_back(temp.substring(start, slash));
    start = slash + 1;
    slash = temp.indexOf('/', start);
  }
  if (start < temp.length()) components.push_back(temp.substring(start));

  String remaining = target;
  while (remaining.length() > 0) {
    while (remaining.startsWith("/")) remaining.remove(0, 1);
    if (remaining.length() == 0) break;

    int nextSlash = remaining.indexOf('/');
    String comp;
    if (nextSlash == -1) {
      comp = remaining;
      remaining = "";
    } else {
      comp = remaining.substring(0, nextSlash);
      remaining = remaining.substring(nextSlash + 1);
    }

    if (comp == ".") {
      // stay
    } else if (comp == "..") {
      if (!components.empty()) components.pop_back();
    } else {
      components.push_back(comp);
    }
  }

  String resolved = "";
  for (const auto &part : components) {
    if (part.length() > 0) {
      resolved += "/" + part;
    }
  }
  if (resolved.length() == 0) resolved = "/";
  return resolved;
}

// ==================== Setup ====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n📻 Portable Audio CLI starting...");

  #ifdef RGB_BUILTIN
    pinMode(RGB_BUILTIN, OUTPUT);
    setLedColor(0, 0, 0);
  #endif

  if (!SD_MMC.setPins(39, 38, 40)) {
    Serial.println("SD_MMC pin set failed!");
    while (1);
  }
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("SD card mount failed!");
    while (1);
  }
  Serial.println("SD card ready.");

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

  i2sMic.setPins(I2S_MIC_BCLK, I2S_MIC_WS, -1, I2S_MIC_DATA);
  if (!i2sMic.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT,
                    I2S_SLOT_MODE_MONO, I2S_STD_SLOT_LEFT)) {
    Serial.println("Mic init failed!");
    while (1);
  }

  i2sSpk.setPins(I2S_SPK_BCLK, I2S_SPK_LRC, I2S_SPK_DIN, -1);
  if (!i2sSpk.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_32BIT,
                    I2S_SLOT_MODE_STEREO)) {
    Serial.println("Speaker init failed!");
    while (1);
  }

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  uiServer.begin();
  Serial.println("Web UI ready at http://" + WiFi.localIP().toString() + ":8080");
  promptShown = false;
}

// ==================== Main Loop ====================
void loop() {
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

  WiFiClient client = uiServer.available();
  if (client) {
    handleClient(client);
    client.stop();
  }

  static bool lastButtonState = HIGH;
  bool buttonState = digitalRead(BUTTON_PIN);
  if (buttonState == LOW && lastButtonState == HIGH) {
    Serial.println("\n⏳ Preparing... (1s delay to clear button click)");
    delay(1000);
    setLedColor(0, 255, 0);
    Serial.println("🎤 Recording... (2 seconds)");
    record();
    setLedColor(0, 0, 0);
    recording_available = true;
    Serial.printf("✅ Recorded %d samples (2.0s)\n", record_samples);
    Serial.println("🔊 Playing back...");
    play();
    Serial.println("✅ Ready! Visit the Web UI to save this file, or press the button to re-record.");
  }
  lastButtonState = buttonState;
  delay(5);
}

void record() {
  record_samples = 0;
  while (record_samples < BUFFER_SIZE) {
    int16_t sample;
    i2sMic.readBytes((char*)&sample, sizeof(sample));
    record_buffer[record_samples++] = sample;
  }
}

void play() {
  if (record_samples == 0) return;
  for (size_t i = 0; i < record_samples; i++) {
    int16_t raw = record_buffer[i];
    int32_t amplified = (int32_t)raw * 8;
    if (amplified > 32767) amplified = 32767;
    if (amplified < -32768) amplified = -32768;
    int16_t sample = (int16_t)amplified;
    int32_t word = ((int32_t)sample) << 16;
    i2sSpk.write((uint8_t*)&word, sizeof(word));
    i2sSpk.write((uint8_t*)&word, sizeof(word));
  }
}

// ==================== Web Client Handler ====================
void handleClient(WiFiClient &client) {
  String header = "";
  while (client.connected() && client.available()) {
    char c = client.read();
    header += c;
    if (header.endsWith("\r\n\r\n")) break;
  }
  if (header.length() == 0) return;

  // --- Save audio ---
  if (header.indexOf("GET /save_audio") >= 0) {
    String response;
    if (!recording_available) {
      response = "❌ No recording available! Press the button first.";
    } else {
      String folderName = currentPath;
      if (folderName.endsWith("/")) folderName = folderName.substring(0, folderName.length() - 1);
      int lastSlash = folderName.lastIndexOf('/');
      String baseName = (lastSlash >= 0) ? folderName.substring(lastSlash + 1) : "sample";
      
      String autoFname = getNextFileName(currentPath, baseName);
      String fullPath = currentPath;
      if (!fullPath.endsWith("/")) fullPath += "/";
      fullPath += autoFname + ".wav";
      
      saveWav(fullPath);
      recording_available = false;
      response = "✅ Saved as: " + autoFname + ".wav";
    }
    client.println("HTTP/1.1 200 OK");
    client.println("Content-type:text/plain");
    client.println("Connection: close");
    client.println();
    client.print(response);
    return;
  }

  // --- Change directory ---
  if (header.indexOf("GET /cd") >= 0) {
    Serial.println(">>> Web request: cd");
    String rawPath = "/";
    int pathStart = header.indexOf("path=");
    if (pathStart > 0) {
      int pathEnd = header.indexOf(" ", pathStart);
      if (pathEnd < 0) pathEnd = header.indexOf(" HTTP", pathStart);
      rawPath = header.substring(pathStart + 5, pathEnd);
      rawPath.trim();
      rawPath = urlDecode(rawPath);
      Serial.println("  Requested path (decoded): " + rawPath);
    }

    String resolved = resolvePath(currentPath, rawPath);
    Serial.println("  Resolved path: " + resolved);

    File testDir = SD_MMC.open(resolved);
    if (testDir && testDir.isDirectory()) {
      currentPath = resolved;
      Serial.println("  Success, currentPath updated to: " + currentPath);
    } else {
      Serial.println("  Directory does not exist, keeping currentPath: " + currentPath);
    }
    if (testDir) testDir.close();

    client.println("HTTP/1.1 302 Found");
    client.println("Location: /");
    client.println("Connection: close");
    client.println();
    return;
  }

  // --- Make directory ---
  if (header.indexOf("GET /mkdir") >= 0) {
    Serial.println(">>> Web request: mkdir");
    String dirName = "";
    int dirStart = header.indexOf("dir=");
    if (dirStart > 0) {
      int dirEnd = header.indexOf(" ", dirStart);
      if (dirEnd < 0) dirEnd = header.indexOf(" HTTP", dirStart);
      dirName = header.substring(dirStart + 4, dirEnd);
      dirName.trim();
      dirName = urlDecode(dirName);
      Serial.println("  Directory name: " + dirName);
    }
    if (dirName.length() > 0) {
      String fullPath = currentPath;
      if (!fullPath.endsWith("/")) fullPath += "/";
      fullPath += dirName;
      Serial.println("  Creating: " + fullPath);
      if (SD_MMC.mkdir(fullPath)) {
        Serial.println("  Folder created successfully.");
      } else {
        Serial.println("  Failed to create folder.");
      }
    }
    client.println("HTTP/1.1 302 Found");
    client.println("Location: /");
    client.println("Connection: close");
    client.println();
    return;
  }

  // --- List directory ---
  if (header.indexOf("GET /list") >= 0) {
    String listHtml = "<table style='width:100%; text-align:left; font-size:14px;'>";
    File dir = SD_MMC.open(currentPath);
    if (dir && dir.isDirectory()) {
      File entry = dir.openNextFile();
      bool hasContent = false;
      while (entry) {
        hasContent = true;
        if (entry.isDirectory()) {
          listHtml += "<tr><td style='padding:4px 0;'>📁 <b>" + String(entry.name()) + "/</b></td><td style='text-align:right; color:#8b949e;'></td></tr>";
        } else {
          listHtml += "<tr><td style='padding:4px 0;'>📄 " + String(entry.name()) + "</td><td style='text-align:right; color:#8b949e;'>" + String(entry.size()) + " B</td></tr>";
        }
        entry = dir.openNextFile();
      }
      if (!hasContent) listHtml += "<tr><td style='color:#8b949e;'>Directory is empty.</td></tr>";
      dir.close();
    } else {
      listHtml += "<tr><td style='color:#da3633;'>Failed to open directory.</td></tr>";
    }
    listHtml += "</table>";
    client.println("HTTP/1.1 200 OK");
    client.println("Content-type:text/html");
    client.println("Connection: close");
    client.println();
    client.print(listHtml);
    return;
  }

  // --- Serve main page ---
  if (header.indexOf("GET / ") >= 0) {
    String ip = WiFi.localIP().toString();
    String folder = currentPath;
    if (folder == "/") folder = "/ (Root)";

    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Audio Collector</title>
  <style>
    :root {
      --bg: #0d1117; --bg-secondary: #161b22; --text: #c9d1d9; --border: #30363d; --input-bg: #0d1117; --link: #58a6ff;
    }
    .light-mode {
      --bg: #f6f8fa; --bg-secondary: #ffffff; --text: #24292f; --border: #d0d7de; --input-bg: #ffffff; --link: #0969da;
    }
    body { font-family: Arial; background: var(--bg); color: var(--text); text-align: center; padding: 20px; transition: background 0.3s, color 0.3s; }
    .container { max-width: 650px; margin: 0 auto; background: var(--bg-secondary); padding: 20px; border-radius: 12px; border: 1px solid var(--border); box-shadow: 0 4px 12px rgba(0,0,0,0.2); }
    .folder-box { background: var(--bg); border: 1px solid var(--border); border-radius: 8px; padding: 10px; margin: 15px 0; text-align: left; }
    input { width: 65%; padding: 8px; border-radius: 6px; border: 1px solid var(--border); background: var(--input-bg); color: var(--text); outline: none; }
    .btn { padding: 8px 16px; font-weight: bold; border: none; border-radius: 6px; cursor: pointer; margin: 5px; transition: 0.2s; }
    .btn-primary { background: #1f6feb; color: white; }
    .btn-primary:hover { background: #388bfd; }
    .btn-success { background: #238636; color: white; }
    .btn-success:hover { background: #2ea043; }
    .btn-danger { background: #da3633; color: white; }
    .btn-theme { position: absolute; top: 20px; right: 20px; background: var(--bg-secondary); color: var(--text); border: 1px solid var(--border); }
    #status { margin-top: 12px; font-size: 14px; color: #8b949e; }
    .waveform { height: 80px; background: var(--bg); border-radius: 6px; margin: 15px 0; border: 1px dashed var(--border); display: flex; align-items: center; justify-content: center; color: #484f58; }
    a { color: var(--link); text-decoration: none; }
    table { border-collapse: collapse; width: 100%; background: var(--bg); border-radius: 8px; overflow: hidden; }
    td { border-bottom: 1px solid var(--border); padding: 6px 10px; }
    tr:last-child td { border-bottom: none; }
  </style>
</head>
<body>
<button class="btn btn-theme" onclick="toggleTheme()">🌓 Toggle Theme</button>
<div class="container">
  <h2>🎙️ Portable Audio Collector</h2>
  <div class="folder-box">
    Current Folder: <b style="color:var(--link)">)rawliteral";
    html += folder;
    html += R"rawliteral(</b>
  </div>
  
  <div class="waveform">📻 Press button on device to record & playback</div>

  <button class="btn btn-success" onclick="saveAudio()">💾 Save Last Recording</button>
  <div id="status">Click 'Save' to store the last recorded 2-second audio.</div>

  <hr style="border-color:var(--border); margin: 20px 0;">
  
  <h4>📂 Directory Browser</h4>
  <div id="file-list-container" style="background: var(--bg); border-radius: 8px; border: 1px solid var(--border); padding: 10px; max-height: 250px; overflow-y: auto; margin-bottom: 15px;">
    <div id="file-list">Loading files...</div>
  </div>

  <hr style="border-color:var(--border); margin: 20px 0;">
  <h4>Directory Controls</h4>
  <div>
    <input type="text" id="dirInput" placeholder="folder_name">
    <button class="btn btn-primary" onclick="mkdir()">📂 Make Folder</button>
  </div>
  <div style="margin-top: 10px;">
    <input type="text" id="cdInput" placeholder="../ or subfolder">
    <button class="btn btn-primary" onclick="cd()">📁 Change Dir</button>
  </div>
</div>
<script>
  if (localStorage.getItem('theme') === 'light') { document.body.classList.add('light-mode'); }
  function toggleTheme() {
    document.body.classList.toggle('light-mode');
    localStorage.setItem('theme', document.body.classList.contains('light-mode') ? 'light' : 'dark');
  }
  const urlParams = new URLSearchParams(window.location.search);
  const status = urlParams.get('status');
  if (status) { document.getElementById('status').innerHTML = decodeURIComponent(status); }
  function loadFileList() {
    fetch('/list').then(r => r.text()).then(html => {
      document.getElementById('file-list').innerHTML = html;
    }).catch(() => {
      document.getElementById('file-list').innerHTML = '<span style="color:#da3633;">Error loading directory.</span>';
    });
  }
  function saveAudio() {
    document.getElementById('status').innerHTML = 'Saving...';
    fetch('/save_audio').then(r => r.text()).then(t => {
      document.getElementById('status').innerHTML = t;
      loadFileList();
    }).catch(() => {
      document.getElementById('status').innerHTML = '❌ Error saving audio!';
    });
  }
  function mkdir() {
    let d = document.getElementById('dirInput').value.trim();
    if (d.startsWith('mkdir ')) d = d.substring(6);
    if (d) window.location.href = '/mkdir?dir=' + encodeURIComponent(d);
  }
  function cd() {
    let p = document.getElementById('cdInput').value.trim();
    if (p.startsWith('cd ')) p = p.substring(3);
    window.location.href = '/cd?path=' + encodeURIComponent(p || '/');
  }
  loadFileList();
  setInterval(loadFileList, 3000);
</script>
</body>
</html>
)rawliteral";
    client.println("HTTP/1.1 200 OK");
    client.println("Content-type:text/html");
    client.println("Connection: close");
    client.println();
    client.print(html);
    return;
  }
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
  uint32_t byteRate = SAMPLE_RATE * numChannels * 2;
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
  writeLE(SAMPLE_RATE, 4); writeLE(byteRate, 4);
  writeLE(blockAlign, 2); writeLE(bitsPerSample, 2);
  f.write((const uint8_t*)"data", 4); writeLE(dataSize, 4);
  for (size_t i = 0; i < record_samples; i++) {
    int16_t s = record_buffer[i];
    f.write((uint8_t*)&s, 2);
  }
  f.close();
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
    if (!dir || !dir.isDirectory()) { Serial.println("Cannot open directory."); return; }
    File entry = dir.openNextFile();
    if (!entry) Serial.println("(empty)");
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

void setLedColor(uint8_t r, uint8_t g, uint8_t b) {
  #ifdef RGB_BUILTIN
    rgbLedWrite(RGB_BUILTIN, r, g, b);
  #endif
}
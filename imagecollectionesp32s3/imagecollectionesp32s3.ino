#define CONFIG_CAMERA_TASK_STACK_SIZE 32768   // Must be before includes
#include "esp_camera.h"
#include <WiFi.h>
#include "SD_MMC.h"
#include "board_config.h"

// ===========================
// WiFi credentials
// ===========================
const char *ssid = "FYI_2.4G";
const char *password = "freenove208";

// ==================== Forward declarations ====================
void startCameraServer();   // from camera server library
void setupLedFlash();       // from camera server library
void processCommand(String cmd);
String getNextFileName(String dir, String base);
String getFolderBaseName(String path);
String getCurrentFolder();
void captureAndSave();

// ==================== File system CLI state ====================
String currentPath = "/";
bool promptShown = false;

// ==================== Web Server on Port 8080 ====================
WiFiServer uiServer(8080);

// =================================================================
// SETUP
// =================================================================
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  // ---- 1. Mount SD card ----
  if (!SD_MMC.setPins(39, 38, 40)) {
    Serial.println("SD_MMC pin set failed!");
    while (1);
  }
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("SD card mount failed!");
    while (1);
  }
  Serial.println("SD card ready.");

  // ---- 2. Camera Init (exactly as working sketch, but JPEG) ----
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 8000000;        // slightly lower than 10MHz
  config.frame_size = FRAMESIZE_QVGA;   // 320x240
  config.pixel_format = PIXFORMAT_JPEG; // small & fast
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 2;                  // double buffer

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    while (1);
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_vflip(s, 1);
  s->set_brightness(s, 1);
  s->set_saturation(s, -1);

#if defined(LED_GPIO_NUM)
  setupLedFlash();
#endif

  // ---- 3. WiFi ----
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected, IP: " + WiFi.localIP().toString());

  // ---- 4. Start the default camera server (port 80) ----
  startCameraServer();
  Serial.println("Camera stream ready on http://" + WiFi.localIP().toString());

  // ---- 5. Start our custom server on port 8080 ----
  uiServer.begin();
  Serial.println("Save interface on http://" + WiFi.localIP().toString() + ":8080");

  promptShown = false;
}

// =================================================================
// MAIN LOOP
// =================================================================
void loop() {
  // ---------- CLI ----------
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

  // ---------- Custom Web Server (port 8080) ----------
  WiFiClient client = uiServer.available();
  if (client) {
    handleClient(client);
    client.stop();
  }

  // ---------- BOOT button (GPIO 0) capture ----------
  static bool lastButtonState = HIGH;
  bool buttonState = digitalRead(0);
  if (buttonState == LOW && lastButtonState == HIGH) {
    delay(50);  // debounce
    if (digitalRead(0) == LOW) {
      Serial.println("📸 BOOT pressed – capturing...");
      captureAndSave();
    }
  }
  lastButtonState = buttonState;

  delay(10);   // let other tasks breathe
}

// =================================================================
// WEB CLIENT HANDLER (port 8080)
// =================================================================
void handleClient(WiFiClient &client) {
  String header = "";
  while (client.connected() && client.available()) {
    char c = client.read();
    header += c;
    if (header.endsWith("\r\n\r\n")) break;
  }
  if (header.length() == 0) return;

  // --- Serve HTML interface ---
  if (header.indexOf("GET / ") >= 0 || header.indexOf("GET / HTTP") >= 0) {
    String ip = WiFi.localIP().toString();
    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Image Collector</title>
  <style>
    body { font-family: Arial; background: #0d1117; color: #c9d1d9; text-align: center; padding: 20px; }
    .container { max-width: 600px; margin: 0 auto; background: #161b22; padding: 20px; border-radius: 12px; border: 1px solid #30363d; }
    img { width: 100%; max-width: 480px; border: 2px solid #30363d; border-radius: 8px; margin: 20px 0; background: #000; }
    button { padding: 12px 24px; font-size: 16px; font-weight: bold; border: none; border-radius: 8px; margin: 5px; cursor: pointer; }
    .btn-blue { background: #1f6feb; color: white; }
    .btn-blue:hover { background: #388bfd; }
    .btn-green { background: #238636; color: white; }
    .btn-green:hover { background: #2ea043; }
    #status { margin-top: 12px; font-size: 14px; color: #8b949e; }
  </style>
</head>
<body>
<div class="container">
  <h2>📸 Image Collector</h2>
  <p>Current CLI Folder: <b>)rawliteral";
    html += getCurrentFolder();
    html += R"rawliteral(</b></p>
  <img id="preview" src="http://)rawliteral";
    html += ip;
    html += R"rawliteral(/capture" alt="Camera Preview">
  <br>
  <button class="btn-blue" onclick="refreshImage()">🔄 Refresh</button>
  <button class="btn-green" onclick="saveImage()">💾 Save</button>
  <div id="status">Click 'Refresh' for a new frame, 'Save' to store on SD.</div>
</div>
<script>
  function refreshImage() {
    document.getElementById('preview').src = 'http://)rawliteral";
    html += ip;
    html += R"rawliteral(/capture?t=' + new Date().getTime();
    document.getElementById('status').innerHTML = 'Preview updated.';
  }
  function saveImage() {
    document.getElementById('status').innerHTML = 'Saving...';
    fetch('/save').then(r => r.text()).then(t => {
      document.getElementById('status').innerHTML = t;
    }).catch(() => {
      document.getElementById('status').innerHTML = 'Error saving image!';
    });
  }
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

  // --- Save endpoint (capture and write to SD) ---
  if (header.indexOf("GET /save") >= 0) {
    captureAndSave();   // function sends response via client? No, we need to send here.
    // Actually captureAndSave currently only prints to Serial. We'll modify to return a string.
    // For simplicity, we'll inline the save logic here.
    camera_fb_t *fb = esp_camera_fb_get();
    String response;
    if (!fb) {
      response = "❌ Capture failed!";
    } else {
      String base = getFolderBaseName(currentPath);
      if (base.length() == 0) base = "img";
      String fname = getNextFileName(currentPath, base) + ".jpg";
      String fullPath = currentPath;
      if (!fullPath.endsWith("/")) fullPath += "/";
      fullPath += fname;

      File file = SD_MMC.open(fullPath, FILE_WRITE);
      if (!file) {
        response = "❌ SD write error!";
      } else {
        file.write(fb->buf, fb->len);
        file.close();
        response = "✅ Saved as: " + fname;
        Serial.println("Saved: " + fullPath);
      }
      esp_camera_fb_return(fb);
    }
    client.println("HTTP/1.1 200 OK");
    client.println("Content-type:text/plain");
    client.println("Connection: close");
    client.println();
    client.print(response);
    return;
  }

  // Unknown route
  client.println("HTTP/1.1 404 Not Found");
  client.println("Connection: close");
  client.println();
}

// =================================================================
// HELPER: CAPTURE AND SAVE (used by BOOT button)
// =================================================================
void captureAndSave() {
  digitalWrite(RGB_BUILTIN, HIGH);  // Turn the RGB LED white
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Capture failed");
    return;
  }
  String base = getFolderBaseName(currentPath);
  if (base.length() == 0) base = "img";
  String fname = getNextFileName(currentPath, base) + ".jpg";
  String fullPath = currentPath;
  if (!fullPath.endsWith("/")) fullPath += "/";
  fullPath += fname;

  File file = SD_MMC.open(fullPath, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open SD file!");
  } else {
    file.write(fb->buf, fb->len);
    file.close();
    Serial.println("Saved: " + fullPath);
  }
  esp_camera_fb_return(fb);
  digitalWrite(RGB_BUILTIN, LOW);  // Turn the RGB LED white
}

// =================================================================
// CLI COMMAND PROCESSOR (unchanged from your version)
// =================================================================
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

// =================================================================
// FILE NAMING HELPERS (unchanged)
// =================================================================
String getNextFileName(String dir, String base) {
  int maxIndex = 0;
  File root = SD_MMC.open(dir);
  if (!root) return base + "1";
  File file = root.openNextFile();
  while (file) {
    String fname = file.name();
    if (fname.startsWith(base) && fname.endsWith(".jpg")) {
      String numStr = fname.substring(base.length());
      numStr.replace(".jpg", "");
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

String getFolderBaseName(String path) {
  if (path == "/") return "root";
  String p = path;
  if (p.endsWith("/")) p = p.substring(0, p.length() - 1);
  int lastSlash = p.lastIndexOf('/');
  return (lastSlash >= 0) ? p.substring(lastSlash + 1) : "img";
}

String getCurrentFolder() {
  if (currentPath == "/") return "/ (Root)";
  return currentPath;
}
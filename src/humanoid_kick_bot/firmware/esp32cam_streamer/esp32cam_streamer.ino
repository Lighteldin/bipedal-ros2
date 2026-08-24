/*
 * esp32cam_streamer.ino
 * -----------------------
 * Minimal MJPEG-over-HTTP streaming firmware for an AI-Thinker ESP32-CAM
 * module, purpose-built for ball_detector_node.py (PART 3 - perception).
 *
 * This is a SEPARATE, DEDICATED board from the plain ESP32 that drives the
 * PCA9685 (esp32_pca9685_bridge.ino). Two different physical ESP32s:
 *   - ESP32-CAM  -> runs THIS sketch -> streams video over WiFi
 *   - Plain ESP32 -> runs esp32_pca9685_bridge.ino -> drives servos over I2C
 * Their GPIO21/22 usages are unrelated: on the ESP32-CAM these pins are
 * hardwired to the camera sensor (Y5/PCLK) by the module's own PCB design;
 * they are NOT available for I2C on this board. Do not try to combine the
 * two roles onto one ESP32-CAM - there aren't enough free GPIOs left after
 * the camera interface to also drive I2C for 9 servos reliably.
 *
 * Stream URL (matches what ball_detector_node.py expects):
 *     http://<esp32-cam-ip>:81/stream
 *
 * Library required: "esp32" board package (Espressif) which bundles the
 * esp_camera driver - install via Arduino IDE's Boards Manager, no extra
 * camera library needed.
 *
 * Board setting: AI Thinker ESP32-CAM, Partition Scheme: "Huge APP".
 */

#include <WiFi.h>
#include "esp_camera.h"

// ---- EDIT THESE ----
const char *WIFI_SSID = "YOUR_WIFI_SSID";
const char *WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
// ---------------------

// AI-Thinker ESP32-CAM pin map (fixed by the module's PCB, do not change
// unless you're on a different ESP32-CAM board variant).
#define PWDN_GPIO_NUM   32
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM    0
#define SIOD_GPIO_NUM   26
#define SIOC_GPIO_NUM   27
#define Y9_GPIO_NUM     35
#define Y8_GPIO_NUM     34
#define Y7_GPIO_NUM     39
#define Y6_GPIO_NUM     36
#define Y5_GPIO_NUM     21
#define Y4_GPIO_NUM     19
#define Y3_GPIO_NUM     18
#define Y2_GPIO_NUM      5
#define VSYNC_GPIO_NUM  25
#define HREF_GPIO_NUM   23
#define PCLK_GPIO_NUM   22

WiFiServer streamServer(81);

bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
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
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // QVGA (320x240) keeps frame rate and WiFi bandwidth reasonable for a
  // ball-tracking loop - raise this only if you have a strong WiFi link.
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;   // lower number = higher quality, more bytes
  config.fb_count = 2;        // double-buffer for smoother streaming

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init FAILED");
    return false;
  }
  return true;
}

void handleStreamClient(WiFiClient &client) {
  String header =
      "HTTP/1.1 200 OK\r\n"
      "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
  client.print(header);

  while (client.connected()) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Frame capture failed");
      break;
    }

    client.print("--frame\r\n");
    client.print("Content-Type: image/jpeg\r\n");
    client.print("Content-Length: " + String(fb->len) + "\r\n\r\n");
    client.write(fb->buf, fb->len);
    client.print("\r\n");

    esp_camera_fb_return(fb);

    if (!client.connected()) break;
    delay(10);  // small yield so WiFi stack / other tasks get CPU time
  }
}

void setup() {
  Serial.begin(115200);

  if (!initCamera()) {
    Serial.println("Halting - camera init error. Check wiring/pin config.");
    while (true) delay(1000);
  }

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Camera ready! Stream at: http://");
  Serial.print(WiFi.localIP());
  Serial.println(":81/stream");

  streamServer.begin();
}

void loop() {
  WiFiClient client = streamServer.available();
  if (client) {
    String request = client.readStringUntil('\r');
    client.readStringUntil('\n');  // consume the rest of the request line
    if (request.indexOf("/stream") >= 0) {
      handleStreamClient(client);
    }
    client.stop();
  }
}

#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
uint8_t broadcastAddress[] = {0xEC, 0xE3, 0x34, 0x7A, 0x8D, 0x9C}; // drone0
// uint8_t broadcastAddress[] = {0xEC, 0xE3, 0x34, 0x7B, 0xFB, 0xE4}; // drone1
// uint8_t broadcastAddress[] = {0x8C, 0x4F, 0x00, 0x35, 0xA3, 0x44}; // drone2


esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

char buffer[1024];
void loop() {
  int availableBytes = Serial.available();
  if (availableBytes) {
    Serial.readBytes(buffer, availableBytes);
    buffer[availableBytes] = '\0';

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)buffer, strlen(buffer) + 1);
    if (result) {Serial.println(esp_err_to_name(result));}
  }
  else{yield();}
}

#include "wifi_ble_onboarding.h"

#if defined(ARDUINO)
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#define SERVICE_UUID           "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26a8"

static Preferences preferences;
static WiFiUDP udp;

class BLECredCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();
        if (rxValue.length() > 0) {
            String data = String(rxValue.c_str());
            int split = data.indexOf('\n');
            if (split > 0) {
                String ssid = data.substring(0, split);
                String pass = data.substring(split + 1);
                ssid.trim();
                pass.trim();
                
                preferences.begin("wifi", false);
                preferences.putString("ssid", ssid);
                preferences.putString("pass", pass);
                preferences.end();
                
                Serial.println("[BLE] Credentials saved. Restarting...");
                delay(1000);
                ESP.restart();
            }
        }
    }
};

static void start_ble_server() {
    Serial.println("[BLE] Starting Onboarding Server (Antigravity_Agent_BLE)...");
    BLEDevice::init("Antigravity_Agent_BLE");
    BLEServer *pServer = BLEDevice::createServer();
    BLEService *pService = pServer->createService(SERVICE_UUID);
    BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
    pCharacteristic->setCallbacks(new BLECredCallbacks());
    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    BLEDevice::startAdvertising();
}

void wifi_ble_onboarding_init(void) {
    preferences.begin("wifi", true);
    String ssid = preferences.getString("ssid", "");
    String pass = preferences.getString("pass", "");
    preferences.end();

    if (ssid.length() == 0) {
        start_ble_server();
        return;
    }

    Serial.print("[WiFi] Connecting to: ");
    Serial.println(ssid);
    WiFi.begin(ssid.c_str(), pass.c_str());
    
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
        delay(100);
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("[WiFi] Connected! IP: ");
        Serial.println(WiFi.localIP());
        
        // Start UDP listener for Wi-Fi Handover
        udp.begin(4242);
        
        // Setup a FreeRTOS task to listen for UDP handover packets
        xTaskCreate([](void* arg) {
            while (true) {
                int packetSize = udp.parsePacket();
                if (packetSize) {
                    char buf[256];
                    int len = udp.read(buf, 255);
                    if (len > 0) {
                        buf[len] = '\0';
                        String data = String(buf);
                        int split = data.indexOf('\n');
                        if (split > 0) {
                            String new_ssid = data.substring(0, split);
                            String new_pass = data.substring(split + 1);
                            new_ssid.trim();
                            new_pass.trim();
                            
                            preferences.begin("wifi", false);
                            preferences.putString("ssid", new_ssid);
                            preferences.putString("pass", new_pass);
                            preferences.end();
                            
                            Serial.println("[UDP] Handover received. Restarting...");
                            delay(500);
                            ESP.restart();
                        }
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(100)); // Check every 100ms
            }
        }, "UDP_Handover", 4096, NULL, 1, NULL);
        
    } else {
        Serial.println("[WiFi] Connection failed. Fallback to BLE.");
        start_ble_server();
    }
}

#else
void wifi_ble_onboarding_init(void) {}
#endif

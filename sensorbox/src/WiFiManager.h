#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

class WiFiManager {
private:
    String ssid;
    String password;
    String apSSID;
    bool isAP;
    AsyncWebServer* server;
    
    // Config file path
    const char* configPath = "/config.json";
    
    // Load WiFi credentials from SPIFFS
    bool loadConfig() {
        if (!SPIFFS.begin(true)) {
            Serial.println("Failed to mount SPIFFS");
            return false;
        }
        
        // Check if config file exists, if not create a default one
        if (!SPIFFS.exists("/config.json")) {
          File configFile = SPIFFS.open("/config.json", "w");
          if (configFile) {
            StaticJsonDocument<256> doc;
            doc["ssid"] = "";
            doc["password"] = "";
            serializeJson(doc, configFile);
            configFile.close();
            Serial.println("Default config file created");
          } else {
            Serial.println("Failed to create config file");
          }
        }
        
        if (!SPIFFS.exists(configPath)) {
            Serial.println("Config file not found");
            return false;
        }
        
        File configFile = SPIFFS.open(configPath, "r");
        if (!configFile) {
            Serial.println("Failed to open config file");
            return false;
        }
        

        StaticJsonDocument<256> doc;
        DeserializationError error = deserializeJson(doc, configFile);
        configFile.close();
        
        if (error) {
            Serial.println("Failed to parse config file");
            return false;
        }
        
        ssid = doc["ssid"].as<String>();
        password = doc["password"].as<String>();
        
        return true;

    }    
    // Save WiFi credentials to SPIFFS
    bool saveConfig() {
        StaticJsonDocument<256> doc;
        doc["ssid"] = ssid;
        doc["password"] = password;
        
        File configFile = SPIFFS.open(configPath, "w");
        if (!configFile) {
            Serial.println("Failed to open config file for writing");
            return false;
        }
        
        if (serializeJson(doc, configFile) == 0) {
            Serial.println("Failed to write config file");
            return false;
        }
        
        configFile.close();
        return true;
    }
    
public:
    WiFiManager() : isAP(false), server(nullptr) {
        // Generate a unique AP SSID using ESP32's MAC address
        uint8_t mac[6];
        WiFi.macAddress(mac);
        char macStr[13];
        sprintf(macStr, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        apSSID = "SensorBox_" + String(macStr).substring(6);
    }
    
    ~WiFiManager() {
        if (server) {
            delete server;
        }
    }
    
    bool begin() {
        // Try to load saved config
        if (loadConfig() && ssid.length() > 0) {
            // Try to connect to WiFi
            Serial.print("Connecting to WiFi: ");
            Serial.println(ssid);
            
            WiFi.mode(WIFI_STA);
            WiFi.begin(ssid.c_str(), password.c_str());
            
            // Wait for connection for 10 seconds
            unsigned long startTime = millis();
            while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000) {
                delay(500);
                Serial.print(".");
            }
            
            if (WiFi.status() == WL_CONNECTED) {
                Serial.println("");
                Serial.print("Connected to WiFi. IP: ");
                Serial.println(WiFi.localIP());
                isAP = false;
                return true;
            }
            
            Serial.println("Failed to connect to WiFi");
        }
        
        // If we reach here, either no config or connection failed
        // Start AP mode
        Serial.print("Starting AP mode: ");
        Serial.println(apSSID);
        
        WiFi.mode(WIFI_AP);
        WiFi.softAP(apSSID.c_str());
        
        Serial.print("AP IP address: ");
        Serial.println(WiFi.softAPIP());
        
        isAP = true;
        return true;
    }
    
    void setupServer(AsyncWebServer* webServer) {
        server = webServer;
        
        // Serve the WiFi configuration page
        server->on("/wifi", HTTP_GET, [this](AsyncWebServerRequest *request) {
            String html = "<html><head><title>WiFi Configuration</title>";
            html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
            html += "<style>";
            html += "body { font-family: Arial, sans-serif; margin: 20px; }";
            html += "input, button { padding: 8px; margin: 5px 0; width: 100%; }";
            html += "button { background-color: #4CAF50; color: white; border: none; cursor: pointer; }";
            html += "</style></head><body>";
            html += "<h1>WiFi Configuration</h1>";
            html += "<form method='post' action='/save-wifi'>";
            html += "SSID: <input type='text' name='ssid' value='" + ssid + "'><br>";
            html += "Password: <input type='password' name='password' value='" + password + "'><br>";
            html += "<button type='submit'>Save</button>";
            html += "</form></body></html>";
            
            request->send(200, "text/html", html);
        });
        
        // Handle form submission
        server->on("/save-wifi", HTTP_POST, [this](AsyncWebServerRequest *request) {
            if (request->hasParam("ssid", true) && request->hasParam("password", true)) {
                ssid = request->getParam("ssid", true)->value();
                password = request->getParam("password", true)->value();
                
                if (saveConfig()) {
                    request->send(200, "text/html", "<html><head><meta http-equiv='refresh' content='5;url=/'></head><body>"
                                 "WiFi settings saved. The device will restart in 5 seconds.</body></html>");
                    // Schedule a restart
                    delay(100);
                    ESP.restart();
                } else {
                    request->send(500, "text/plain", "Failed to save configuration");
                }
            } else {
                request->send(400, "text/plain", "Missing parameters");
            }
        });
    }
    
    bool isAPMode() const {
        return isAP;
    }
    
    String getIP() const {
        if (isAP) {
            return WiFi.softAPIP().toString();
        } else {
            return WiFi.localIP().toString();
        }
    }
    
    String getSSID() const {
        if (isAP) {
            return apSSID;
        } else {
            return ssid;
        }
    }
};

#endif // WIFI_MANAGER_H

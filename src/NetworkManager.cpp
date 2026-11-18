#include "NetworkManager.hpp"

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include "secrets.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

NetworkManager networkManager;

void NetworkManager::begin()
{
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);  // Zachowujemy poprawke plynnosci TCP.
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    Serial.print("Connecting to WiFi");

    unsigned long start = millis();
    while (
        WiFi.status() != WL_CONNECTED &&
        millis() - start < 10000
    )
    {
        delay(100);
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println();
        Serial.print("ESP IP: ");
        Serial.println(WiFi.localIP());

        ArduinoOTA.setHostname("ploter-esp32");
        ArduinoOTA.setPassword(OTA_PASSWORD);
        ArduinoOTA.onStart([]() {
            // Zachowujemy dotychczasowe zachowanie OTA.
            Serial.println("OTA START");
        });
        ArduinoOTA.begin();

        Serial.println("OTA READY");
        Serial.println(WiFi.localIP());
    }
    else
    {
        Serial.println("WiFi connection failed");
    }
}

void NetworkManager::maintainWiFi()
{
    static uint32_t lastReconnectAttempt = 0;

    if (WiFi.status() == WL_CONNECTED)
        return;

    if (millis() - lastReconnectAttempt < 5000)
        return;

    lastReconnectAttempt = millis();

    Serial.println("WiFi reconnecting...");
    WiFi.reconnect();
}

void NetworkManager::otaTask(void* parameter)
{
    (void)parameter;

    while (true)
    {
        networkManager.maintainWiFi();

        if (WiFi.status() == WL_CONNECTED)
        {
            ArduinoOTA.handle();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


#pragma once

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// ----------------------------------------
// Struktury komunikacji
// ----------------------------------------

struct TcpCommand
{
    char text[112];
};

struct TcpResponse
{
    char text[128];
};

// ----------------------------------------
// Kolejki FreeRTOS
// ----------------------------------------

extern QueueHandle_t tcpCommands;
extern QueueHandle_t tabletCommands;
extern QueueHandle_t tcpResponses;
extern QueueHandle_t tcpPosition;

// ----------------------------------------
// CommunicationManager
// ----------------------------------------

enum class InputSource : uint8_t
{
    NONE,
    TCP,
    USB
};

class CommunicationManager
{
public:
    bool begin();
    void sendResponse(const char* message);
    void processCommand(InputSource source, const char* text);
    void readUsbCommands();
};

// Globalny obiekt komunikacji
extern CommunicationManager communication;

// Funkcja używana przez istniejący kod
void sendResponse(const char* message);

void tcpTask(void* parameter);

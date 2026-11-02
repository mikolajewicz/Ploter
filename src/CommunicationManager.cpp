#include "CommunicationManager.hpp"
#include <WiFi.h>
#include "GCodeStorage.hpp"
#include "esp_system.h"

extern GCodeStorage gcodeStorage;

extern esp_reset_reason_t bootResetReason;
extern uint32_t lastCrashStage;

constexpr uint16_t TCP_PORT = 5000;

WiFiServer tcpServer(TCP_PORT);


// ----------------------------------------
// Globalne kolejki FreeRTOS
// ----------------------------------------

QueueHandle_t tcpCommands = nullptr;
QueueHandle_t tabletCommands = nullptr;
QueueHandle_t tcpResponses = nullptr;
QueueHandle_t tcpPosition = nullptr;

// ----------------------------------------
// Globalny obiekt komunikacji
// ----------------------------------------

CommunicationManager communication;

// ----------------------------------------
// Inicjalizacja kolejek
// ----------------------------------------

bool CommunicationManager::begin()
{
    tcpCommands = xQueueCreate(
        16,
        sizeof(TcpCommand)
    );

    tabletCommands = xQueueCreate(
        1,
        sizeof(TcpCommand)
    );

    tcpResponses = xQueueCreate(
        32,
        sizeof(TcpResponse)
    );

    tcpPosition = xQueueCreate(
        1,
        sizeof(TcpResponse)
    );

    if (
        tcpCommands == nullptr ||
        tabletCommands == nullptr ||
        tcpResponses == nullptr ||
        tcpPosition == nullptr
    )
    {
        Serial.println(
            "Communication queue allocation failed"
        );

        return false;
    }

    return true;
}

// ----------------------------------------
// Wysyłanie odpowiedzi
// ----------------------------------------

void CommunicationManager::sendResponse(
    const char* message
)
{
    Serial.println(message);

    TcpResponse response = {};

    snprintf(
        response.text,
        sizeof(response.text),
        "%s\n",
        message
    );

    if (strncmp(message, "POS ", 4) == 0)
    {
        if (tcpPosition != nullptr)
        {
            xQueueOverwrite(
                tcpPosition,
                &response
            );
        }
    }
    else
    {
        if (
            tcpResponses != nullptr &&
            xQueueSend(
                tcpResponses,
                &response,
                0
            ) != pdTRUE
        )
        {
            Serial.println(
                "WARN TCP response queue full"
            );
        }
    }
}

// ----------------------------------------
// Funkcja zgodna ze starym interfejsem
// ----------------------------------------

void sendResponse(const char* message)
{
    communication.sendResponse(message);
}

void processCommand(const char* text)
{
    TcpCommand command = {};

   if (text == nullptr)
    {
        return;
    }

    size_t length = strlen(text);

    if (length >= sizeof(command.text))
    {
        sendResponse("ERR COMMAND TOO LONG");
        return;
    }

    memcpy(command.text, text, length + 1);

    // ==================================
    // GCODE_UPLOAD
    // ==================================

    if (
        strcasecmp(command.text, "PING") == 0
    )
    {
        sendResponse("PONG");
    }

    else if (
        strncmp(
            command.text,
            "GCODE_UPLOAD ",
            13
        ) == 0
    )
    {
        char* endPointer = nullptr;

        unsigned long size =
            strtoul(
                command.text + 13,
                &endPointer,
                10
            );

        if (
            endPointer == command.text + 13
            ||
            *endPointer != '\0'
            ||
            size == 0
        )
        {
            sendResponse(
                "ERR GCODE SIZE"
            );
        }
        else
        {
            bool started =
                gcodeStorage.beginUpload(
                    static_cast<size_t>(
                        size
                    )
                );

            if (!started)
            {
                sendResponse(
                    "ERR GCODE OPEN"
                );
            }
            else
            {
                char response[80];

                snprintf(
                    response,
                    sizeof(response),
                    "GCODE READY %u",
                    static_cast<unsigned>(
                        size
                    )
                );

                sendResponse(
                    response
                );
            }
        }
    }

    // ==================================
    // TABLET
    // ==================================

    else if (
        strncmp(
            command.text,
            "tablet ",
            7
        ) == 0
    )
    {
        // Zachowujemy tylko
        // najnowszy cel tabletu.
        if (
            tabletCommands
            != nullptr
        )
        {
            xQueueOverwrite(
                tabletCommands,
                &command
            );
        }
    }


    // ==================================
    // STOP
    // ==================================

    else if (
        strcmp(
            command.text,
            "stop"
        ) == 0
    )
    {
        // STOP ma pierwszeństwo.
        if (
            tcpCommands
            != nullptr
        )
        {
            if (
                xQueueSendToFront(
                    tcpCommands,
                    &command,
                    0
                ) != pdTRUE
            )
            {
                Serial.println(
                    "WARN TCP command queue full"
                );
            }
        }
    }


    // ==================================
    // POZOSTAŁE KOMENDY
    // ==================================

    else
    {
        if (
            tcpCommands
            != nullptr
        )
        {
            if (
                xQueueSend(
                    tcpCommands,
                    &command,
                    0
                ) != pdTRUE
            )
            {
                Serial.println(
                    "WARN TCP command queue full"
                );
            }
        }
    }
}

void tcpTask(void* parameter)
{
    

    bool serverStarted = false;

    WiFiClient client;

    char buffer[112] = {};
    size_t length = 0;
    bool discardLine = false;

    TcpResponse pendingResponse = {};
    size_t pendingOffset = 0;
    bool responsePending = false;


    // --------------------------------------------------
    // Reset połączenia
    // --------------------------------------------------

    auto resetConnection = [&]()
    {
        gcodeStorage.abortUpload();

        client.stop();

        length = 0;
        discardLine = false;

        responsePending = false;
        pendingOffset = 0;

        // Usuwamy odpowiedzi ze starego połączenia.
        if (tcpResponses != nullptr)
        {
            xQueueReset(
                tcpResponses
            );
        }

        if (tcpPosition != nullptr)
        {
            xQueueReset(
                tcpPosition
            );
        }

        // Usuwamy stare pozycje tabletu.
        if (tabletCommands != nullptr)
        {
            xQueueReset(
                tabletCommands
            );
        }

        // tcpCommands celowo nie czyścimy.
    };


    // --------------------------------------------------
    // Główna pętla TCP
    // --------------------------------------------------

    while (true)
    {
        // --------------------------------------------------
        // Brak Wi-Fi
        // --------------------------------------------------

        if (WiFi.status() != WL_CONNECTED)
        {
            resetConnection();

            if (serverStarted)
            {
                tcpServer.end();
                serverStarted = false;

                Serial.println(
                    "TCP server stopped"
                );
            }

            vTaskDelay(
                pdMS_TO_TICKS(100)
            );

            continue;
        }


        // --------------------------------------------------
        // Uruchomienie serwera po połączeniu Wi-Fi
        // --------------------------------------------------

        if (!serverStarted)
        {
            tcpServer.begin();
            serverStarted = true;

            Serial.println(
                "TCP server ready on port 5000"
            );
        }


        // --------------------------------------------------
        // Akceptowanie klienta
        // --------------------------------------------------

        if (!client || !client.connected())
        {
            resetConnection();

            client =
                tcpServer.available();

            if (
                client &&
                client.connected()
            )
            {
                client.setNoDelay(true);
                client.setTimeout(20);

                client.println(
                    "TCP CONNECTED"
                );

                Serial.println(
                    "TCP client connected"
                );

                char diagnostic[96];

                snprintf(
                    diagnostic,
                    sizeof(diagnostic),
                    "BOOT RESET=%d CRASH_STAGE=%u",
                    static_cast<int>(bootResetReason),
                    static_cast<unsigned>(lastCrashStage)
                );

                client.println(diagnostic);
            }
        }


        // --------------------------------------------------
        // Odbieranie danych
        // --------------------------------------------------

        if (
            client &&
            client.connected()
        )
        {
            int budget = 128;

            while (
                client.available() > 0 &&
                budget-- > 0
            )
            {
                int value =
                    client.read();

                if (value < 0)
                {
                    break;
                }

                char c =
                    static_cast<char>(
                        value
                    );


                // ==========================================
                // TRYB UPLOADU G-CODE
                // ==========================================

                if (gcodeStorage.isUploading())
                {
                    bool ok =
                        gcodeStorage.writeByte(
                            static_cast<uint8_t>(
                                c
                            )
                        );

                    if (!ok)
                    {
                        gcodeStorage.abortUpload();

                        sendResponse(
                            "ERR GCODE WRITE"
                        );

                        continue;
                    }

                    if (
                        gcodeStorage.getUploadRemaining()
                        == 0
                    )
                    {
                        if (
                            gcodeStorage.finishUpload()
                        )
                        {
                            sendResponse(
                                "GCODE STORED"
                            );

                        
                        }
                        else
                        {
                            sendResponse(
                                "ERR GCODE STORE"
                            );
                        }
                    }

                    continue;
                }

                // ==========================================
                // NORMALNY TRYB KOMEND TCP
                // ==========================================

                if (c == '\r')
                {
                    continue;
                }


                // ------------------------------------------
                // Koniec linii
                // ------------------------------------------

                if (c == '\n')
                {
                    if (
                        !discardLine &&
                        length > 0
                    )
                    {
                        buffer[length] = '\0';

                        processCommand(
                            buffer
                        );
                    }

                    // Przygotowanie na następną linię.
                    length = 0;
                    discardLine = false;
                }

                // ------------------------------------------
                // Zwykły znak linii
                // ------------------------------------------
                else if (!discardLine)
                {
                    if (
                        length
                        < sizeof(buffer) - 1
                    )
                    {
                        buffer[length++] = c;
                    }
                    else
                    {
                        // Linia za długa.
                        // Odrzucamy ją aż do następnego \n.
                        length = 0;
                        discardLine = true;
                    }
                }
            }
        }


        // --------------------------------------------------
        // Wysyłanie odpowiedzi do Pythona
        // --------------------------------------------------

        if (
            client &&
            client.connected()
        )
        {
            if (!responsePending)
            {
                // Najpierw ważne komunikaty.
                if (
                    tcpResponses != nullptr &&
                    xQueueReceive(
                        tcpResponses,
                        &pendingResponse,
                        0
                    ) == pdTRUE
                )
                {
                    responsePending = true;
                    pendingOffset = 0;
                }

                // Potem najnowsza pozycja.
                else if (
                    tcpPosition != nullptr &&
                    xQueueReceive(
                        tcpPosition,
                        &pendingResponse,
                        0
                    ) == pdTRUE
                )
                {
                    responsePending = true;
                    pendingOffset = 0;
                }
            }


            if (responsePending)
            {
                size_t total =
                    strlen(
                        pendingResponse.text
                    );


                if (pendingOffset < total)
                {
                    size_t remaining =
                        total - pendingOffset;


                    size_t written =
                        client.write(
                            reinterpret_cast<
                                const uint8_t*
                            >(
                                pendingResponse.text
                                + pendingOffset
                            ),
                            remaining
                        );


                    pendingOffset += written;
                }


                if (
                    pendingOffset >= total
                )
                {
                    responsePending = false;
                    pendingOffset = 0;
                }
            }
        }


        vTaskDelay(
            pdMS_TO_TICKS(2)
        );
    }
}



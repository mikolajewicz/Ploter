#pragma once

class NetworkManager
{
public:
    void begin();
    void maintainWiFi();
    static void otaTask(void* parameter);
};

extern NetworkManager networkManager;

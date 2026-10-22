#pragma once

#include <Arduino.h>
#include <LittleFS.h>

class GCodeStorage
{
public:
    bool begin();

    bool beginUpload(size_t size);

    bool writeByte(uint8_t byte);

    bool isUploading() const;

    bool finishUpload();

    void abortUpload();

    size_t getUploadRemaining() const;

    bool openProgram();

    bool readLine(String& line);

    void closeProgram();


private:
    File uploadFile_;
    File programFile_;

    bool uploading_ = false;

    size_t uploadTotal_ = 0;
    size_t uploadRemaining_ = 0;
};
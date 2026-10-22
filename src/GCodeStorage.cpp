#include "GCodeStorage.hpp"


bool GCodeStorage::begin()
{
    return LittleFS.begin(true);
}


bool GCodeStorage::beginUpload(size_t size)
{
    if (size == 0)
        return false;

    abortUpload();

    LittleFS.remove("/job.tmp");

    uploadFile_ =
        LittleFS.open(
            "/job.tmp",
            "w"
        );

    if (!uploadFile_)
        return false;

    uploadTotal_ = size;
    uploadRemaining_ = size;

    uploading_ = true;

    return true;
}


bool GCodeStorage::writeByte(uint8_t byte)
{
    if (!uploading_)
        return false;

    if (!uploadFile_)
        return false;

    if (uploadRemaining_ == 0)
        return false;

    if (uploadFile_.write(byte) != 1)
        return false;

    uploadRemaining_--;

    return true;
}


bool GCodeStorage::isUploading() const
{
    return uploading_;
}


size_t GCodeStorage::getUploadRemaining() const
{
    return uploadRemaining_;
}


bool GCodeStorage::finishUpload()
{
    if (!uploading_)
        return false;

    if (uploadRemaining_ != 0)
        return false;

    uploadFile_.close();

    uploading_ = false;

    LittleFS.remove("/job.gcode");

    return LittleFS.rename(
        "/job.tmp",
        "/job.gcode"
    );
}


void GCodeStorage::abortUpload()
{
    if (uploadFile_)
    {
        uploadFile_.close();
    }

    if (uploading_)
    {
        LittleFS.remove(
            "/job.tmp"
        );
    }

    uploading_ = false;

    uploadTotal_ = 0;
    uploadRemaining_ = 0;
}


bool GCodeStorage::openProgram()
{
    closeProgram();

    if (!LittleFS.exists("/job.gcode"))
        return false;

    programFile_ =
        LittleFS.open(
            "/job.gcode",
            "r"
        );

    return static_cast<bool>(
        programFile_
    );
}


bool GCodeStorage::readLine(String& line)
{
    if (!programFile_)
        return false;

    if (!programFile_.available())
        return false;

    line =
        programFile_.readStringUntil(
            '\n'
        );

    if (
        line.length() > 0 &&
        line[line.length() - 1] == '\r'
    )
    {
        line.remove(
            line.length() - 1
        );
    }

    return true;
}


void GCodeStorage::closeProgram()
{
    if (programFile_)
    {
        programFile_.close();
    }
}


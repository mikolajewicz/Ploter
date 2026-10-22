#pragma once

#include <Arduino.h>

#include "GCodeStorage.hpp"
#include "GCodeInterpreter.hpp"
#include "MotionManager.hpp"


class GCodeProgram
{
public:
    GCodeProgram(
        GCodeStorage& storage,
        MotionManager& motionManager
    );

    bool start();

    void update();

    void stop();

    bool isRunning() const;


private:
    bool loadNextLine();

    bool isMotionLine(
        const String& line
    ) const;

    void finish();


    GCodeStorage& storage_;
    MotionManager& motionManager_;

    GCodeInterpreter interpreter_;


    bool running_ = false;
    bool endOfFile_ = false;

    bool hasPendingLine_ = false;

    String pendingLine_;

    uint32_t lineNumber_ = 0;
};
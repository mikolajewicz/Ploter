#include "GCodeProgram.hpp"

extern void sendResponse(
    const char* message
);


GCodeProgram::GCodeProgram(
    GCodeStorage& storage,
    MotionManager& motionManager
)
    : storage_(storage),
      motionManager_(motionManager)
{
}


bool GCodeProgram::start()
{
    if (running_)
    {
        return false;
    }

    if (!storage_.openProgram())
    {
        return false;
    }

    // Fizyczna pozycja jest potrzebna tylko RAZ:
    // na początku programu.
    if (
        !motionManager_.calculateCurrentPosition()
    )
    {
        storage_.closeProgram();
        return false;
    }

    interpreter_.setPosition(
        motionManager_.getCurrentX(),
        motionManager_.getCurrentY()
    );

    // Tablet nie może walczyć z G-code.
    motionManager_.clearLiveTarget();

    running_ = true;
    endOfFile_ = false;

    hasPendingLine_ = false;
    pendingLine_ = "";

    lineNumber_ = 0;

    sendResponse(
        "GCODE RUN STARTED"
    );

    return true;
}


bool GCodeProgram::loadNextLine()
{
    if (hasPendingLine_)
    {
        return true;
    }

    String line;

    while (
        storage_.readLine(line)
    )
    {
        lineNumber_++;

        line.trim();

        // Pomijamy puste linie.
        if (line.length() == 0)
        {
            continue;
        }

        pendingLine_ = line;
        hasPendingLine_ = true;

        return true;
    }

    // Nie ma już więcej linii.
    endOfFile_ = true;

    storage_.closeProgram();

    return false;
}


bool GCodeProgram::isMotionLine(
    const String& line
) const
{
    String command = line;

    command.trim();
    command.toUpperCase();

    return
        command.startsWith("G0 ") ||
        command == "G0" ||

        command.startsWith("G00 ") ||
        command == "G00" ||

        command.startsWith("G1 ") ||
        command == "G1" ||

        command.startsWith("G01 ") ||
        command == "G01" ||

        command.startsWith("G2 ") ||
        command == "G2" ||

        command.startsWith("G02 ") ||
        command == "G02" ||

        command.startsWith("G3 ") ||
        command == "G3" ||

        command.startsWith("G03 ") ||
        command == "G03";
}


void GCodeProgram::update()
{
    if (!running_)
    {
        return;
    }


    // --------------------------------------------------
    // Doszliśmy do końca pliku
    // --------------------------------------------------

    if (endOfFile_)
    {
        // Program kończymy dopiero gdy ostatni
        // ruch/pauza naprawdę się zakończy.
        if (!motionManager_.isBusy())
        {
            finish();
        }

        return;
    }


    // --------------------------------------------------
    // Pobierz następną linię
    // --------------------------------------------------

    if (!loadNextLine())
    {
        return;
    }


    // --------------------------------------------------
    // G0 / G1 / G2 / G3
    // --------------------------------------------------

    if (
        isMotionLine(
            pendingLine_
        )
    )
    {
        // Poprzedni ruch MOŻE się nadal wykonywać.
        //
        // Potrzebujemy tylko:
        // - planner wolny
        // - brak już gotowej następnej trajektorii
        if (
            !motionManager_.canPlanNext()
        )
        {
            return;
        }

        interpreter_.processCommand(
            pendingLine_,
            motionManager_
        );

        hasPendingLine_ = false;

        return;
    }


    // --------------------------------------------------
    // Pozostałe komendy
    // --------------------------------------------------
    //
    // M3, M5, G4 itd. nie mogą zostać wykonane
    // z wyprzedzeniem.
    //
    // G1 X100
    // M5
    //
    // M5 musi nastąpić dopiero po zakończeniu G1.
    // --------------------------------------------------

    if (motionManager_.isBusy())
    {
        return;
    }

    interpreter_.processCommand(
        pendingLine_,
        motionManager_
    );

    hasPendingLine_ = false;
}


void GCodeProgram::finish()
{
    running_ = false;
    endOfFile_ = false;

    hasPendingLine_ = false;
    pendingLine_ = "";

    sendResponse(
        "GCODE RUN FINISHED"
    );
}


void GCodeProgram::stop()
{
    if (!running_)
    {
        return;
    }

    storage_.closeProgram();

    running_ = false;
    endOfFile_ = false;

    hasPendingLine_ = false;
    pendingLine_ = "";

    sendResponse(
        "GCODE RUN STOPPED"
    );
}


bool GCodeProgram::isRunning() const
{
    return running_;
}
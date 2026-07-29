#include<iostream>
#include <Arduino.h>
#pragma once

class MotorDriverPosition {
public:
    MotorDriverPosition(int StepPinNum, int DirPinNum, int EnablePinNum = -1);

private:
    int stepPin;
    int dirPin;
    int enablePin;  
};
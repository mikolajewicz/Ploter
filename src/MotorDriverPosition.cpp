#include<iostream>
#include <Arduino.h>
#include "MotorDriverPosition.hpp"

MotorDriverPosition::MotorDriverPosition(int StepPinNum, int DirPinNum, int EnablePinNum) {
    stepPin = StepPinNum;
    dirPin = DirPinNum;
    enablePin = EnablePinNum;

}






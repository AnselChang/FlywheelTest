#pragma once
#include <functional>

class TBH {

public:

    TBH(float gainConstant, float initialTargetRPM, std::function<float(float)> rpmToVoltFunction);
    float getNextMotorVoltage(float currentVelocityRPM);
    void setTargetRPM(float targetVelocityRPM);
    float getTargetRPM() {return targetRPM;}

private:

    std::function<float(float)> rpmToVolt;
    float gain;
    float targetRPM;
    float output = 0;
    float prevError = 0;
    float tbh;
    bool isFirstCrossover;

};
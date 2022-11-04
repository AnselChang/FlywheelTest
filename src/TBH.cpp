#include "TBH.h"
#include <math.h>

int sign(double x) {
    if (x >= 0) return 1;
    return -1;
}

double clamp(int value, double min, double max) {
    return fmin(max, fmax(min, value));
}


TBH::TBH(float gainConstant, float initialTargetRPM, std::function<float(float)> rpmToVoltFunction):
    rpmToVolt(rpmToVoltFunction),
    gain(gainConstant),
    targetRPM(initialTargetRPM)
{}

float TBH::getNextMotorVoltage(float currentVelocityRPM) {
    
    float error = targetRPM - currentVelocityRPM; // calculate the error;
    output += gain * error; // integrate the output

    output = clamp(output, 0, 12); // bound output to possible voltages
        
    if (sign(error) != sign(prevError)) { // if zero crossing,

        if (isFirstCrossover) { // First zero crossing after a new set velocity command
            // Set drive to the open loop approximation
            output = rpmToVolt(targetRPM);
        } else {
            output = 0.5 * (output + tbh); // Take Back Half
            tbh = output;// update Take Back Half variable
            isFirstCrossover = false;
        }
        
        prevError = error; // save the previous error
    }

    return output;

}

void TBH::setTargetRPM(float targetVelocityRPM) {
    targetRPM = targetVelocityRPM;
    isFirstCrossover = true;
}
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       anselchang                                                */
/*    Created:      11/4/2022, 1:05:34 AM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include <iostream>
#include <string>
#include <vector>
#include "VisualGraph.h"
#include "FixedRingQueue.h"
#include "SimplePID.h"
#include "TBH.h"

using namespace vex;

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain       Brain;

// define your global instances of motors and other devices here
motor flywheel1(PORT1, ratio6_1, true);
motor flywheel2(PORT3, ratio6_1, false);
motor_group flywheel(flywheel1, flywheel2);
controller c;

typedef struct Point {
    float rpm, volt;
} Point;

std::vector<Point> data = {
    {1615, 5},
    {1966, 6},
    {2306, 7},
    {2646, 8},
    {3054, 9},
    {3416, 10},
    {3751, 11},
    {4141, 12}
};

float voltToRpm(float volt) {
    
    int lowerBound = 0;
    while (lowerBound < data.size() - 2 && data[lowerBound+1].volt < volt) lowerBound++;
    
    float percent = (volt - data[lowerBound].volt) / (data[lowerBound+1].volt - data[lowerBound].volt);
    return data[lowerBound].rpm + (data[lowerBound+1].rpm - data[lowerBound].rpm) * percent;

}

float rpmToVolt(float rpm) {

    int lowerBound = 0;
    while (lowerBound < data.size() - 2 && data[lowerBound+1].rpm < rpm) lowerBound++;
    
    float percent = (rpm - data[lowerBound].rpm) / (data[lowerBound+1].rpm - data[lowerBound].rpm);
    return data[lowerBound].volt + (data[lowerBound+1].volt - data[lowerBound].volt) * percent;

}

TBH tbh(0.003, 3600, rpmToVolt);

void dec() {
    float target = tbh.getTargetRPM();
    if (target > 2000) tbh.setTargetRPM(target - 100);
}

void inc() {
    float target = tbh.getTargetRPM();
    if (target < 3550) tbh.setTargetRPM(target + 100);
}

void tune(float voltage) {

    flywheel.spin(forward, voltage, volt);
    while (!c.ButtonA.pressing()) {
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1,1);
        Brain.Screen.print(flywheel.velocity(rpm));
        wait(10, msec);
    }
    float sum = 0;
    int N = 200;
    for (int i = 0; i < N; i++) {
        sum += flywheel.velocity(rpm);
        wait(10, msec);
    }
    std::cout << voltage << ": " << (sum / N * 6) << std::endl;

}

int main() {


    c.ButtonDown.pressed(dec);
    c.ButtonUp.pressed(inc);

    VisualGraph g(0, 3600, 10, 400, 4);
    RingQueue averageSpeed(50);

    SimplePID pid(PIDParameters(5,0,0));

    int count = 0;
    float sd = 0;

    float y = 30;
    while(1) {

        double currentSpeed = flywheel.velocity(rpm) * 6;
        averageSpeed.push(currentSpeed);

        float motorInputVolts = tbh.getNextMotorVoltage(currentSpeed);
        flywheel.spin(forward, motorInputVolts, volt);
    
        g.push(currentSpeed, 0);
        g.push(tbh.getTargetRPM(), 1);
        g.push(voltToRpm(motorInputVolts), 2);
        g.push(voltToRpm(tbh.getTBH()), 3);
        g.display();
        Brain.Screen.setCursor(1,y);
        Brain.Screen.print(motorInputVolts);
        Brain.Screen.setCursor(2,y);
        Brain.Screen.print(currentSpeed);
        Brain.Screen.setCursor(3,y);
        Brain.Screen.print(averageSpeed.getAverage());
        Brain.Screen.setCursor(4,y);
        Brain.Screen.print(tbh.getTBH());
        Brain.Screen.setCursor(5,y);
        Brain.Screen.print(sd);

        count = (count + 1) % 100;
        if (count == 0) {
            sd = averageSpeed.standardDeviation();
        }
        
        // Allow other tasks to run
        this_thread::sleep_for(10);
    }
}

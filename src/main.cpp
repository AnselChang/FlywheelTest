/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       anselchang                                                */
/*    Created:      11/4/2022, 1:05:34 AM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include <string>
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

float voltToRpm(float x) {
    return -1.1932 * pow(x,4) + 38.3207 * pow(x,3) - 449.6780 * pow(x,2) + 2641.9012 * x - 4496.2554;
}

float rpmToVolt(float x) {
    return 0.0028*x + 0.7502;
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

int main() {
    
    c.ButtonDown.pressed(dec);
    c.ButtonUp.pressed(inc);

    VisualGraph g(0, 3600, 10, 400, 4);
    RingQueue averageSpeed(20);

    SimplePID pid(PIDParameters(5,0,0));

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
        
        // Allow other tasks to run
        this_thread::sleep_for(10);
    }
}

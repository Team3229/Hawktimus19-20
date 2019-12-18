// Author: Team 3229 Programming Subteam

#ifndef LIFT_H
#define LIFT_H

#include "Debug.h"

#include <iostream>
#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>
#include <ctre/phoenix/motorcontrol/ControlMode.h>

class Lift
{
public:
    Lift();
    ~Lift();
    void MoveLift(bool direction); //Moves lift up and down
    void StopLift(); //Stops lift
                                
private:
    ctre::phoenix::motorcontrol::can::WPI_VictorSPX * liftMotor;
    //frc::DigitalInput * topSwitch;
    //frc::DigitalInput * bottomSwitch;

    // port
    const int LIFT_PORT = 5;
    const int TOP_DIO = 0;
    const int BOTTOM_DIO = 1;

    // power constants
    const float LIFT_POWER = 1.0;
    const float SMOOTH_TIME = 0.25;
    const float DOWN_POWER = -0.5;
    const float HOLD_POWER = 0.15;

};

#endif // LIFT_H
// Author: Team 3229 Programming Subteam

#ifndef INTAKE_H
#define INTAKE_H

#include "Debug.h"

#include <iostream>
#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>
#include <ctre/phoenix/motorcontrol/ControlMode.h>
class Intake
{
public:
    Intake();
    ~Intake();
    void RunWheels(bool inOrOut); // run intake wheels
    void MoveIntake(bool upOrDown); // pivot intake on motor
    void StopWheels();
    void StopIntakePivot();

private:
    ctre::phoenix::motorcontrol::can::WPI_VictorSPX * intakeWheels; // Motor for wheels on intake
    ctre::phoenix::motorcontrol::can::WPI_VictorSPX * intakePivot; // Motor for pivot on intake
   
    //Setting intake ports
    const int WHEEL_PORT = 7;
    const int PIVOT_PORT = 6;
    
    //Setting intake pivot and wheel speed constants
    const float WHEEL_POWER = 1.0;
    const float WHEEL_SHOOT_POWER = -1.0;
    const float WHEEL_HOLD_POWER = 0.1;
    const float PIVOT_POWER = -0.5; // was .4
    const float PIVOT_DOWN_POWER = .25; // was -.25
    const float PIVOT_HOLD_POWER = -.1; // was .1

};

#endif // INTAKE_H
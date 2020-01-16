// Author: Team 3229 Programming Subteam

#include "Lift.h"

//Uses 1 VictorSPX motor

Lift::Lift()
{
    liftMotor = new VictorSPX(LIFT_PORT);
    liftMotor->ClearStickyFaults(0);
    liftMotor->ConfigOpenloopRamp(SMOOTH_TIME, 0);
}

Lift::~Lift()
{
    delete liftMotor;
}

void Lift::MoveLift(bool direction)
{
    if (direction == true) //&& topSwitch->Get() == false)
    {
        liftMotor->Set(ControlMode::PercentOutput, LIFT_POWER); //Lifts the lift up
        debug("Lift up\n");
    }
    else if (direction == false) //&& bottomSwitch->Get() == false)
    {
        liftMotor->Set(ControlMode::PercentOutput, DOWN_POWER); //Moves the lift down
        debug("Lift down\n");
    }
    else
        StopLift();
}

void Lift::StopLift()
{
    liftMotor->Set(ControlMode::PercentOutput, HOLD_POWER); //stops lift
}
// Author: Team 3229 Programming Subteam

#include "Intake.h"

Intake::Intake()
{
    intakeWheels = new ctre::phoenix::motorcontrol::can::WPI_VictorSPX(WHEEL_PORT); //Setting intake wheel port
    intakePivot = new ctre::phoenix::motorcontrol::can::WPI_VictorSPX(PIVOT_PORT); //Setting intake pivot port
    intakeWheels->ClearStickyFaults(0);
	intakePivot->ClearStickyFaults(0); 
} 

Intake::~Intake()
{
    delete intakeWheels;
    delete intakePivot;
}

void Intake::StopWheels()
{
    intakeWheels->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, WHEEL_HOLD_POWER); //holds cargo by running wheels
}

void Intake::MoveIntake(bool upOrDown)
{
    if (upOrDown == true)
    {
        intakePivot->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, PIVOT_POWER); //Points Intake up
        debug("Intake up\n");
    }
    else
    {
        intakePivot->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, PIVOT_DOWN_POWER); //Points Inatake down
        debug("Intake down\n");
    }
}

void Intake::RunWheels(bool inOrOut)
{
    if (inOrOut == true)
    {
        intakeWheels->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, WHEEL_POWER); //Spin wheels in
        debug("Wheels in\n");
    }
    else
    {
        intakeWheels->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, WHEEL_SHOOT_POWER); //Spin wheels out
        debug("Wheels out\n");
    }
}

void Intake::StopIntakePivot()
{
    intakePivot->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, PIVOT_HOLD_POWER);//holds intake pivot in place 
}

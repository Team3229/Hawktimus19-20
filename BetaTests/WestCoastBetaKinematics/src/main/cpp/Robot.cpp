/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() 
{
  m_auto.AddOptions();
  m_auto.SetupAutoCommands();
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() 
{
  m_auto.AutoInit();
}

void Robot::AutonomousPeriodic() 
{
  if(!m_auto.autodone)
    m_auto.AutoPeriodic();
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic()
{
  const auto y1 = -m_controller.GetY(frc::GenericHID::kLeftHand) * Drivetrain::kMaxSpeed;        //left stick vertical
  const auto x2 = -m_controller.GetX(frc::GenericHID::kRightHand) * Drivetrain::kMaxAngularSpeed;//right stick horizontal

  m_drive.Drive(y1,x2);
  if(m_controller.GetAButton())
  {
    m_drive.SetEncoder(0);
  }
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

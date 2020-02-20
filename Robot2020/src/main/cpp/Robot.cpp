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
  frc::SmartDashboard::PutNumber("RPM",10000);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() 
{}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() 
{
  m_auto.SetupPlayback();
}

void Robot::AutonomousPeriodic() 
{
  m_auto.AutoPeriodic();
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() 
{
  /*
  drivetrain
  */
  const auto m_y1 = -m_driveController.GetY(frc::GenericHID::kRightHand);
  const auto m_x2 = -m_driveController.GetX(frc::GenericHID::kLeftHand);
  if (kDRIVEDEADBAND > std::abs(m_y1) && kDRIVEDEADBAND > std::abs(m_x2))
  {
    m_drive.StopMotor();
  }
  else
  {
    //increase rotation speed at high velocity
    double rotateOffset = 1+std::abs(m_y1); 
    m_drive.Drive(m_y1*m_drive.kMaxSpeed,m_x2*m_drive.kMaxAngularSpeed*rotateOffset);
  }
  m_drive.UpdateOdometry();

  //power cell manipulations
  double turretTurn = -m_maniController.GetX(frc::GenericHID::kRightHand);
  m_turret.Turn(turretTurn);
  double hoodAdjust = -m_maniController.GetY(frc::GenericHID::kLeftHand);
  m_shooter.adjustHood(units::inch_t(hoodAdjust));
  
  double rpm = frc::SmartDashboard::GetNumber("RPM",6000);
  m_shooter.adjustFWSpeed(rpm);
  //testing calculations
  //shooter.calcRPM(limelight.calcDist());
  //shooter.adjustFWSpeed(shooter.calcRPM(limelight.calcDist()));
  
/*might need gyro to confirm it's possible to find the targer before this
  if(limelight.aimOperation() && controller.GetXButton())
  {
    limelight.scoreOperation();
  } 
*/

//intake
/*
  if(m_maniController.GetXButton())
    m_intake.extendIntake();
  if(m_maniController.GetYButton())
    m_intake.retractIntake();
*/
  (m_maniController.GetAButton()) ? (m_intake.reverseIntake())
  : (m_intake.runIntake());  
}

void Robot::TestInit() 
{
  m_auto.SetupRecording();
}
void Robot::TestPeriodic() 
{
  m_auto.Record();
}
void Robot::DisabledInit()
{
  m_auto.CloseFile();
}
#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

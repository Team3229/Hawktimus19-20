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
  frc::SmartDashboard::PutNumber("RPM",0);
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
  const auto m_x1 = -m_driveController.GetX(frc::GenericHID::kLeftHand);
  if (kDRIVEDEADBAND > std::abs(m_y1) && kDRIVEDEADBAND > std::abs(m_x1))
  {
    debugDashNum("drive", 0);
    m_drive.StopMotor();
  }
  else
  {
    debugDashNum("drive",1);
    //increase rotation speed at high velocity
    //double rotateOffset = 1+std::abs(m_y1); 
    m_drive.Drive(m_y1*m_drive.kMaxSpeed,m_x1*m_drive.kMaxAngularSpeed);
  }
  m_drive.UpdateOdometry();

  //power cell manipulations

  double turretTurn = -m_maniController.GetX(frc::GenericHID::kRightHand)/5;
  (std::abs(turretTurn) > .1) ? (m_turret.Turn(turretTurn))
  : (m_turret.Turn(0));
  debugDashNum("Turret input", turretTurn);

  double hoodAdjust = -m_maniController.GetY(frc::GenericHID::kLeftHand);
  m_shooter.adjustHood(units::inch_t(hoodAdjust));
  debugDashNum("Hood Actuator Control in", hoodAdjust);

  double rpm = frc::SmartDashboard::GetNumber("RPM",6000);
  if(m_maniController.GetTriggerAxis(frc::GenericHID::kRightHand)>.2)
  {
    m_shooter.adjustFWSpeed(30);
    m_shooter.feedShooter();
  }
  else
  {
    (m_maniController.GetBumper(frc::GenericHID::kLeftHand)) ? 
    (m_shooter.reverseFeed())
    : (m_shooter.stopFeed());
  }
  

  
  //might need gyro to confirm it's possible to find the targer before this
  /*
  shooter
  left bumper -> force reverse & maintain state of shooter
  
  */
 /*
  if(m_maniController.GetBumper(frc::GenericHID::kLeftHand))
  {
    m_shooter.reverseFeed();
    m_shooter.maintainState();
  }
  else if(m_limelight.aimOperation())
  { 
    if(m_maniController.GetAButton() || 
      m_maniController.GetTriggerAxis(frc::GenericHID::kRightHand) > .1)
    {
      int povRead = m_maniController.GetPOV();
      (povRead != -1 || m_maniController.GetAButton()) ? (m_limelight.scoreWithPOV(povRead))
      : (m_limelight.scoreOperation());
    }
    else if(m_maniController.GetBButton())
    {
      m_limelight.scoreOperation();
    }
    else
    {
      m_shooter.stopShooter();
      m_shooter.stopFeed();
    }
  }
  else
  {
    m_shooter.stopFeed();
    m_shooter.stopShooter();
  }
*/

//intake
  /*
  if(m_maniController.GetXButton())
    m_intake.extendIntake();
  if(m_maniController.GetYButton())
    m_intake.retractIntake();
  */
  (m_maniController.GetBumper(frc::GenericHID::kRightHand)) ? (m_intake.forceRunIntake(-.7))
  : (m_intake.forceRunIntake(0));  
}

void Robot::TestInit() 
{
  m_auto.SetupRecording();
}
void Robot::TestPeriodic() 
{
  if (m_recordMode) { // recording
    TeleopPeriodic();

    // Populate struct
    m_auto.autocommand->drive_rightY = (double) m_y1;
    m_auto.autocommand->drive_leftX = (double) m_x1;
    // m_auto.autocommand->drive_leftX = 
    // m_auto.autocommand->drive_rightX = 
    m_auto.autocommand->drive_AButton = m_driveController.GetAButton();
    m_auto.autocommand->drive_BButton = m_driveController.GetBButton();
    m_auto.autocommand->drive_XButton = m_driveController.GetXButton();
    m_auto.autocommand->drive_YButton = m_driveController.GetYButton();
    m_auto.autocommand->drive_RightBumper =
        m_driveController.GetBumper(frc::GenericHID::kRightHand);
    m_auto.autocommand->drive_LeftBumper =
        m_driveController.GetBumper(frc::GenericHID::kLeftHand);
    m_auto.autocommand->drive_RightTriggerAxis =
        m_driveController.GetTriggerAxis(frc::GenericHID::kRightHand);
    m_auto.autocommand->drive_LeftTriggerAxis =
        m_driveController.GetTriggerAxis(frc::GenericHID::kLeftHand);
    // m_auto.autocommand->mani_leftY = d2_leftY;
    // m_auto.autocommand->mani_rightY = d2_rightY;
    m_auto.autocommand->mani_AButton = m_maniController.GetAButton();
    m_auto.autocommand->mani_BButton = m_maniController.GetBButton();
    m_auto.autocommand->mani_XButton = m_maniController.GetXButton();
    m_auto.autocommand->mani_YButton = m_maniController.GetYButton();
    m_auto.autocommand->mani_RightBumper =
        m_maniController.GetBumper(frc::GenericHID::kRightHand);
    m_auto.autocommand->mani_LeftBumper =
        m_maniController.GetBumper(frc::GenericHID::kLeftHand);
    m_auto.autocommand->mani_RightTriggerAxis =
        m_maniController.GetTriggerAxis(frc::GenericHID::kRightHand);
    m_auto.autocommand->mani_LeftTriggerAxis =
        m_maniController.GetTriggerAxis(frc::GenericHID::kLeftHand);

    // Write current struct to file
    m_auto.Record();
  }
}
void Robot::DisabledInit()
{
  m_auto.CloseFile();
}
#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

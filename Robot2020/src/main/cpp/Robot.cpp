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

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() 
{
  m_auto.SetupPlayback();
}

void Robot::AutonomousPeriodic() 
{
  m_auto.ReadFile(m_controllerInputs);
  ExecuteControls();
}

void Robot::TeleopInit() 
{
  m_auto.CloseFile();
}

void Robot::TeleopPeriodic() 
{
  // Populate controller struct
  m_controllerInputs->drive_rightY = m_driveController.GetY(frc::GenericHID::kRightHand);
  m_controllerInputs->drive_rightX = m_driveController.GetX(frc::GenericHID::kRightHand);
  m_controllerInputs->drive_leftY = m_driveController.GetY(frc::GenericHID::kLeftHand);
  m_controllerInputs->drive_leftX = m_driveController.GetX(frc::GenericHID::kLeftHand);
  m_controllerInputs->drive_AButton = m_driveController.GetAButton();
  m_controllerInputs->drive_BButton = m_driveController.GetBButton();
  m_controllerInputs->drive_XButton = m_driveController.GetXButton();
  m_controllerInputs->drive_YButton = m_driveController.GetYButton();
  m_controllerInputs->drive_RightBumper =
      m_driveController.GetBumper(frc::GenericHID::kRightHand);
  m_controllerInputs->drive_LeftBumper =
      m_driveController.GetBumper(frc::GenericHID::kLeftHand);
  m_controllerInputs->drive_RightTriggerAxis =
      m_driveController.GetTriggerAxis(frc::GenericHID::kRightHand);
  m_controllerInputs->drive_LeftTriggerAxis =
      m_driveController.GetTriggerAxis(frc::GenericHID::kLeftHand);
  m_controllerInputs->drive_POV = m_driveController.GetPOV();
  m_controllerInputs->mani_rightY = m_maniController.GetY(frc::GenericHID::kRightHand);
  m_controllerInputs->mani_rightX = m_maniController.GetX(frc::GenericHID::kRightHand);
  m_controllerInputs->mani_leftY = m_maniController.GetY(frc::GenericHID::kLeftHand);
  m_controllerInputs->mani_leftX = m_maniController.GetX(frc::GenericHID::kLeftHand);
  m_controllerInputs->mani_AButton = m_maniController.GetAButton();
  m_controllerInputs->mani_BButton = m_maniController.GetBButton();
  m_controllerInputs->mani_XButton = m_maniController.GetXButton();
  m_controllerInputs->mani_YButton = m_maniController.GetYButton();
  m_controllerInputs->mani_RightBumper =
      m_maniController.GetBumper(frc::GenericHID::kRightHand);
  m_controllerInputs->mani_LeftBumper =
      m_maniController.GetBumper(frc::GenericHID::kLeftHand);
  m_controllerInputs->mani_RightTriggerAxis =
      m_maniController.GetTriggerAxis(frc::GenericHID::kRightHand);
  m_controllerInputs->mani_LeftTriggerAxis =
      m_maniController.GetTriggerAxis(frc::GenericHID::kLeftHand);
  m_controllerInputs->mani_POV = m_maniController.GetPOV();
  
  m_limelight.limelightDash();
  m_shooter.shooterDash();
  m_drive.drivetrainDash();
  m_turret.turretDash();

  ExecuteControls();
}

void Robot::TestInit() 
{
  m_auto.SetupRecording();
}

void Robot::TestPeriodic() 
{
  if (m_recordMode) { // recording
    // Run TeleOp as normal
    TeleopPeriodic();
    // Write current struct to file
    m_auto.Record(m_controllerInputs);
  }
}

void Robot::DisabledInit()
{
  m_auto.CloseFile();
}

// TeleOp
void Robot::ExecuteControls()
{
  // Drive - ADD POWER CURVE
  if (kDRIVEDEADBAND > std::abs(m_controllerInputs->drive_rightY) && 
      kDRIVEDEADBAND > std::abs(m_controllerInputs->drive_leftX)) {
    m_drive.StopMotor();
  } else {
    m_drive.Drive(m_controllerInputs->drive_rightY*m_drive.kMaxSpeed,
                  -m_controllerInputs->drive_leftX*m_drive.kMaxAngularSpeed);
  }
  m_drive.UpdateOdometry();

  // Running the shooter
  if (m_controllerInputs->mani_RightTriggerAxis > .1) {
    m_shooter.runShooter();
  } else { //**manual shooter control
    m_shooter.stopShooter();
    m_limelight.limelightLED(1);
    m_limelight.limelightPipeLine(1);
  }
   
  // Aiming the shooter
  if (std::abs(m_controllerInputs->mani_rightX) > .1) {
    m_turret.Turn(m_controllerInputs->mani_rightX/5);
  } else {
    m_turret.Turn(0);
  }

  // Control the feeder
  if (m_controllerInputs->mani_LeftTriggerAxis > .1) {
    m_shooter.feedShooter();
  } else if (m_controllerInputs->mani_RightBumper) { 
    m_shooter.reverseFeed();
  } else {
    m_shooter.stopFeed();
  }

  // Hood control
  if (abs(m_controllerInputs->mani_leftY) > kDRIVEDEADBAND) {
    if (m_controllerInputs->mani_leftY < 0) {
      m_shooter.incrementalHood(HOOD_INCRIMENT);
    } else if (m_controllerInputs->mani_leftY > 0) {
      m_shooter.incrementalHood(-HOOD_INCRIMENT);
    }
  } else {
    m_shooter.maintainHood();
  }

  if(m_controllerInputs->mani_AButton){
    m_intake.extendIntake();
  } else if (m_controllerInputs->mani_BButton) {
    m_intake.retractIntake();
  }
  // Run the intake - NEED TO EXTEND
  if (m_controllerInputs->mani_LeftBumper) {
    m_intake.forceRunIntake(-.7);
  } else {
    m_intake.forceRunIntake(0); 
  }
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif 
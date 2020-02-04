/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// Author: Team 3229 Programming Subteam

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() 
{
  /*
  m_chooser.SetDefaultOption("With Gyro (Field Oriented)", kDriveNameDefault);
  m_chooser.AddOption("Without Gyro (Robot Oriented)", kDriveNameCustom);
  */
  frc::SmartDashboard::PutString("Drive Mode", "Without Gyro");
  frc::SmartDashboard::PutString("Current Template", m_template);

  // Auto
  autoChooser.SetDefaultOption("Auto Mode", "Read Mode");
  autoChooser.AddOption("Auto Mode", "Write Mode");

  // // Use for file handling
  // FILE * f;
  // char buffer[] = {'x', 'y', 'z'};
  // f = fopen("/home/lvuser/test.bin", "w");
  // fwrite(buffer, sizeof(char), sizeof(buffer), f);
  // fclose(f);

}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() 
{
  // if (autoChooser.GetSelected() == "Read Mode")
  //   recordMode = false;
  // else 
  //   recordMode = true;

  autoMode.SetupReading();
    
  chassis.ResetGyro();
  debug("Sandstorm starting...\n");
}

void Robot::AutonomousPeriodic() 
{
  if (!recordMode) { // playback
    autoMode.ReadFile();
    autoMode.AutoPeriodic();
  }
}

void Robot::TeleopInit() 
{
  autoMode.CloseFile();
  //Needs to be removed before comp:
  //chassis.ResetGyro();
  debug("TeleOp starting...\n");
}

void Robot::TeleopPeriodic()
{    
  //Update controller axis values
  d1_leftY = xbox1.GetY(frc::GenericHID::kLeftHand);
  d1_leftX = xbox1.GetX(frc::GenericHID::kLeftHand);
  d1_rightX = xbox1.GetX(frc::GenericHID::kRightHand);
  
  d2_leftY = xbox2.GetY(frc::GenericHID::kLeftHand);
  d2_rightY = xbox2.GetY(frc::GenericHID::kRightHand);

  // DRIVE
  debug("Gyro angle: " << chassis.TestGyro() << "\n");
  if(abs(d1_leftX) > DEAD_BAND || abs(d1_leftY) > DEAD_BAND || abs(d1_rightX) > DEAD_BAND )
	{
    if (m_driveWithGyro == true)
      chassis.Drive(d1_leftY, d1_leftX, d1_rightX); // drives robot with mecanum chassis + gyro
    else
      chassis.DriveWithoutGyro(d1_leftY, d1_leftX, d1_rightX); // drives mecanum without gyro
	}
	else
  {
    if (m_usingVision == false)
      chassis.Stop(); // stops driving
  }
  
  // swap robot and field orient with button
  if (xbox1.GetTriggerAxis(frc::GenericHID::kRightHand) > DEAD_BAND)
    SwitchDriveMode();

  // speed changer 
  // BOTH CONTROLLERS NOW HAVE ACCESS TO THESE
  if (xbox1.GetAButton())
  {
    chassis.ChangeSpeed(2); // normal speed
    m_lastUsedSpeed = 2;
  }

  if (xbox1.GetBButton())
  {
    chassis.ChangeSpeed(1); // slow speed
    m_lastUsedSpeed = 1;
  }

  if (xbox1.GetXButton())
  {
    chassis.ChangeSpeed(3); // fast
    m_lastUsedSpeed = 3;
  }

  // PNEUMATICS
  // climber
  air.ControlComp();
  if (xbox1.GetBumper(frc::GenericHID::kRightHand))
    air.MoveFrontClimb(); // toggle front climbing poles

  if (xbox1.GetBumper(frc::GenericHID::kLeftHand))
    air.MoveBackClimb(); // toggle back climbing poles

  // INTAKE OPERATION
  // wheels
  if (xbox2.GetTriggerAxis(frc::GenericHID::kLeftHand) > DEAD_BAND)
    intake.RunWheels(true); // wheels in
  else if (xbox2.GetTriggerAxis(frc::GenericHID::kRightHand) > DEAD_BAND)
    intake.RunWheels(false); // wheels out
  else 
    intake.StopWheels();

  // pivoting the intake
  if (abs(d2_leftY) > DEAD_BAND)
  {
    if (d2_leftY < 0)
      intake.MoveIntake(true); // pivot intake up
    else
      intake.MoveIntake(false); // pivot intake down
  }
  else 
    intake.StopIntakePivot(); // holds intake in place


  // LIMELIGHT VISION TRACKING AND GYRO ALIGNMENT
  // robot will stop moving when target is in desired range/orientation
  visionSystem.GetValues();
  if (xbox1.GetYButton())
  {
    m_usingVision = true;
    chassis.ChangeSpeed(2); //normal
    chassis.DetermineTarget(m_template);
    if (chassis.CanTurn())
      chassis.TurnToTarget();
    else
      visionSystem.SeekTarget(); 
  }
  else
  {
    m_usingVision = false;
    chassis.ChangeSpeed(m_lastUsedSpeed);
  }

  // LIFT OPERATION
  if (abs(d2_rightY) > DEAD_BAND && m_lockLift == false)
  {
    if (d2_rightY < 0)
      lift.MoveLift(true); // moves lift up
    else
      lift.MoveLift(false); // moves lift down
  }
  else 
    lift.StopLift(); // holds lift in place
  
  //toggle angle mode (Rocket Hatch vs. other)
  if (xbox2.GetXButton())
  {
    if (m_template == "Other")
      m_template = "Rocket Hatch";
    else 
      m_template = "Other";  
    frc::SmartDashboard::PutString("Current Template", m_template);
    frc::Wait(0.25);
  }
}

void Robot::TestInit() {
  autoMode.SetupWriting();
}

void Robot::TestPeriodic() {
  if (recordMode) { // recording
    TeleopPeriodic();

    // Populate struct
    autoMode.autocommand->xbox1_leftY = d1_leftY;
    autoMode.autocommand->xbox1_leftX = d1_leftX;
    autoMode.autocommand->xbox1_rightX = d1_rightX;
    autoMode.autocommand->xbox1_AButton = xbox1.GetAButton();
    autoMode.autocommand->xbox1_BButton = xbox1.GetBButton();
    autoMode.autocommand->xbox1_XButton = xbox1.GetXButton();
    autoMode.autocommand->xbox1_YButton = xbox1.GetYButton();
    autoMode.autocommand->xbox1_RightBumper = xbox1.GetBumper(frc::GenericHID::kRightHand);
    autoMode.autocommand->xbox1_LeftBumper = xbox1.GetBumper(frc::GenericHID::kLeftHand);
    autoMode.autocommand->xbox1_RightTriggerAxis = xbox1.GetTriggerAxis(frc::GenericHID::kRightHand);
    autoMode.autocommand->xbox2_leftY = d2_leftY;
    autoMode.autocommand->xbox2_rightY = d2_rightY;
    autoMode.autocommand->xbox2_XButton = xbox2.GetXButton();
    autoMode.autocommand->xbox2_YButton = xbox2.GetYButton();
    autoMode.autocommand->xbox2_RightBumper = xbox2.GetBumper(frc::GenericHID::kRightHand);
    autoMode.autocommand->xbox2_LeftBumper = xbox2.GetBumper(frc::GenericHID::kLeftHand);
    autoMode.autocommand->xbox2_RightTriggerAxis = xbox2.GetTriggerAxis(frc::GenericHID::kRightHand);
    autoMode.autocommand->xbox2_LeftTriggerAxis = xbox2.GetTriggerAxis(frc::GenericHID::kLeftHand);

    // Write current struct to file
    autoMode.WriteFile();
  }
}

void Robot::DisabledInit() {
  autoMode.CloseFile();
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

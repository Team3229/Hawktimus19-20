/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// Author: Team 3229 Programming Subteam

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

// Our needed includes
#include <frc/XboxController.h>
#include <Math.h>

// Subsystem includes
#include "DriveSystem.h"
#include "Camera.h"
#include "Limelight.h"
#include "Lift.h"
#include "Intake.h"
#include "Pneumatics.h"
#include "Debug.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

 private:
  // SmartDashboard setup
  frc::SendableChooser<std::string> m_chooser;
  const std::string kDriveNameDefault = "With Gyro";
  const std::string kDriveNameCustom = "Without Gyro";
  std::string m_driveSelected;

  // Constants
  const int XBOX_USB_DRIVER_1 = 0;
  const int XBOX_USB_DRIVER_2 = 1;
  const float DEAD_BAND = 0.1;

  // Controller variables
  double d1_leftY, d1_leftX, d1_rightX, d2_leftY, d2_rightY;
  bool m_driveWithGyro = false; //Update init driver station message
  bool m_usingVision = false;
  bool m_lockLift = false; // Locks the 2nd driver from using lift
  int m_lastUsedSpeed = 2; // keeps track of last used speed setting, initialized to normal speed
  std::string m_template = "Other"; // 2 templates that the manipulation driver can switch between: "Other" and "Rocket Hatch"

  // Objects of subsystems
  DriveSystem chassis{};

  Limelight visionSystem{&chassis};

  Lift lift{};

  Intake intake{};

  Pneumatics air{};
  
  frc::XboxController xbox1 {XBOX_USB_DRIVER_1}; //Chassis driver
  frc::XboxController xbox2 {XBOX_USB_DRIVER_2}; //Manipulation driver

  Camera driveCam{};
  
  //Toggle robot/field orient
  void SwitchDriveMode()
  {
    debug("Drive mode switched...\n");
    if (m_driveWithGyro == true)
    {
      frc::SmartDashboard::PutString("Drive Mode", "Without Gyro");
      m_driveWithGyro = false;
    }
    else 
    {
      frc::SmartDashboard::PutString("Drive Mode", "With Gyro");
      m_driveWithGyro = true;
    }
    frc::Wait(0.5);
  }
};

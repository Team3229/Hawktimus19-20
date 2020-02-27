/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include "powercell/Limelight.h"
#include "powercell/Intake.h"
#include "drive/Drivetrain.h"
#include "Auto.h"
#include "Camera.h"

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/XboxController.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;
  void DisabledInit() override;

  void ExecuteControls();
 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  cmd * m_controllerInputs = new cmd;

  frc::XboxController m_driveController{0};
  frc::XboxController m_maniController{1};

  Intake m_intake;
  Turret m_turret;
  Shooter m_shooter;
  Limelight m_limelight{&m_turret,&m_shooter};

  Drivetrain m_drive;

  Auto m_auto{&m_drive,&m_turret,&m_shooter,&m_limelight,&m_intake};
  const bool m_recordMode = true; // use this to force disable recording, useful at competitions
  
  Camera m_camera;
  double m_x1,m_y1;
  const int kDRIVEDEADBAND = .1; 
};

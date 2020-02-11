#ifndef AUTO_H
#define AUTO_H

// includes
#include <iostream>
#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/XboxController.h>
#include <Math.h>
#include "frc/Timer.h"

#include "DriveSystem.h"
#include "Intake.h"
#include "Pneumatics.h"
#include "Limelight.h"
#include "Lift.h"

#include "CaptureFile.h"
#include "Debug.h"

class Auto {
private:
  bool autoDone = false;

  // Pass in our subsystems
  DriveSystem *autoChassis;
  Limelight *autoVisionSystem;
  Lift *autoLift;
  Intake *autoIntake;
  Pneumatics *autoAir;

  // files stuff
  // Use .aut file extension
  std::string defaultFileName = "defaultAutoPath.aut";
  std::string inputFileName;
  const bool WRITE = true;
  const bool READ = false;
  CaptureFile cmdFile {};

  // Method of storing and replaying drivers inputs
  struct cmd {
    // Driver 1
    float xbox1_leftY;
    float xbox1_leftX;
    float xbox1_rightX;
    bool xbox1_AButton;
    bool xbox1_BButton;
    bool xbox1_XButton;
    bool xbox1_YButton;
    bool xbox1_RightBumper;
    bool xbox1_LeftBumper;
    float xbox1_RightTriggerAxis;
    // Driver 2
    float xbox2_leftY;
    float xbox2_rightY;
    bool xbox2_XButton;
    bool xbox2_YButton;
    bool xbox2_RightBumper;
    bool xbox2_LeftBumper;
    float xbox2_RightTriggerAxis;
    float xbox2_LeftTriggerAxis;
  };

  // TeleOp stuff for driving
  const float DEAD_BAND = 0.1;
  double d1_leftY, d1_leftX, d1_rightX, d2_leftY,
      d2_rightY;                // Controller variables
  bool m_driveWithGyro = false; // Update init driver station message
  bool m_usingVision = false;
  bool m_lockLift = false; // Locks the 2nd driver from using lift
  int m_lastUsedSpeed = 2;
  std::string m_template = "Other";

public:
  Auto(DriveSystem *c, Pneumatics *a, Lift *l, Limelight *v, Intake *i);
  ~Auto();
  void SetupAuto();
  void SetupReading();
  void ReadFile();
  void SetupRecording();
  void Record();
  void CloseFile();
  void AutoPeriodic();

  cmd * autocommand = new cmd;

  void SwitchDriveMode() {
    debug("Drive mode switched...\n");
    if (m_driveWithGyro == true) {
      frc::SmartDashboard::PutString("Drive Mode", "Without Gyro");
      m_driveWithGyro = false;
    } else {
      frc::SmartDashboard::PutString("Drive Mode", "With Gyro");
      m_driveWithGyro = true;
    }
    frc::Wait(0.5);
  }
};

#endif // AUTO_H
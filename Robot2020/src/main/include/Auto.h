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
  // MAKE SURE TO UPDATE CONSTRUCTOR ARGS AND SUBSYSTEMS FOR THIS YEAR

  // EXAMPLE FROM ROBOT2019
  DriveSystem *autoChassis;
  Limelight *autoVisionSystem;
  Lift *autoLift;
  Intake *autoIntake;
  Pneumatics *autoAir;

  // files stuff
  // Use .aut file extension
  std::string defaultFileName = "defaultAutoPath.aut";
  std::string driverStationText = "Auto file path: /home/lvuser/";
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
    float xbox1_rightY;
    bool xbox1_AButton;
    bool xbox1_BButton;
    bool xbox1_XButton;
    bool xbox1_YButton;
    bool xbox1_RightBumper;
    bool xbox1_LeftBumper;
    float xbox1_RightTriggerAxis;
    float xbox1_LeftTriggerAxis;
    // Driver 2
    float xbox2_leftY;
    float xbox2_rightY;
    bool xbox2_AButton;
    bool xbox2_BButton;
    bool xbox2_XButton;
    bool xbox2_YButton;
    bool xbox2_RightBumper;
    bool xbox2_LeftBumper;
    float xbox2_RightTriggerAxis;
    float xbox2_LeftTriggerAxis;
  };

  // TeleOp stuff for driving
  // TELEOP VARS GO HERE

public:
  Auto(DriveSystem *c, Pneumatics *a, Lift *l, Limelight *v, Intake *i);
  ~Auto();
  void SetupPlayback();
  void ReadFile();
  void SetupRecording();
  void Record();
  void CloseFile();
  void AutoPeriodic();

  cmd * autocommand = new cmd;

};

#endif // AUTO_H
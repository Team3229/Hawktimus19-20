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

#include "drive/Drivetrain.h"
#include "powercell/Limelight.h"
#include "powercell/Intake.h"

#include "CaptureFile.h"
#include "Debug.h"

class Auto {
private:
  bool autoDone = false;

  // Pass in our subsystems
  // MAKE SURE TO UPDATE CONSTRUCTOR ARGS AND SUBSYSTEMS FOR THIS YEAR

  // EXAMPLE FROM ROBOT2019
  Drivetrain *autoChassis;
  Limelight *autoVisionSystem;
  Shooter *autoShooter;
  Intake *autoIntake;
  Turret *autoTurret;

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
    // Driver 1 DRIVE TRAIN
    double drive_leftX;
    double drive_rightX;
    double drive_leftY;
    double drive_rightY;
    bool drive_AButton;
    bool drive_BButton;
    bool drive_XButton;
    bool drive_YButton;
    bool drive_RightBumper;
    bool drive_LeftBumper;
    float drive_RightTriggerAxis;
    float drive_LeftTriggerAxis;
    // Driver 2 MANIPULATION
    float mani_leftY;
    float mani_rightY;
    bool mani_AButton;
    bool mani_BButton;
    bool mani_XButton;
    bool mani_YButton;
    bool mani_RightBumper;
    bool mani_LeftBumper;
    float mani_RightTriggerAxis;
    float mani_LeftTriggerAxis;
  };

  // TeleOp stuff for driving
  // TELEOP VARS GO HERE

public:
  Auto(Drivetrain *c, Turret *t, Shooter *s, Limelight *v, Intake *i);
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
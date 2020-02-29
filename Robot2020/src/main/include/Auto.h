#ifndef AUTO_H
#define AUTO_H

// includes
#include <iostream>
#include <string>

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/XboxController.h>
#include <Math.h>
#include "frc/Timer.h"

#include "drive/Drivetrain.h"
#include "powercell/Limelight.h"
#include "powercell/Intake.h"

#include "CaptureFile.h"
#include "Debug.h"
#include "ControllerInputs.h"

class Auto {
private:
  bool autoDone = false;

  // Pass in our subsystems
  // MAKE SURE TO UPDATE CONSTRUCTOR ARGS AND SUBSYSTEMS FOR THIS YEAR

  // EXAMPLE FROM ROBOT2019
  Drivetrain * autoChassis;
  Limelight * autoVisionSystem;
  Shooter * autoShooter;
  Intake * autoIntake;
  Turret * autoTurret;

  // files stuff
  // Use .aut file extension
  std::string defaultFileName = "defaultAutoPath.aut";
  std::string driverStationText = "Auto file path: /home/lvuser/";
  std::string inputFileName = "driveAndShoot";
  const bool WRITE = true;
  const bool READ = false;
  CaptureFile cmdFile {};

public:
  Auto(Drivetrain *c, Turret *t, Shooter *s, Limelight *v, Intake *i);
  ~Auto();
  void SetupPlayback();
  void ReadFile(cmd * inputs);
  void SetupRecording();
  void Record(cmd * inputs);
  void CloseFile();
  // void AutoPeriodic(cmd * inputs);

};

#endif // AUTO_H
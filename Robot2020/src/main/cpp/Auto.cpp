// 2020 test for capture/replay autonomous system

#include "Auto.h"

Auto::Auto(Drivetrain *c, Turret *t, Shooter *s, Limelight *v, Intake *i) {
  // Passes in subsystems from Robot
  autoChassis = c;
  autoTurret = t;
  autoShooter = s;
  autoVisionSystem = v;
  autoIntake = i;

  // Setup SmartDashboard
  frc::SmartDashboard::PutString("Auto files:\n", stationFileList);
  frc::SmartDashboard::PutString(stationText, defaultFileName);
}

Auto::~Auto() {
  delete autoChassis;
  delete autoTurret;
  delete autoShooter;
  delete autoVisionSystem;
  delete autoIntake;
}

void Auto::SetupPlayback() {
  // Put in Robot::AutonomousInit
  // Get driver station info and setup
  inputFileName =
      frc::SmartDashboard::GetString(stationText, defaultFileName);
  debugCons("Reading auto instructions from /home/lvuser/" + inputFileName + ".aut\n");
  std::string filePath = "/home/lvuser/" + inputFileName + ".aut";
  cmdFile.Open(filePath, READ);
}

void Auto::ReadFile(cmd * inputs) {
  if (!autoDone) {
    debugCons("Reading auto file...\n");

    // Read controller inputs
    cmdFile.Read(inputs, sizeof(*inputs));
  }
}

void Auto::SetupRecording() {
  // Put in Robot::TestInit()
  inputFileName =
      frc::SmartDashboard::GetString(stationText, defaultFileName);
  debugCons("Writing instructions to /home/lvuser/" + inputFileName + ".aut\n");
  std::string filePath = "/home/lvuser/" + inputFileName + ".aut";
  cmdFile.Open(filePath, WRITE);
}

void Auto::Record(cmd * inputs) {
  // Put in Robot::TestPeriodic()
  debugCons("Writing auto file...\n");

  // Write controller inputs
  cmdFile.Write(inputs, sizeof(*inputs));
}

void Auto::CloseFile() {
  // Put in Robot::DisabledInit()
  debugCons("File closed.\n");
  cmdFile.Close();
}

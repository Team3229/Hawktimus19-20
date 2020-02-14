// 2020 test for capture/replay autonomous system

#include "Auto.h"

Auto::Auto(DriveSystem *c, Pneumatics *a, Lift *l, Limelight *v, Intake *i) {
  // Passes in subsystems from Robot
  autoChassis = c;
  autoAir = a;
  autoLift = l;
  autoVisionSystem = v;
  autoIntake = i;

  // Setup SmartDashboard
  frc::SmartDashboard::PutString(driverStationText, defaultFileName);
}

Auto::~Auto() {
  delete autoChassis;
  delete autoAir;
  delete autoLift;
  delete autoVisionSystem;
  delete autoIntake;
  delete autocommand;
}

void Auto::SetupPlayback() {
  // Put in Robot::AutonomousInit
  // Get driver station info and setup
  inputFileName =
      frc::SmartDashboard::GetString(driverStationText, defaultFileName);
  debug("Reading auto instructions from /home/lvuser/" + inputFileName + "\n");
  std::string filePath = "/home/lvuser/" + inputFileName;
  cmdFile.Open(filePath, READ);
}

void Auto::ReadFile() {
  debug("Reading auto file...\n");

  // Read controller inputs
  debug("Size of struct: " << sizeof(*autocommand) << "\n");
  cmdFile.Read(autocommand, sizeof(*autocommand));
}

void Auto::SetupRecording() {
  // Put in Robot::TestInit()
  inputFileName =
      frc::SmartDashboard::GetString(driverStationText, defaultFileName);
  debug("Writing instructions to /home/lvuser/" + inputFileName + "\n");
  std::string filePath = "/home/lvuser/" + inputFileName;
  cmdFile.Open(filePath, WRITE);
}

void Auto::Record() {
  // Put in Robot::TestPeriodic()
  debug("Writing auto file...\n");

  // Write controller inputs
  cmdFile.Write(autocommand, sizeof(*autocommand));
  debug("Driver 1 left stick Y: " << autocommand->xbox1_leftY << "\n");
}

void Auto::CloseFile() {
  // Put in Robot::DisabledInit()
  debug("File closed.\n");
  cmdFile.Close();
}

void Auto::AutoPeriodic() {
  // Put in Robot::AutonomousPeriodic
  if (!autoDone) {
    ReadFile();

    // TELEOP GOES HERE
    // REPLACE CONTROLLER get()s WITH STRUCT DATA
    // EXAMPLE: 
    // if (autocommand->xbox1_RightBumper) {
    //   autoAir->MoveFrontClimb();
    // }


}

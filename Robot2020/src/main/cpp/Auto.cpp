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
  frc::SmartDashboard::PutString(driverStationText, defaultFileName);
}

Auto::~Auto() {
  delete autoChassis;
  delete autoTurret;
  delete autoShooter;
  delete autoVisionSystem;
  delete autoIntake;
  delete autocommand;
}

void Auto::SetupPlayback() {
  // Put in Robot::AutonomousInit
  // Get driver station info and setup
  inputFileName =
      frc::SmartDashboard::GetString(driverStationText, defaultFileName);
  debugCons("Reading auto instructions from /home/lvuser/" + inputFileName + "\n");
  std::string filePath = "/home/lvuser/" + inputFileName;
  cmdFile.Open(filePath, READ);
}

void Auto::ReadFile() {
  debugCons("Reading auto file...\n");

  // Read controller inputs
  debugCons("Size of struct: " << sizeof(*autocommand) << "\n");
  cmdFile.Read(autocommand, sizeof(*autocommand));
}

void Auto::SetupRecording() {
  // Put in Robot::TestInit()
  inputFileName =
      frc::SmartDashboard::GetString(driverStationText, defaultFileName);
  debugCons("Writing instructions to /home/lvuser/" + inputFileName + "\n");
  std::string filePath = "/home/lvuser/" + inputFileName;
  cmdFile.Open(filePath, WRITE);
}

void Auto::Record() {
  // Put in Robot::TestPeriodic()
  debugCons("Writing auto file...\n");

  // Write controller inputs
  cmdFile.Write(autocommand, sizeof(*autocommand));
  debugCons("Driver 1 left stick Y: " << autocommand->xbox1_leftY << "\n");
}

void Auto::CloseFile() {
  // Put in Robot::DisabledInit()
  debugCons("File closed.\n");
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
    if(std::abs(autocommand->xbox1_leftX) < .1 && std::abs(autocommand->xbox1_rightY) < .1)
      autoChassis->StopMotor();
    else
    {
      double rotationOffset = 1+std::abs(autocommand->xbox1_leftX);
      autoChassis->Drive(autocommand->xbox1_rightY * autoChassis->kMaxSpeed
                        ,autocommand->xbox1_leftX * autoChassis->kMaxAngularSpeed*rotationOffset);
    }
  /*
    if(autocommand->xbox2_XButton)
      autoIntake->extendIntake();
    if(autocommand->xbox2_YButton)
      autoIntake->retractIntake();
      */
    (autocommand->xbox2_AButton) ? (autoIntake->reverseIntake())
    : (autoIntake->runIntake());  

    
  }


}

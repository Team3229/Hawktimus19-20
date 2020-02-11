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
  frc::SmartDashboard::PutString("File path: /home/lvuser/", defaultFileName);
}

Auto::~Auto() {
  delete autoChassis;
  delete autoAir;
  delete autoLift;
  delete autoVisionSystem;
  delete autoIntake;
  delete autocommand;
}

void Auto::SetupAuto() {
  // Get driver station info and setup
  inputFileName = frc::SmartDashboard::GetString("File path: /home/lvuser/", defaultFileName);
  SetupReading();
}

void Auto::SetupReading() {
  // fileName = (FILE_DIR + inputFileName).c_str();
  debug("Reading auto instructions from /home/lvuser/" + inputFileName + "\n");
  cmdFile.Open(inputFileName, READ);
}

void Auto::ReadFile() {
  debug("Reading auto file...\n");

  // Read controller inputs
  debug("Size of struct: " << sizeof(*autocommand) << "\n");
  cmdFile.Read(autocommand, sizeof(*autocommand));
}

void Auto::SetupRecording() {
  // fileName = (FILE_DIR + inputFileName).c_str();
  debug("Writing instructions to /home/lvuser/" + inputFileName + "\n");
  cmdFile.Open(inputFileName, WRITE);
}

void Auto::Record() {
  debug("Writing auto file...\n");

  // Write controller inputs
  cmdFile.Write(autocommand, sizeof(*autocommand));
  debug("Driver 1 left stick Y: " << autocommand->xbox1_leftY << "\n");
}

void Auto::CloseFile() {
  debug("File closed.\n");
  cmdFile.Close();
}

void Auto::AutoPeriodic() {
  if (!autoDone) {
    ReadFile();

    // Update controller axis values
    d1_leftY = autocommand->xbox1_leftY;
    d1_leftX = autocommand->xbox1_leftX;
    d1_rightX = autocommand->xbox1_rightX;

    d2_leftY = autocommand->xbox2_leftY;
    d2_rightY = autocommand->xbox2_rightY;

    // DRIVE
    debug("Gyro angle: " << autoChassis->TestGyro() << "\n");
    if (abs(d1_leftX) > DEAD_BAND || abs(d1_leftY) > DEAD_BAND ||
        abs(d1_rightX) > DEAD_BAND) {
      if (m_driveWithGyro == true) {
      }
      // drives robot with mecanum chassis + gyro
      autoChassis->Drive(d1_leftY, d1_leftX, d1_rightX);
    } else {
      // drives mecanum without gyro
      autoChassis->DriveWithoutGyro(d1_leftY, d1_leftX, d1_rightX);
    }
  } else {
    if (m_usingVision == false)
      autoChassis->Stop(); // stops driving
  }

  // swap robot and field orient with button
  if (autocommand->xbox1_RightTriggerAxis > DEAD_BAND)
    SwitchDriveMode();

  // speed changer
  // BOTH CONTROLLERS NOW HAVE ACCESS TO THESE
  if (autocommand->xbox1_AButton) {
    autoChassis->ChangeSpeed(2); // normal speed
    m_lastUsedSpeed = 2;
  }

  if (autocommand->xbox1_BButton) {
    autoChassis->ChangeSpeed(1); // slow speed
    m_lastUsedSpeed = 1;
  }

  if (autocommand->xbox1_XButton) {
    autoChassis->ChangeSpeed(3); // fast
    m_lastUsedSpeed = 3;
  }

  // PNEUMATICS
  // climber
  autoAir->ControlComp();
  if (autocommand->xbox1_RightBumper)
    autoAir->MoveFrontClimb(); // toggle front climbing poles

  if (autocommand->xbox1_LeftBumper)
    autoAir->MoveBackClimb(); // toggle back climbing poles

  // INTAKE OPERATION
  // wheels
  if (autocommand->xbox2_LeftTriggerAxis > DEAD_BAND)
    autoIntake->RunWheels(true); // wheels in
  else if (autocommand->xbox2_RightTriggerAxis > DEAD_BAND)
    autoIntake->RunWheels(false); // wheels out
  else
    autoIntake->StopWheels();

  // pivoting the intake
  if (abs(d2_leftY) > DEAD_BAND) {
    if (d2_leftY < 0)
      autoIntake->MoveIntake(true); // pivot intake up
    else
      autoIntake->MoveIntake(false); // pivot intake down
  } else
    autoIntake->StopIntakePivot(); // holds intake in place

  // LIMELIGHT VISION TRACKING AND GYRO ALIGNMENT
  // robot will stop moving when target is in desired range/orientation
  autoVisionSystem->GetValues();
  if (autocommand->xbox1_YButton) {
    m_usingVision = true;
    autoChassis->ChangeSpeed(2); // normal
    autoChassis->DetermineTarget(m_template);
    if (autoChassis->CanTurn())
      autoChassis->TurnToTarget();
    else
      autoVisionSystem->SeekTarget();
  } else {
    m_usingVision = false;
    autoChassis->ChangeSpeed(m_lastUsedSpeed);
  }

  // LIFT OPERATION
  if (abs(d2_rightY) > DEAD_BAND && m_lockLift == false) {
    if (d2_rightY < 0)
      autoLift->MoveLift(true); // moves lift up
    else
      autoLift->MoveLift(false); // moves lift down
  } else
    autoLift->StopLift(); // holds lift in place

  // toggle angle mode (Rocket Hatch vs. other)
  if (autocommand->xbox2_XButton) {
    if (m_template == "Other")
      m_template = "Rocket Hatch";
    else
      m_template = "Other";
    frc::SmartDashboard::PutString("Current Template", m_template);
    frc::Wait(0.25);
  }
}

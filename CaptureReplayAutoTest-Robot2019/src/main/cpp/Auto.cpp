// 2020 test for capture/replay autonomous system

#include "Auto.h"

Auto::Auto(DriveSystem * c, Pneumatics * a, Lift * l, Limelight * v, Intake * i) {
    // Passes in subsystems from Robot
    autoChassis = c;
    autoAir = a;
    autoLift = l;
    autoVisionSystem = v;
    autoIntake = i;
}

Auto::~Auto() {
    delete autoChassis;
    delete autoAir;
    delete autoLift;
    delete autoVisionSystem;
    delete autoIntake;
    delete autocommand;
    delete cmdFile;
    delete fileName;
}

void Auto::SetupReading() {
    fileName = (FILE_DIR + inputFileName).c_str();
    debug("Reading auto instructions from " + (FILE_DIR + inputFileName) + "\n");
    cmdFile = fopen(fileName, "rb");
}

void Auto::SetupWriting() {
    fileName = (FILE_DIR + inputFileName).c_str();
    debug("Writing instructions to " + (FILE_DIR + inputFileName) + "\n");
    cmdFile = fopen(fileName, "wb");
}

void Auto::WriteFile() {
    debug("Writing auto file...\n");

    // Write controller inputs
    fwrite((char*) autocommand, sizeof(char), sizeof(cmd), cmdFile);
    debug("Driver 1 left stick Y: " << autocommand->xbox1_leftY << "\n");
}

void Auto::ReadFile() {
    debug("Reading auto file...\n");

    // Read controller inputs
    fread((char*) autocommand, sizeof(char), sizeof(cmd), cmdFile);
}

void Auto::CloseFile() {
    fclose(cmdFile);
    debug("File closed.\n");
}

void Auto::AutoPeriodic() {   
    while (!autoDone) {
        //Update controller axis values
        d1_leftY = autocommand->xbox1_leftY;
        d1_leftX = autocommand->xbox1_leftX;
        d1_rightX = autocommand->xbox1_rightX;

        d2_leftY = autocommand->xbox2_leftY;
        d2_rightY = autocommand->xbox2_rightY;

        // DRIVE
        debug("Gyro angle: " << autoChassis->TestGyro() << "\n");
        if(abs(d1_leftX) > DEAD_BAND || abs(d1_leftY) > DEAD_BAND || abs(d1_rightX) > DEAD_BAND ) {
            if (m_driveWithGyro == true)
                autoChassis->Drive(d1_leftY, d1_leftX, d1_rightX); // drives robot with mecanum chassis + gyro
            else
                autoChassis->DriveWithoutGyro(d1_leftY, d1_leftX, d1_rightX); // drives mecanum without gyro
        }
        else {
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
            }
        else 
            autoIntake->StopIntakePivot(); // holds intake in place


        // LIMELIGHT VISION TRACKING AND GYRO ALIGNMENT
        // robot will stop moving when target is in desired range/orientation
        autoVisionSystem->GetValues();
        if (autocommand->xbox1_YButton) {
            m_usingVision = true;
            autoChassis->ChangeSpeed(2); //normal
            autoChassis->DetermineTarget(m_template);
            if (autoChassis->CanTurn())
                autoChassis->TurnToTarget();
            else
                autoVisionSystem->SeekTarget(); 
        }
        else {
            m_usingVision = false;
            autoChassis->ChangeSpeed(m_lastUsedSpeed);
        }

        // LIFT OPERATION
        if (abs(d2_rightY) > DEAD_BAND && m_lockLift == false) {
        if (d2_rightY < 0)
            autoLift->MoveLift(true); // moves lift up
        else
            autoLift->MoveLift(false); // moves lift down
        }
        else 
            autoLift->StopLift(); // holds lift in place

        //toggle angle mode (Rocket Hatch vs. other)
        if (autocommand->xbox2_XButton) {
            if (m_template == "Other")
                m_template = "Rocket Hatch";
            else 
                m_template = "Other";  
            frc::SmartDashboard::PutString("Current Template", m_template);
            frc::Wait(0.25);
        }
    }
}
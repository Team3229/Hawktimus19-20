#ifndef INTAKE_H
#define INTAKE_H

#pragma once

#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Timer.h>

#include <rev/CANSparkMax.h>

class Intake
{
private:
    frc::Compressor * m_compressor;
    frc::DoubleSolenoid * m_intakeSolenoid;

    rev::CANSparkMax * m_intakeMotor;

    const int kIntakeMotorID = 10;
    const int kCompressorPCMID = 0;
    const int kForwardIntakeID = 1;
    const int kReverseIntakeID = 2;

    bool intakeExtended = false;
    /* data */
public:
    Intake(/* args */);
    ~Intake();
    void controlComp();

    void extendIntake();
    void retractIntake();

    void runIntake();
    void reverseIntake();

    void stopIntake();
};

#endif
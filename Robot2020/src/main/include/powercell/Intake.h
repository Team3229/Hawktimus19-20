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

    const int kIntakeMotorID = 9;
    
    const int kCompressorPCMID = 0;
    const int kForwardIntakeID = 2; //was 0
    const int kReverseIntakeID = 3; //was 1
    
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

    void forceRunIntake(double power);
    void stopIntake();
};

#endif
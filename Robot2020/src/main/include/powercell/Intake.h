#ifndef INTAKE_H
#define INTAKE_H

#pragma once

#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Timer.h>

#include <rev/CANSparkMax.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>

class Intake
{
private:
    frc::Compressor * m_compressor;
    frc::DoubleSolenoid * m_intakeSolenoid;

    //rev::CANSparkMax * m_intakeMotor;
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX * m_intakeMotor;

    const int kIntakeMotorID = 9;
    
    const int kCompressorPCMID = 0;
    const int kForwardIntakeID = 2; //was 0
    const int kReverseIntakeID = 3; //was 1
    
    bool intakeExtended = false;
    const double INTAKE_POWER_IN = -.7;
    const double INTAKE_POWER_OUT = .7;

public:
    Intake();
    ~Intake();

    void controlComp();
    void extendIntake();
    void retractIntake();

    void runIntake();
    void reverseIntake();

    void forceRunIntake();
    void stopIntake();
};

#endif
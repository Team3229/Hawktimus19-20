#ifndef SHOOTER_H
#define SHOOTER_H

#pragma once

#include <rev/CANSparkMax.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/PIDController.h>
#include <frc/Servo.h>

#include "Debug.h"

class Shooter
{
private:
    frc::Servo * m_hoodServo;
    const int kHoodServoID = 1; //PWM

    rev::CANSparkMax * m_flyWheelFront;
    rev::CANSparkMax * m_flyWheelBack;
    rev::CANSparkMax * m_feeder;
    const int kFrontFWID = 7;   //CAN
    const int kBackFWID = 8;
    const int kFeederID = 9;

    frc2::PIDController * m_flyWheelPID;
    const double kP = .2;
    const double kI = 0;
    const double kD = 0;

    const double kRPMErrRange = 200;
    const double kHoodError = .05;

    const double kHoodAngleRatio = 1;
    const double kDistRPMRatio = 1;
public:
    Shooter();
    ~Shooter();

    double calcRPM(units::inch_t dist);
    bool readyFeed(units::inch_t dist);
    bool adjustFWSpeed(double rpm);
    bool adjustHood(units::inch_t dist);
    
    void feedShooter();
    void stopFeed();
    
    void hoodTest(double y);
};




#endif
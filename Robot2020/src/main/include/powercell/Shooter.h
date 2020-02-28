#ifndef SHOOTER_H
#define SHOOTER_H

#pragma once

#include <rev/CANSparkMax.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/PIDController.h>
#include <frc/Servo.h>

#include "Debug.h"

class Shooter
{
private:
    frc::Servo * m_hoodServo;
    const int kHoodServoID = 0; //PWM

    rev::CANSparkMax * m_flyWheelFront;
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX * m_feeder;
    const int kFrontFWID = 8;   //CAN
    const int kFeederID = 11;

    frc2::PIDController * m_flyWheelPID;
    const double kP = .1;
    const double kI = 0;
    const double kD = 0;

    const double kRPMErrRange = 200;
    const double kHoodError = .05;

    const double kHoodAngleRatio = 1;
    const double kDistRPMRatio = 1;

    const float FEEDER_FORWARD_POWER = 1.0;
    const float FEEDER_REVERSE_POWER = -0.8;
    const float SHOOTER_POWER = 1.0;

    double m_lastHoodPos = .0, hoodSetPosDebug;
    double FWOutputDebug, FWSpeedDebug, FWSetRPMDebug, m_lastOutput;
public:
    Shooter();
    ~Shooter();

    double calcRPM(units::inch_t dist);
    double calcHoodPos(units::inch_t dist);
    bool readyFeed(units::inch_t dist);
    bool adjustFWSpeed(double rpm);
    bool adjustHood(double position);
    void incrementalHood(double incrementValue);
    void runShooter();

    void maintainState();
    void maintainHood();
    void stopShooter();
    
    void feedShooter();
    void reverseFeed();
    void stopFeed();
    
    void hoodTest(double y);
    void shooterTest(double pow);

    void shooterDash();
};

#endif
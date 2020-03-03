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
    //frc::Servo * m_hoodServo;
    //const int kHoodServoID = 0; //PWM

    rev::CANSparkMax * m_flyWheelFront;
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX * m_feeder;
    const int kFrontFWID = 8;   //CAN
    const int kFeederID = 11;

    //Flywheel PID
    frc2::PIDController * m_flyWheelPID;
    const double kP = .1;
    const double kI = 0;
    const double kD = 0;

    //Appropriate range for max shooter rpm and hood position
    //const double kMAX_HOOD_POS = .8;
    const double kMAX_FW_RPM = 5500;  //** NEO 550 (max 12000), NEO (max 5500)

    //Appropriate range for shooter and hood errors
    const double kRPM_ERR_RANGE = 200;
    //const double kHOOD_ERR_RANGE = .05;

    //Ratio for dist to rpm/position conversion
    //const double kHOOD_ANGLE_RATIO = 1;
    const double kDIST_RPM_RATIO = 1;

    //Motor powers
    const float FEEDER_FORWARD_POWER = 0.8;
    const float FEEDER_REVERSE_POWER = -0.6;
    const float SHOOTER_POWER = 1.0;

    //register last Hood Position
    //double m_lastHoodPos = .0;
    
    //debugging
    double FWOutputDebug, FWSpeedDebug, FWSetRPMDebug, m_lastOutput, hoodSetPosDebug;
public:
    Shooter();
    ~Shooter();

    double calcRPM(units::inch_t dist);
    bool readyFeed(units::inch_t dist);
    bool adjustFWSpeed(double rpm);

    void runShooter();

    void maintainState();
    void stopShooter();
    
    void feedShooter();
    void reverseFeed();
    void stopFeed();

    void shooterDash();

    //void maintainHood();
    //bool adjustHood(double position);
    //void incrementalHood(double incrementValue);
    //double calcHoodPos(units::inch_t dist);
};

#endif
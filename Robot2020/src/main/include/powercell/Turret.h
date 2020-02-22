#ifndef TURRET_H
#define TURRET_H

#pragma once

#include <ctre/Phoenix.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/AnalogEncoder.h>
#include <frc/AnalogInput.h>
#include "Debug.h"
class Turret
{
    public:
        Turret();
        ~Turret();

        bool VisionTurn(double tX);
        void Turn(double setPower);
        double GetAngle();

    private:        
        ctre::phoenix::motorcontrol::can::WPI_TalonSRX * m_turretMotor;
        frc2::PIDController * m_turretPID;
        frc::AnalogEncoder * m_turEncoder;

        const int kTurretEncoderID = 1;
        frc::AnalogInput turretEncID{kTurretEncoderID}; //Analog Input
        const int kTurretMotorID = 10;

        const double kMaxTurretPower = .5;
        const units::degree_t kMaxRange = 90_deg;
        const double kEncoderRatio = 73.17;
        const double kP = .15;
        const double kI = 0;
        const double kD = 0;

        const double kNominalTX = 3;
        /* music
        ctre::phoenix::motorcontrol::can::TalonFX * m_fx;
        ctre::phoenix::music::Orchestra * m_music;
        */
};
#endif
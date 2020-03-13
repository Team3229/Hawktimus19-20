#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#pragma once

#include <frc/controller/PIDController.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>

#include <wpi/math>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <rev/CANSparkMax.h>

#include <AHRS.h>

#include <frc/PowerDistributionPanel.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/drive/DifferentialDrive.h>

#include "Debug.h"

/**
 * Represents a differential drive style drivetrain.
 * See http://docs.wpilib.org/en/latest/docs/software/kinematics-and-odometry/index.html
 */
class Drivetrain {
 public:
  Drivetrain();
  ~Drivetrain();

  frc::Rotation2d GetAngle() const;
  frc::DifferentialDriveWheelSpeeds GetSpeeds() const;
  frc::Pose2d GetPose() const;
  frc::DifferentialDriveKinematics GetKinematics() const;

  void SetPose(frc::Pose2d pose);
  void SetSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds);
  void Drive(units::meters_per_second_t speed, units::radians_per_second_t rot);
  void UpdateOdometry();
  void StopMotor();

  void drivetrainDash();

  static constexpr units::meters_per_second_t kMaxSpeed = 8.0_mps;  //max speed it is allowed to calculate
  static constexpr units::radians_per_second_t kMaxAngularSpeed{wpi::math::pi*2};  // 1/2 rotation per second
  static constexpr units::meters_per_second_t kSlowMaxSpeed = 4.0_mps;
 private:
  static constexpr units::meter_t kTrackWidth = 27_in; //distance between left wheels to the right wheels
  static constexpr double kWheelRadius = units::meter_t(6_in/2).to<double>();  // 6 inch diameter
  static constexpr int kEncToWheel = 8.68; // _ encoder rotation = 1 wheel rotation

  double m_leftSetpoint, m_rightSetpoint;
  double m_angleSetpoint;
  double m_leftVelocity;
  double m_rightVelocity;
  const float MAX_POWER = 1.0;
  const float DRIVE_RAMP_TIME = 0.7;

  units::meter_t m_leftPosition,m_rightPosition;
  AHRS * navXGyro;

  rev::CANSparkMax * m_leftUpper;  
  rev::CANSparkMax * m_leftFront;  
  rev::CANSparkMax * m_leftBack;   
  rev::CANSparkMax * m_rightUpper; 
  rev::CANSparkMax * m_rightFront; 
  rev::CANSparkMax * m_rightBack;  
  
  //PID Controllers, only kP should be necessary, lower it reduces accleration
  frc2::PIDController m_leftPIDController{.15, 0.0, 0.0};
  frc2::PIDController m_rightPIDController{.15, 0.0, 0.0};

  frc::DifferentialDriveKinematics m_kinematics{kTrackWidth};
  frc::DifferentialDriveOdometry m_odometry{frc::Rotation2d{0_deg}};
};

#endif 
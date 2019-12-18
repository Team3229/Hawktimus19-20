
#pragma once

#include <units/units.h>
#include <iostream>

#include <frc/SpeedControllerGroup.h>
#include <frc/controller/PIDController.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <wpi/math>

#include <rev/CANSparkMax.h>
#include <rev/CANEncoder.h>

/**
 * Represents a differential drive style drivetrain.
 */
class Drivetrain {
 public:

  Drivetrain();

  ~Drivetrain(); 

  /**
   * Get the robot angle as a Rotation2d.
   */
  // No Gyro yet
  // frc::Rotation2d GetAngle() const {
  //   // Negating the angle because WPILib Gyros are CW positive.
  //   return frc::Rotation2d(units::degree_t(-m_gyro.GetAngle()));
  // }

  static constexpr units::meters_per_second_t kMaxSpeed = 12.0_mps;  // 3 meters per second
  static constexpr units::radians_per_second_t kMaxAngularSpeed{wpi::math::pi};  // 1/2 rotation per second
  bool firstRunPos = true;

  frc::DifferentialDriveWheelSpeeds GetSpeeds() const;
  
  void SetSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds);
  
  void Drive(units::meters_per_second_t xSpeed, units::radians_per_second_t rot);
  //void RatioDrive(units::meters_per_second_t xSpeed, units::radians_per_second_t rot);
  void DriveDist(double dist,bool firstRun);
  //needs Gyro to work
  //void UpdateOdometry();

 private:

  const int LEFT_UP_ID = 1;
  const int LEFT_FR_ID = 2;
  const int LEFT_BA_ID = 3;
  const int RIGHT_UP_ID = 4;
  const int RIGHT_FR_ID = 5;
  const int RIGHT_BA_ID = 6;


  static constexpr units::meter_t kTrackWidth = 0.7092_m;  //dist between left and right wheels
  static constexpr double kWheelRadius = 0.0762;  // meters, .0508 m = 2 in
  //static constexpr int kEncoderResolution = 42;
  static constexpr double kEncToWheel = 8.68;        // # rotation on enc = 1 rotation on wheel
  static constexpr double kP = .3, kI = 0, kD = 0;
  //static constexpr double kRatioDrive = .05;
  
  double m_leftVelocity, m_rightVelocity,
         m_leftRot, m_rightRot;
  double m_setLeftRot, m_setRightRot;

  rev::CANSparkMax * leftUpper;  
  rev::CANSparkMax * leftFront;  
  rev::CANSparkMax * leftBack;   
  rev::CANSparkMax * rightUpper; 
  rev::CANSparkMax * rightFront; 
  rev::CANSparkMax * rightBack;  

  //rev::CANEncoder * leftEnc; 
  //rev::CANEncoder * rightEnc;

  frc2::PIDController m_leftPIDController{1.0, 0.0, 0.0};
  frc2::PIDController m_rightPIDController{1.0, 0.0, 0.0};

  rev::CANPIDController * m_leftPosPIDController;
  rev::CANPIDController * m_rightPosPIDController;


  frc::DifferentialDriveKinematics m_kinematics{kTrackWidth};
  //frc::DifferentialDriveOdometry m_odometry{m_kinematics};
};
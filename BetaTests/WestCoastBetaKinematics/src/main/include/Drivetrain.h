
#pragma once

#include <units/units.h>
#include <iostream>

#include <frc/controller/PIDController.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <wpi/math>

#include <rev/CANSparkMax.h>

/**
 * Represents a differential drive style drivetrain.
 */
class Drivetrain {
 public:

  Drivetrain();
  ~Drivetrain(); 

  static constexpr units::meters_per_second_t kMaxSpeed = 6_mps;  //the max speed (* the controller input [-1,1])
  static constexpr units::radians_per_second_t kMaxAngularSpeed{wpi::math::pi};  // 1/2 rotation per second

  frc::DifferentialDriveWheelSpeeds GetSpeeds() const;
  void SetSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds);
  void Drive(units::meters_per_second_t xSpeed, units::radians_per_second_t rot);
  void SetEncoder(double rot);

 private:

  const int LEFT_UP_ID = 1;
  const int LEFT_FR_ID = 2;
  const int LEFT_BA_ID = 3;
  const int RIGHT_UP_ID = 4;
  const int RIGHT_FR_ID = 5;
  const int RIGHT_BA_ID = 6;


  static constexpr units::meter_t kTrackWidth = 0.7092_m;  //dist between left and right wheels
  static constexpr double kWheelRadius = 0.0762;           //meters
  static constexpr double kEncToWheel = 8.68;              // # rotation on enc = 1 rotation on wheel
  static constexpr double kP = .3, kI = 0, kD = 0;
  
  double m_leftVelocity, m_rightVelocity;

  rev::CANSparkMax * leftUpper;  
  rev::CANSparkMax * leftFront;  
  rev::CANSparkMax * leftBack;   
  rev::CANSparkMax * rightUpper; 
  rev::CANSparkMax * rightFront; 
  rev::CANSparkMax * rightBack;  

  frc2::PIDController m_leftPIDController{1.0, 0.0, 0.0};
  frc2::PIDController m_rightPIDController{1.0, 0.0, 0.0};

  rev::CANPIDController * m_leftRevPIDController;
  rev::CANPIDController * m_rightRevPIDController;


  frc::DifferentialDriveKinematics m_kinematics{kTrackWidth};
};
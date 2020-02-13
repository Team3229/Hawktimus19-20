
#include "Drivetrain.h"

#include <math.h>

Drivetrain::Drivetrain()
{
  /*Motors, West Coast Drive
               Upper
     |front      O      |back
     |robot    O   O    |robot
          Front    Back
  */
  leftUpper   =   new rev::CANSparkMax(LEFT_UP_ID,rev::CANSparkMax::MotorType::kBrushless);
  leftFront   =   new rev::CANSparkMax(LEFT_FR_ID,rev::CANSparkMax::MotorType::kBrushless);
  leftBack    =   new rev::CANSparkMax(LEFT_BA_ID,rev::CANSparkMax::MotorType::kBrushless);
  rightUpper  =   new rev::CANSparkMax(RIGHT_UP_ID,rev::CANSparkMax::MotorType::kBrushless);
  rightFront  =   new rev::CANSparkMax(RIGHT_FR_ID,rev::CANSparkMax::MotorType::kBrushless);
  rightBack   =   new rev::CANSparkMax(RIGHT_BA_ID,rev::CANSparkMax::MotorType::kBrushless);

  //left follower
  leftBack->Follow(*leftUpper);
  leftFront->Follow(*leftUpper);
  //right follower
  rightBack->Follow(*rightUpper);
  rightFront->Follow(*rightUpper);
  
  
  //REV-Robotics PID Controllers 
  m_leftRevPIDController = new rev::CANPIDController(leftUpper->GetPIDController());
  m_rightRevPIDController = new rev::CANPIDController(rightUpper->GetPIDController());
  m_leftRevPIDController->SetP(kP);
  m_rightRevPIDController->SetP(kP);
  m_rightRevPIDController->SetSmartMotionAllowedClosedLoopError(10);
}



Drivetrain::~Drivetrain()
{
  
  delete leftUpper;
  delete leftFront;
  delete leftBack;

  delete rightUpper;
  delete rightFront;
  delete rightBack;

  delete m_leftRevPIDController;
  delete m_rightRevPIDController;
}



frc::DifferentialDriveWheelSpeeds Drivetrain::GetSpeeds() const 
{
  return {units::meters_per_second_t(m_leftVelocity),
          units::meters_per_second_t(m_rightVelocity)};
}


void Drivetrain::SetSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds)                               //velocity PID controller, frc library
{
  //velocity formula, v = RPM/60 * WheelCircum / MotorToWheelRatio
  m_leftVelocity = (leftUpper->GetEncoder().GetVelocity() / kEncToWheel) * 2 * wpi::math::pi* kWheelRadius / 60;    //max vel should be 5.218 m/s or 17.12 ft/s
  m_rightVelocity = (rightUpper->GetEncoder().GetVelocity() / kEncToWheel) * 2 * wpi::math::pi * kWheelRadius / 60; //NEO Motor free speed at 5676 RPM
  
  std::cout << "left v: " << m_leftVelocity << std::endl;
  const auto leftOutput = m_leftPIDController.Calculate(m_leftVelocity, speeds.left.to<double>());        //I assume it takes in velocity since the setpoint is in velocity
  const auto rightOutput = m_rightPIDController.Calculate(m_rightVelocity, speeds.right.to<double>());    
  std::cout << "left set speed: " << speeds.left << std::endl;
  std::cout << "left output: " << leftOutput << std::endl;
  
  rightUpper->Set(rightOutput);
  leftUpper->Set(leftOutput);
}

void Drivetrain::Drive(units::meters_per_second_t xSpeed, units::radians_per_second_t rot) 
{
  std::cout << "xSpeed" << xSpeed << std::endl;
  SetSpeeds(m_kinematics.ToWheelSpeeds({xSpeed, 0_mps, rot}));  //+x is actually forward
}

void Drivetrain::SetEncoder(double rot)
{
  m_rightRevPIDController->SetReference(rot,rev::kPosition);
  leftUpper->GetEncoder().SetPosition(rot);
}
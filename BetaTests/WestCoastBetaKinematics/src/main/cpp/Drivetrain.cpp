
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

  leftUpper->RestoreFactoryDefaults(); 
  leftFront->RestoreFactoryDefaults();
  leftBack->RestoreFactoryDefaults();
  rightUpper->RestoreFactoryDefaults();
  rightFront->RestoreFactoryDefaults();
  rightBack ->RestoreFactoryDefaults();

  //time to go from 0 to full throttle
  leftUpper->SetClosedLoopRampRate(1);
  leftFront->SetClosedLoopRampRate(1);
  leftBack->SetClosedLoopRampRate(1);
  rightUpper->SetClosedLoopRampRate(1);
  rightFront->SetClosedLoopRampRate(1);
  rightBack->SetClosedLoopRampRate(1);

  //left follower
  leftBack->Follow(*leftUpper);
  leftFront->Follow(*leftUpper);
  //right follower
  rightBack->Follow(*rightUpper);
  rightFront->Follow(*rightUpper);
  
  //calculate velocity, v = rw, w = motor velocity / enctowheel, motorvelocity gives RPM
  
  //m_leftVelocity = (leftUpper->GetEncoder().GetVelocity() / kEncToWheel) * kWheelRadius / 60;
  //m_rightVelocity = (rightUpper->GetEncoder().GetVelocity() / kEncToWheel) * kWheelRadius / 60;
  
  //REV-Robotics PID Controllers 
  m_leftPosPIDController = new rev::CANPIDController(leftUpper->GetPIDController());
  m_rightPosPIDController = new rev::CANPIDController(rightUpper->GetPIDController());
  m_leftPosPIDController->SetP(kP);
  m_rightPosPIDController->SetP(kP);
}



Drivetrain::~Drivetrain()
{
  
  delete leftUpper;
  delete leftFront;
  delete leftBack;

  delete rightUpper;
  delete rightFront;
  delete rightBack;

  delete m_leftPosPIDController;
  delete m_rightPosPIDController;
}



frc::DifferentialDriveWheelSpeeds Drivetrain::GetSpeeds() const 
{
  return {units::meters_per_second_t(m_leftVelocity),
          units::meters_per_second_t(m_rightVelocity)};
}


void Drivetrain::SetSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds)                               //velocity PID controller, frc library
{
  m_leftVelocity = (leftUpper->GetEncoder().GetVelocity() / kEncToWheel) * kWheelRadius / 60;
  m_rightVelocity = (rightUpper->GetEncoder().GetVelocity() / kEncToWheel) * kWheelRadius / 60;
  
  const auto leftOutput = m_leftPIDController.Calculate(m_leftVelocity, speeds.left.to<double>());        //I assume it takes in velocity since the setpoint is in velocity
  const auto rightOutput = m_rightPIDController.Calculate(m_rightVelocity, speeds.right.to<double>());    

  rightUpper->Set(rightOutput);
  leftUpper->Set(leftOutput);

}

void Drivetrain::Drive(units::meters_per_second_t xSpeed, units::radians_per_second_t rot) 
{
  //std::cout << "xSpeed" << xSpeed << std::endl;
  SetSpeeds(m_kinematics.ToWheelSpeeds({xSpeed, 0_mps, rot}));  //+x is actually forward
}

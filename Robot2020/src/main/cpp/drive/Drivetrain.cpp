
#include "drive/Drivetrain.h"

Drivetrain::Drivetrain()
{
  m_leftUpper = new rev::CANSparkMax(4,rev::CANSparkMax::MotorType::kBrushless);
  m_leftFront = new rev::CANSparkMax(6,rev::CANSparkMax::MotorType::kBrushless);
  m_leftBack = new rev::CANSparkMax(5,rev::CANSparkMax::MotorType::kBrushless);
  m_rightUpper = new rev::CANSparkMax(1,rev::CANSparkMax::MotorType::kBrushless);
  m_rightFront = new rev::CANSparkMax(3,rev::CANSparkMax::MotorType::kBrushless);
  m_rightBack = new rev::CANSparkMax(2,rev::CANSparkMax::MotorType::kBrushless);
  
  navXGyro = new AHRS(frc::SPI::Port::kMXP);
  
  m_leftUpper->RestoreFactoryDefaults();
  m_leftFront->RestoreFactoryDefaults();
  m_leftBack->RestoreFactoryDefaults();
  m_rightUpper->RestoreFactoryDefaults();
  m_rightBack->RestoreFactoryDefaults();
  m_rightFront->RestoreFactoryDefaults();
  m_leftUpper->SetClosedLoopRampRate(1);
  m_rightUpper->SetClosedLoopRampRate(1);

  m_leftBack->Follow(*m_leftUpper);
  m_leftFront->Follow(*m_leftUpper);

  m_rightBack->Follow(*m_rightUpper);
  m_rightFront->Follow(*m_rightUpper);

  //m_leftUpper->GetEncoder().SetVelocityConversionFactor((1/ kEncToWheel) *2*wpi::math::pi* kWheelRadius / 60);
  //m_rightUpper->GetEncoder().SetVelocityConversionFactor((1/ kEncToWheel) *2*wpi::math::pi* kWheelRadius / 60);
  //m_leftUpper->GetEncoder().SetPositionConversionFactor();
  //m_rightUpper->GetEncoder().SetPositionConversionFactor();
}

Drivetrain::~Drivetrain()
{
  delete m_leftUpper;
  delete m_leftFront;
  delete m_leftBack;
  delete m_rightUpper;
  delete m_rightFront;
  delete m_rightBack;

  delete navXGyro;
}

//returns angle in radians, updates the robot position
frc::Rotation2d Drivetrain::GetAngle() const {
  return frc::Rotation2d(units::degree_t(-navXGyro->GetYaw()));
}

//return m_left and m_right wheel speeds, for setSpeeds() in Drive()
frc::DifferentialDriveWheelSpeeds Drivetrain::GetSpeeds() const {
  return {units::meters_per_second_t(m_leftVelocity),
          units::meters_per_second_t(m_rightVelocity)};
}

//return Pose2d (for Ramsete controller)
frc::Pose2d Drivetrain::GetPose() const
{
  frc::Pose2d currentPos = m_odometry.GetPose();
  //tests
  units::meter_t currentX = currentPos.Translation().X();
  units::meter_t currentY = currentPos.Translation().Y();
  units::degree_t currentDeg = currentPos.Rotation().Degrees();
  debugDashNum("Pose x (m)", currentX.to<double>());
  debugDashNum("Pose y (m)", currentY.to<double>());
  debugDashNum("Pose angle (degree)", currentDeg.to<double>());
  
  return currentPos;
}

frc::DifferentialDriveKinematics Drivetrain::GetKinematics() const
{
  return m_kinematics;
}

/**
 * sets the robot position on field, use at beginning of the match
 * @param pose, position
 */ 
void Drivetrain::SetPose(frc::Pose2d pose)
{
  m_odometry.ResetPosition(pose,GetAngle());
}

/**
 * setting the speed of left and right side to the motors
 * @param speeds, wheel speeds of diff drivetrain
 */ 
void Drivetrain::SetSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds) {
  m_leftVelocity = (m_leftUpper->GetEncoder().GetVelocity() / kEncToWheel) *2*wpi::math::pi* kWheelRadius / 60;
  m_rightVelocity = -(m_rightUpper->GetEncoder().GetVelocity() / kEncToWheel) *2*wpi::math::pi* kWheelRadius / 60;
  
  debugDashNum("leftVelocity",m_leftVelocity);
  debugDashNum("rightVelocity",m_rightVelocity);
  
  const auto leftOutput = m_leftPIDController.Calculate(
      m_leftVelocity, speeds.left.to<double>());
  const auto rightOutput = -m_rightPIDController.Calculate(
      m_rightVelocity, speeds.right.to<double>());

  m_rightUpper->Set(rightOutput);
  m_leftUpper->Set(leftOutput);
}
/**
 * main drive 
 * @param speed forward speed
 * @param rot angular velocity
 */
void Drivetrain::Drive(units::meters_per_second_t speed, units::radians_per_second_t rot) 
{
  SetSpeeds(m_kinematics.ToWheelSpeeds({speed, 0_mps, rot}));
}

/**
 * update robot's position on the field
 */ 
void Drivetrain::UpdateOdometry() 
{

  //1 enc report = 1 full motor rotation
  m_leftPosition = units::meter_t(m_leftUpper->GetEncoder().GetPosition() * (2*wpi::math::pi*kWheelRadius)/kEncToWheel);
  m_rightPosition = units::meter_t(m_leftUpper->GetEncoder().GetPosition() * (2*wpi::math::pi*kWheelRadius)/kEncToWheel);

  debugDashNum("Left Position out",m_leftPosition.to<double>());
  debugDashNum("Right Position out",m_rightPosition.to<double>());

  m_odometry.Update(GetAngle(), m_leftPosition,m_rightPosition);
}

/**
 * stops drivetrain
 */ 
void Drivetrain::StopMotor()
{
  m_rightUpper->StopMotor();
  m_leftUpper->StopMotor();
}
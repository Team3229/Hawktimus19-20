#include "powercell/Shooter.h"

Shooter::Shooter()
{
    m_hoodServo = new frc::Servo(kHoodServoID);

    m_flyWheelFront = new rev::CANSparkMax(kFrontFWID,rev::CANSparkMax::MotorType::kBrushless);
    m_flyWheelBack  = new rev::CANSparkMax(kBackFWID,rev::CANSparkMax::MotorType::kBrushless);
    m_flyWheelPID = new frc2::PIDController(kP,kI,kD);

    m_feeder = new rev::CANSparkMax(kFeederID, rev::CANSparkMax::MotorType::kBrushless);

    m_hoodServo->SetBounds(2.0,1.8,1.5,1.2,1.0);

    m_flyWheelBack->RestoreFactoryDefaults();
    m_flyWheelFront->RestoreFactoryDefaults();
    m_feeder->RestoreFactoryDefaults();
    
    m_flyWheelBack->Follow(*m_flyWheelFront);
    //motor configurations
}

Shooter::~Shooter()
{
    delete m_flyWheelFront;
    delete m_flyWheelBack;
    delete m_flyWheelPID;
    delete m_feeder;
}
/*
calculate rpm with a distance input
*/
double Shooter::calcRPM(units::inch_t dist)
{
    //find ratio to rpm, minimum 6000 rpm
    double rpm = std::clamp(dist.to<double>() * kDistRPMRatio,5000.0,12000.0);
    debugDashNum("calculated RPM",rpm);
    return rpm;
}
/**
 * sets the fly wheel rpm 
 * returns true if it is in appropriate rpm
 */ 
bool Shooter::adjustFWSpeed(double rpm)
{
    //adjusting based on rpm
    double FWSpeed = -m_flyWheelFront->GetEncoder().GetVelocity();
    debugDashNum("current rpm",FWSpeed);
    double output = m_flyWheelPID->Calculate(FWSpeed,rpm);
    m_flyWheelFront->Set(-output);
    if (std::abs(FWSpeed - rpm) > kRPMErrRange)
    {
        debugDashNum("FWSpeed correct",0);
        return false;
    }
    debugDashNum("FWSpeed correct",1);
    return true;
}
/**
 * entire shooter mechanism
 * check conditions before feeding in balls
 */
bool Shooter::readyFeed(units::inch_t dist)
{
    //add a timer if this is too fast
    if (adjustFWSpeed(calcRPM(dist)) && adjustHood(dist))
        return true;
    return false;
}
/**
 * adjust hood angle based on ty from limelight
 * might need a calcPos() if it isn't linear relationship
 */
bool Shooter::adjustHood(units::inch_t dist)
{
    double correctPos = std::clamp(dist.to<double>()*kHoodAngleRatio,0.0,1.0);
    m_hoodServo->SetPosition(correctPos);
    debugDashNum("Hood Actuator",m_hoodServo->GetPosition());
    if (std::abs(m_hoodServo->GetPosition() - correctPos) > kHoodError)
    {
        debugDashNum("Hood correct",0);
        return false;
    }
    debugDashNum("Hood correct",1);
    return true;
}

//feeding shooter
void Shooter::feedShooter()
{
    m_feeder->Set(.3);
}
void Shooter::stopFeed()
{
    m_feeder->Set(-.1);
}
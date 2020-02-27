#include "powercell/Shooter.h"

Shooter::Shooter()
{
    m_hoodServo = new frc::Servo(kHoodServoID);

    m_flyWheelFront = new rev::CANSparkMax(kFrontFWID,rev::CANSparkMax::MotorType::kBrushless);
    m_flyWheelPID = new frc2::PIDController(kP,kI,kD);

    m_feeder = new ctre::phoenix::motorcontrol::can::WPI_TalonSRX(kFeederID);

    //m_hoodServo->SetBounds(2.0,1.8,1.5,1.2,1.0);
    // Test values with lower maximum:
    m_hoodServo->SetBounds(1.6,1.4,1.1,0.8,0.6);

    m_flyWheelFront->RestoreFactoryDefaults();

    m_feeder->ClearStickyFaults();
}

Shooter::~Shooter()
{
    delete m_flyWheelFront;
    delete m_flyWheelPID;
    delete m_feeder;
}
/*
calculate rpm with a distance input
*/
double Shooter::calcRPM(units::inch_t dist)
{
    double FWSetRPM = std::clamp(dist.to<double>() * kDistRPMRatio,0.0,5700.0); //** NEO 550 (max 12000), NEO (max 5700)
    debug(FWSetRPMDebug = FWSetRPM);
    return FWSetRPM;
}
double Shooter::calcHoodPos(units::inch_t dist)
{
    double hoodSetPos = dist.to<double>() * kHoodAngleRatio;
    debug(hoodSetPosDebug = hoodSetPos);
    return hoodSetPos;
}
/**
 * sets the fly wheel rpm 
 * returns true if it is in appropriate rpm
 */ 
bool Shooter::adjustFWSpeed(double rpm)
{
    //adjusting based on rpm
    double FWSpeed = m_flyWheelFront->GetEncoder().GetVelocity();
    double FWOutput = m_flyWheelPID->Calculate(FWSpeed,rpm);
    m_lastOutput = FWOutput;

    debug(FWOutputDebug = FWOutput); 
    debug(FWSpeedDebug = FWSpeed);
    
    m_flyWheelFront->Set(1);
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
    if (adjustFWSpeed(calcRPM(dist)) && adjustHood(calcHoodPos(dist)))
        return true;
    return false;
}
/**
 * adjust hood angle based on ty from limelight
 * might need a calcPos() if it isn't linear relationship
 */
bool Shooter::adjustHood(double pos)
{
    double correctPos = std::clamp(pos,0.0,0.8);
    m_lastHoodPos = correctPos;
    m_hoodServo->SetPosition(correctPos);

    if (std::abs(m_hoodServo->GetPosition() - correctPos) > kHoodError)
    {
        debugDashNum("Hood correct",0);
        return false;
    }
    debugDashNum("Hood correct",1);
    return true;
}
void Shooter::incrementalHood(double incrementalValue)
{
    m_lastHoodPos += incrementalValue;
    adjustHood(m_lastHoodPos);
}
/**
 * Maintain the state of last registered FW Speed and Hood position
 */
void Shooter::maintainState()
{
    m_flyWheelFront->Set(m_lastOutput);
    m_hoodServo->SetPosition(m_lastHoodPos);
}
//stop shooter
void Shooter::stopShooter()
{
    m_flyWheelFront->Set(0);
}

//feeder, feeding shooter
void Shooter::feedShooter()
{
    m_feeder->Set(1);
}
void Shooter::reverseFeed()
{
    m_feeder->Set(-.4);
}
void Shooter::stopFeed()
{
    m_feeder->Set(0);
}

void Shooter::shooterDash()
{
    debugDashNum("(S) Hood Last Position", m_lastHoodPos);
    debugDashNum("(S) Hood Current Position",m_hoodServo->GetPosition());
    debugDashNum("(S) FW output", FWOutputDebug);
    debugDashNum("(S) current FWrpm",FWSpeedDebug);
    debugDashNum("(S) calculated RPM",FWSetRPMDebug);
    debugDashNum("(S) calculated Hood Pos", hoodSetPosDebug);
}
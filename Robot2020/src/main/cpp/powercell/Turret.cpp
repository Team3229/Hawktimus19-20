#include "powercell/Turret.h"

Turret::Turret()
{
    /*
    m_fx = new ctre::phoenix::motorcontrol::can::TalonFX(3);
    m_music = new ctre::phoenix::music::Orchestra();
    m_music->AddInstrument(*m_fx);
    */
    m_turretMotor = new ctre::phoenix::motorcontrol::can::WPI_TalonSRX(kTurretMotorID);
    //m_turretEncoder = new frc::AnalogPotentiometer(kTurretEncoderID,kTurretRange,kZeroVoltRead);
    m_turretPID = new frc2::PIDController(kP,kI,kD);
    m_turEncoder = new frc::AnalogEncoder(turretEncID);
    m_turretPID->SetTolerance(.05);
    debugCons("\nEncoder Reset")    
    m_turEncoder->Reset();

    m_turretMotor->ClearStickyFaults();
}
Turret::~Turret()
{
    delete m_turretMotor;
    //delete m_turretEncoder;
    delete m_turretPID;
    delete m_turEncoder;
}
/*
Turn based on Limelight tx Value
turn until tx is within error range of [-.5,.5]
*/ 
bool Turret::VisionTurn(double tX)
{
    if (std::abs(tX)>kNominalTX)
    {
        lastOutput = m_turretPID->Calculate(tX,0);
        Turn(lastOutput);
        return false;
    }
    else
    {
        Turn(-lastOutput);
        return true;
    }
}
/*
turn with set motor power
*/
void Turret::Turn(double setPower)
{
    //ratio, angle:encoder is 73.17:1
    //GetDistance() start: 0, left 90: -1.23, right 90: 1.23
    //Note: around 180 degrees the encoder jumps from 2.x to -2.x (or opposite) 
    //and then original 0 becomes -1
    //Try not allow that condition to happen and if necessary, rotate the robot
    //turn with setpower if the encoder is within allowed range
    debugDashNum("(Tur) Turret Power", setPower);
    debugCons(std::abs(GetAngle()) << "\n");
    if (std::abs(GetAngle()) < 90.0)
    {
        debugDashNum("(Tur) larger than max range",0);
        m_turretMotor->Set(std::clamp(setPower,-.2,.2));
    }
    else
    {
        debugDashNum("(Tur) larger than max range",1);
        m_turretMotor->Set(std::clamp(-setPower,-.1,.1));//prevent locking of the turret
    }
}
/*
Return the angle of the turret
*/
double Turret::GetAngle()
{
    return m_turEncoder->GetDistance() * kEncoderRatio;
}

void Turret::turretDash()
{
    debugDashNum("(Tur) Turret Encoder",GetAngle());
}
#include "powercell/Limelight.h"

Limelight::Limelight(Turret * turyeet, Shooter * yeeter)
{
    table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("pipeline", 0);
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 1);
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("camMode", 0);
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("stream", 0);

    m_turret = turyeet;
    m_shooter = yeeter;
}

Limelight::~Limelight()
{
    delete m_turret;
    delete m_shooter;
}

units::inch_t Limelight::calcDist()
{
    double angle = table->GetNumber("ty",0.0)+cameraMount.to<double>();
    return heightDiff/std::tan(angle);
}
//press button
void Limelight::scoreOperation()
{
    (m_shooter->adjustFWSpeed(m_shooter->calcRPM(calcDist()))) ? (m_shooter->feedShooter())
    : (m_shooter->stopFeed());
}
//periodic
bool Limelight::aimOperation()
{
    if(table->GetNumber("tv",0) == 1)
    {
        m_shooter->adjustHood(m_shooter->calcHoodPos(calcDist()));
        if(calcDist().to<double>() != 0)
        {
            if(m_turret->VisionTurn(table->GetNumber("tx",0.0)) &&
                m_shooter->adjustHood(m_shooter->calcHoodPos(calcDist())))
            {
                return true;
            }
        }
    }
    return false;
}

void Limelight::scoreWithPOV(double povValue)
{
    if(povValue == 0 || povValue == -1)
    {
        (m_shooter->adjustFWSpeed(6000) && m_shooter->adjustHood(.4)) ? (m_shooter->feedShooter())
        : (m_shooter->stopFeed());
    }
    else if(povValue == 90)
    {
        (m_shooter->adjustFWSpeed(7000) && m_shooter->adjustHood(.4)) ? (m_shooter->feedShooter())
        : (m_shooter->stopFeed());
    }
}

void Limelight::scoreWithPOVManual(double povValue)
{
    if(povValue == 0 || povValue == -1)
    {
        (m_shooter->adjustFWSpeed(4000)) ? (m_shooter->feedShooter())
        : (m_shooter->stopFeed());
    }
    else if(povValue == 90)
    {
        (m_shooter->adjustFWSpeed(4500)) ? (m_shooter->feedShooter())
        : (m_shooter->stopFeed());
    }
    else if(povValue == 180)
    {
        (m_shooter->adjustFWSpeed(5000)) ? (m_shooter->feedShooter())
        : (m_shooter->stopFeed());
    }
    else if(povValue == 270)
    {
        (m_shooter->adjustFWSpeed(5500)) ? (m_shooter->feedShooter())
        : (m_shooter->stopFeed());
    }
    else
    {
        (m_shooter->adjustFWSpeed(3500)) ? (m_shooter->feedShooter())
        : (m_shooter->stopFeed());
    }   
}
void Limelight::limelightDash()
{
    debugDashNum("(L) tx",getTX());
    debugDashNum("(L) ty",getTY());
    debugDashNum("(L) distance",calcDist().to<double>());
}
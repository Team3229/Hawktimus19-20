#include "powercell/Limelight.h"

Limelight::Limelight(Turret * turyeet, Shooter * yeeter)
{
    table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("pipeline", 0);
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 3);
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("camMode", 0);

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
    (aimOperation()) ? (m_shooter->feedShooter())
    : (m_shooter->stopFeed());
}
//periodic
bool Limelight::aimOperation()
{
    if(m_turret->VisionTurn(table->GetNumber("tx",0.0)) &&
        m_shooter->adjustFWSpeed(m_shooter->calcRPM(calcDist())) &&
        m_shooter->adjustHood(calcDist()))
        return true;
    return false;
}
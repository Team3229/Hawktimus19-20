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

/**
 * Calculate distance from target
 * Formulas See https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
 * @return distance unit from target  
 */ 
units::inch_t Limelight::calcDist()
{
    double angle = table->GetNumber("ty",0.0)+cameraMount.to<double>();
    return heightDiff/std::tan(angle);
}

/**
 * Auto aim, run & check fly wheel speed
 * Scoring power cells
 */ 
void Limelight::scoreOperation()
{
    if (m_shooter->adjustFWSpeed(m_shooter->calcRPM(calcDist()))) {
        m_shooter->feedShooter();
    } else {
        m_shooter->stopFeed();
    }
}

/**
 * ! add function when no target detected
 * Auto aim, adjust hood and turret
 * @return true if in position
 */ 
bool Limelight::aimOperation()
{
    if(table->GetNumber("tv",0) == 1)
    {
        //remove adjust hood
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

/**
 * Remove when hood removed
 * 
 * Auto aim, adjust flywheel and hood position based on POV button values
 * uses preset flywheel speed and hood position, bypass turret check
 * @param povValue, POV button value
 */ 
void Limelight::scoreWithPOV(int povValue)
{
    switch(povValue)
    {
        case -1:
            if(m_shooter->adjustFWSpeed(6000) && m_shooter->adjustHood(.4)){
                m_shooter->feedShooter();
            }else{
                m_shooter->stopFeed();
            }
            break;
        case 0:
            if(m_shooter->adjustFWSpeed(6000) && m_shooter->adjustHood(.4)){
                m_shooter->feedShooter();
            }else{
                m_shooter->stopFeed();
            }
            break;
        case 90:
            if(m_shooter->adjustFWSpeed(7000) && m_shooter->adjustHood(.4)){
                m_shooter->feedShooter();
            }else{
                m_shooter->stopFeed();
            }
            break;
        default:
            scoreOperation();
            break;
    }
}

/**
 * Force a set rpm on shooter
 * Manual aim, adjust flywheel speed based on POV button values
 * uses preset flyWheel velocity, bypass turret and hood check
 * @param povValue, POV button value
 */ 
void Limelight::scoreWithPOVManual(int povValue)
{
    switch(povValue)
    {
        //depends what happen to progress on tuning/control
        //case -1:
        //    if(m_shooter->adjustFWSpeed(4500)){
        //        m_shooter->feedShooter();
        //    }else{
        //        m_shooter->stopFeed();
        //    }
        //    break;
        case 0:
            if(m_shooter->adjustFWSpeed(4500)){
                m_shooter->feedShooter();
            }else{
                m_shooter->stopFeed();
            }
            break;
        case 90:    //right trench
            if(m_shooter->adjustFWSpeed(5000)){
                m_shooter->feedShooter();
            }else{
                m_shooter->stopFeed();
            }
            break;
        case 180:   //back
            if(m_shooter->adjustFWSpeed(5500)){
                m_shooter->feedShooter();
            }else{
                m_shooter->stopFeed();
            }
            break;
        case 270:   //left side
            if(m_shooter->adjustFWSpeed(5000)){
                m_shooter->feedShooter();
            }else{
                m_shooter->stopFeed();
            }
            break;
        default:    //others
            scoreOperation();
            break;
    }
}
void Limelight::limelightDash()
{
    debugDashNum("(L) tx",getTX());
    debugDashNum("(L) ty",getTY());
    debugDashNum("(L) distance",calcDist().to<double>());
}
#ifndef LIMELIGHT_H
#define LIMELIGHT_H

#pragma once

#include "Turret.h"
#include "Shooter.h"

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

class Limelight
{
private:
    std::shared_ptr<NetworkTable> table;
    Turret * m_turret;
    Shooter * m_shooter;

    double distRatio = 1;

    const units::degree_t cameraMount = 25_deg;
    const units::inch_t mountHeight = 14_in;
    const units::inch_t heightDiff = 8_ft + 2.25_in - mountHeight;

    
public:
    Limelight(Turret * turyeet, Shooter * yeeter);
    ~Limelight();

    units::inch_t calcDist();
    void scoreOperation();
    bool aimOperation();
    double getTX() {return table->GetNumber("tx",0.0);}
    double getTY() {return table->GetNumber("ty",0.0);}
};
#endif
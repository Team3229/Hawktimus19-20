#ifndef LIMELIGHT_H
#define LIMELIGHT_H

#pragma once

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include "Turret.h"
#include "Shooter.h"

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

    double turretNoTargetPos; 
public:
    Limelight(Turret * turyeet, Shooter * yeeter);
    ~Limelight();

    void limelightLED(int mode){table->PutNumber("ledMode",mode);} //0 pipeline setting, 1 force off, 2 force blink, 3 force on
    void limelightPipeLine(int mode){table->PutNumber("pipeline",mode);} //0 powerport aim, 1 driver camera
    void limelightCamMode(int mode){table->PutNumber("camMode",mode);}

    units::inch_t calcDist();
    void scoreOperation();
    bool aimOperation();
    void scoreWithPOVManual(int povValue,int rpm);
    double getTX() {return table->GetNumber("tx",0.0);}
    double getTY() {return table->GetNumber("ty",0.0);}
    void limelightDash();
};

#endif
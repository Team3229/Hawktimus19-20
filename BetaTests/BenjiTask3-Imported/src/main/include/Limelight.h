// Author: Team 3229 Programming Subteam

#ifndef LIMELIGHT_H
#define LIMELIGHT_H

#include "DriveSystem.h"
#include "Debug.h"

#include <iostream>
#include <frc/smartdashboard/Smartdashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

class Limelight
{
public:
    Limelight(DriveSystem * chassis);
    ~Limelight();
    void GetValues();
    bool IsTargeting();
    void SeekTarget();  

private:
    DriveSystem * visionChassis;
   
    float m_validObject; // yes/no is there a target on screen "tv"
    float m_xOffset; // x angle offset from target "tx"
	float m_yOffset; // y "ty"
    float m_skew;
    float m_targetDistance; // % of screen filled by current target "ta"
    float m_vertDistance; //"tvert"
    float m_shortDistance; // "tshort"
    
    //threshholds and desires
    const float STRAFE_RANGE = 4.0;

    // drive powers
    double m_stillPow = 0.0;
    double m_leftAdjPow = -0.3; //turn
    double m_rightAdjPow = 0.3; //turn
    double m_leftStrafePow = -.55;
    double m_rightStrafePow = .55;

};

#endif // LIMELIGHT_H
// Author: Team 3229 Programming Subteam

#ifndef PNEUMATICS_H
#define PNEUMATICS_H

#include "Debug.h"

#include <iostream>
//#include <frc/WPILib.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Compressor.h>
#include <frc/Timer.h>

class Pneumatics
{
public:
    Pneumatics();
    ~Pneumatics();
    void MoveFrontClimb();
    void MoveBackClimb();
    void ControlComp();

private:
    // Solenoids
    frc::DoubleSolenoid * frontClimb;
    frc::DoubleSolenoid * backClimb;
    // Compressor(s)
    frc::Compressor * comp;
    
    // ports
    const int FRONT_FORWARD_ID = 0;
    const int FRONT_REVERSE_ID = 1;
    
    const int BACK_FORWARD_ID = 2;
    const int BACK_REVERSE_ID = 3;

    bool m_frontToggle = true;
    bool m_backToggle = true;

    const int COMP_ID = 0;
    bool m_pressureSwitch = false;

};

#endif // PNEUMATICS_H
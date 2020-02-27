#ifndef CLIMB_H
#define CLIMB_H

#pragma once

#include <frc/DoubleSolenoid.h>
#include <frc/Compressor.h>
#include <frc/Timer.h>

#include "Debug.h"

class Climb {
  public:
    Climb();
    ~Climb();
    void ControlComp();
    void ClimbUp();

  private:
    frc::Compressor * comp;


    const int COMP_ID = 0;
    bool m_pressureSwitch = false;


};

#endif // CLIMB_H
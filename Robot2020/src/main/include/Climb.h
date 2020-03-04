#ifndef CLIMB_H
#define CLIMB_H

#pragma once

#include <frc/DoubleSolenoid.h>
#include <frc/Timer.h>

#include "Debug.h"

class Climb {
  public:
    Climb();
    ~Climb();
    void ToggleClimb();

  private:
    // Compressor handled in intake
    frc::DoubleSolenoid * m_climber;

    const int FORWARD_ID = 0;
    const int REVERSE_ID = 1;
};

#endif // CLIMB_H
#include "Climb.h"

Climb::Climb() {
  comp = new frc::Compressor(COMP_ID);
  comp->SetClosedLoopControl(true);
}

Climb::~Climb() {
  comp->SetClosedLoopControl(false);
  delete comp;
}

void Climb::ControlComp() {
  m_pressureSwitch = comp->GetPressureSwitchValue();
  debugCons("Pressure switch state: " << m_pressureSwitch << "\n");
  if (m_pressureSwitch == true)
    comp->SetClosedLoopControl(false);
  else 
    comp->SetClosedLoopControl(true);
}

void Climb::ClimbUp() {

}
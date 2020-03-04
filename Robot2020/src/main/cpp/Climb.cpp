#include "Climb.h"

Climb::Climb() {
  m_climber = new frc::DoubleSolenoid(FORWARD_ID, REVERSE_ID);

  m_climber->ClearAllPCMStickyFaults();
  // Start pos
  m_climber->Set(frc::DoubleSolenoid::Value::kReverse);
}

Climb::~Climb() {
  delete m_climber;
}

void Climb::ToggleClimb() {
  // Run the climber pneumatics, acts as a toggle
  if (m_climber->Get() == frc::DoubleSolenoid::Value::kReverse) {
    m_climber->Set(frc::DoubleSolenoid::Value::kForward);
  } else {
    m_climber->Set(frc::DoubleSolenoid::Value::kReverse);
  }

  debugCons("Climber pneumatics toggled.\n");
  frc::Wait(1.0);
}
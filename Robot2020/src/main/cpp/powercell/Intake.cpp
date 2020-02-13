#include "powercell/Intake.h"

Intake::Intake(/* args */)
{
    m_compressor = new frc::Compressor{kCompressorPCMID};
    m_intakeSolenoid = new frc::DoubleSolenoid{kForwardIntakeID,kReverseIntakeID};
    m_intakeMotor = new rev::CANSparkMax{kIntakeMotorID,rev::CANSparkMax::MotorType::kBrushless};

    m_intakeSolenoid->ClearAllPCMStickyFaults();
    m_compressor->ClearAllPCMStickyFaults();
    m_intakeMotor->RestoreFactoryDefaults();

    m_compressor->SetClosedLoopControl(true);
}

Intake::~Intake()
{
    m_compressor->SetClosedLoopControl(false);
    delete m_compressor;
    delete m_intakeMotor;
    delete m_intakeSolenoid;
}
void Intake::controlComp()
{
    bool compressorState = m_compressor->GetPressureSwitchValue();
    (compressorState) ? (m_compressor->SetClosedLoopControl(false))
    : (m_compressor->SetClosedLoopControl(true));
}
void Intake::extendIntake()
{
    m_intakeSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
    intakeExtended = true;
    frc::Wait(.5);
}
void Intake::retractIntake()
{
    m_intakeSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    intakeExtended = false;
    frc::Wait(.5);
}

void Intake::runIntake()
{
    (intakeExtended) ? (m_intakeMotor->Set(.4))
    :(stopIntake());
}
void Intake::reverseIntake()
{
    (intakeExtended) ? (m_intakeMotor->Set(-.4))
    :(stopIntake());
}
void Intake::stopIntake()
{
    m_intakeMotor->StopMotor();
}
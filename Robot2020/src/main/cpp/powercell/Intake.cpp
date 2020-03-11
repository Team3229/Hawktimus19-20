#include "powercell/Intake.h"

Intake::Intake()
{
    m_compressor = new frc::Compressor(kCompressorPCMID);
    m_intakeSolenoid = new frc::DoubleSolenoid(kForwardIntakeID,kReverseIntakeID);
    m_intakeMotor = new rev::CANSparkMax(kIntakeMotorID,rev::CANSparkMax::MotorType::kBrushless);
    //m_intakeMotor = new ctre::phoenix::motorcontrol::can::WPI_TalonSRX(kIntakeMotorID);
    m_intakeSolenoid->ClearAllPCMStickyFaults();
    m_compressor->ClearAllPCMStickyFaults();
    
    m_intakeMotor->RestoreFactoryDefaults();
    //m_intakeMotor->ClearStickyFaults();

    m_compressor->SetClosedLoopControl(true);
}

Intake::~Intake()
{
    m_compressor->SetClosedLoopControl(false);
    delete m_compressor;
    delete m_intakeMotor;
    delete m_intakeSolenoid;
}

/**
 * Control Compressor, low pressure -> turn off, high pressure, turn on.
 * Currently unused
 */ 
void Intake::controlComp()
{
    bool compressorState = m_compressor->GetPressureSwitchValue();
    if (!compressorState) { 
        m_compressor->SetClosedLoopControl(false);
    } else {
        m_compressor->SetClosedLoopControl(true);
    }
}
/**
 * Pull down intake
 */ 
void Intake::extendIntake()
{
    m_intakeSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
    intakeExtended = true;
    frc::Wait(.5);
}
/**
 * Pull back intake
 */
void Intake::retractIntake()
{
    m_intakeSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    intakeExtended = false;
    frc::Wait(.5);
}
/**
 * Run intake with check of intake pulled down or not
 * Currently unused, using forceRunIntake() right now
 */ 
void Intake::runIntake()
{
    if (intakeExtended) {
        m_intakeMotor->Set(INTAKE_POWER_IN);
    } else {
        stopIntake();
    }
}
/**
 * Reverse intake, check intake extended or not
 * Currently unused, no need of reversing intake
 */ 
void Intake::reverseIntake()
{
    if (intakeExtended) {
        m_intakeMotor->Set(INTAKE_POWER_OUT);
    } else {
        stopIntake();
    }
}
/**
 * Run intake
 */ 
void Intake::forceRunIntake()
{
    m_intakeMotor->Set(INTAKE_POWER_IN);
}
/**
 * stop intake
 */ 
void Intake::stopIntake()
{
    m_intakeMotor->StopMotor();
}


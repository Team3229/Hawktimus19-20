#include "powercell/Turret.h"

Turret::Turret()
{
    m_drive = new Drivetrain();

    m_turretMotor = new ctre::phoenix::motorcontrol::can::WPI_TalonSRX(kTurretMotorID);
    m_turretPID = new frc2::PIDController(kP,kI,kD);
    m_turEncoder = new frc::AnalogEncoder(turretEncID);
    m_turretPID->SetTolerance(kNOMINAL_TX_ERROR);
    debugCons("\nEncoder Reset")   
    m_turEncoder->Reset();

    m_turretMotor->ClearStickyFaults();
}

Turret::~Turret()
{
    delete m_drive;
    delete m_turretMotor;
    delete m_turretPID;
    delete m_turEncoder;
}

/**
 * Turn based on Limelight tx Value
 * @return true if tx is within error range
 */ 
bool Turret::VisionTurn(double tX)
{
    //it should output 0 when error is reduced to kNOMINAL_TX_ERROR with the setTolarence
    double output = -m_turretPID->Calculate(tX,0);   //negative output if -tx doesn't work in aimOperation()
    if(m_turretPID->AtSetpoint()){
        Turn(0);
        return true;
    }else{
        Turn(output);
        return false;
    }
}

/**
 * turn with set motor power
 * @param setpower % power set to motor
 */
void Turret::Turn(double setPower)
{
    debugDashNum("(Tur) Turret Power", setPower);
    //limits turret turn range
    if (std::abs(GetAngle()) < kMAX_RANGE.to<double>()){
        debugDashNum("(Tur) larger than max range",0);
        m_turretMotor->Set(std::clamp(setPower,-kMAX_TURRET_POWER,kMAX_TURRET_POWER));
    }else{
        debugDashNum("(Tur) larger than max range",1);
        //prevent locking of the turret
        m_turretMotor->Set(std::clamp(-setPower,-kMAX_TURRET_CORRECT_POWER,kMAX_TURRET_CORRECT_POWER));
    }
}
/**
 * finding the target based on turret angle and gyro angle, limited to turret motion
 * called when limelight aimOperation can't find a target
 * attemp to find the correct target on the field
 * @param startPos, last position registered with a target
 */ 
void Turret::findTarget(double startPos)
{
    double robotPos = m_drive->GetAngle().Degrees().to<double>(); 
    double turretPos = GetAngle();
    /**
     * check if the chassis angle is possible to have a target
     * check if the turret angle with rotating angle
     *     ^            2,3 is the range that limelight will detect a target from turret
     *   2   3          1,4 is the range that limelight can detect a target with chassis position (facing powerport at start)
     * 1   O   4        O is turret center
     */ 
    if(robotPos > -85 && robotPos < 85)
    {
        //if the turret is within 2 and 3, perform check by turning left all the way and turn back right
        if(startPos > -55.0 && startPos < 55.0 && canReverse){
            reverseDirection = true;
        }else{
            reverseDirection = false;
        }
        //main check of turning condition
        if(startPos > 55.0 && !reverseDirection){
            Turn(-kMAX_TURRET_POWER); //turn left if turret is > 3
        }else if(startPos < -55.0 && !reverseDirection){
            Turn(kMAX_TURRET_POWER); //right right if turret is < 2
        }else if(turretPos > -59.0 && startPos < 0 && reverseDirection){
            Turn(-kMAX_TURRET_POWER); //turn left if turret is starting within 2,^
            //no longer reverse since it has completed checking
            if(turretPos < -55.0){  
                reverseDirection = !reverseDirection;
                canReverse = false;
            }
        }else if(turretPos < 59.0 && startPos > 0 && reverseDirection){
            Turn(kMAX_TURRET_POWER); //turn right if turret is starting within ^,3
            //no longer reverse since it has completed checking
            if(turretPos > 55.0){
                reverseDirection = !reverseDirection;
                canReverse = false;
            }
        }else{
            Turn(0);
            debugCons("\nproblem with turret in findTarget(), turret: " << turretPos)
            debugCons("\tstarting pos: " << startPos)
        }
    }else{
        //implement chassis turn below, try not to create conflict with control in robot.cpp
        debugCons("\nproblem with gyro in findTarget(), gyro: " << robotPos)
    }
}
/**
 * ratio of angle:encoder is 73.17:1
 * GetDistance() start: 0, left 90: -1.23, right 90: 1.23
 * Note: around 180 degrees the encoder jumps from 2.x to -2.x (or opposite) 
 * and then original 0 becomes -1
 * @return double, angle of the turret from starting position (when power cycle/restart robot code)
 */
double Turret::GetAngle()
{
    return m_turEncoder->GetDistance() * kENCODER_RATIO;
}

/**
 * Debugging Turret
 */ 
void Turret::turretDash()
{
    debugDashNum("(Tur) Turret Encoder",GetAngle());
    debugDashNum("(Tur) atSetpoint",m_turretPID->AtSetpoint());
}
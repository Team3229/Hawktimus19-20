// Author: Team 3229 Programming Subteam

#include "DriveSystem.h"

DriveSystem::DriveSystem()
{
    //Instantiate motor controllers
	leftLead = new WPI_TalonSRX(LEFT_LEAD_ID);
	rightLead = new WPI_TalonSRX(RIGHT_LEAD_ID);
	leftFollower = new WPI_TalonSRX(LEFT_FOLLOWER_ID);
	rightFollower = new WPI_TalonSRX(RIGHT_FOLLOWER_ID);

    //Set followers
	leftFollower->Follow(*leftLead);
	rightFollower->Follow(*rightLead);

	//Instantiate DriveTrain
	driveTrain = new frc::MecanumDrive(*leftLead, *leftFollower, *rightLead, *rightFollower);

    //Reset the controllers
	leftLead->Set(ControlMode::PercentOutput, 0);
	rightLead->Set(ControlMode::PercentOutput, 0);
	leftFollower->Set(ControlMode::PercentOutput, 0);
	leftFollower->Set(ControlMode::PercentOutput, 0);

    //Clears sticky faults
	leftLead->ClearStickyFaults(0);
	rightLead->ClearStickyFaults(0);
	leftFollower->ClearStickyFaults(0);
	rightFollower->ClearStickyFaults(0);

	//Sets smoothing curve to talons
	leftLead->ConfigOpenloopRamp(SMOOTH_TIME, 0); //passes in seconds from neutral to full and timeout in miliseconds
	rightLead->ConfigOpenloopRamp(SMOOTH_TIME, 0);
	leftFollower->ConfigOpenloopRamp(SMOOTH_TIME, 0);
	rightFollower->ConfigOpenloopRamp(SMOOTH_TIME, 0);

	//Sets max initial speed and turn off safety
	driveTrain->SetMaxOutput(MAX_OUTPUT);
	driveTrain->SetSafetyEnabled(false); 
	driveTrain->SetExpiration(SAFETY_TIMEOUT); //Set safety timer

    //navX stuff
    navxGyro = new AHRS(SPI::Port::kMXP);
}

DriveSystem::~DriveSystem()
{
    delete leftLead;
	delete rightLead;
	delete leftFollower;
	delete rightFollower;
	delete driveTrain;
	delete navxGyro;
}

void DriveSystem::Drive(double& Y, double& X, double& Z)
{
    //Drive function
	Y = -Y; // invert Y
    driveTrain->DriveCartesian(Y, X, Z, navxGyro->GetAngle()); //Optional 4th parameter for gyro input
	debug("Drive mode: With Gyro\n");
}

void DriveSystem::DriveWithoutGyro(double& Y, double& X, double& Z)
{
	Y = -Y;
	driveTrain->DriveCartesian(Y, X, Z); // without gyro FOR TESTING
	debug("Drive mode: Without Gyro\n");
}

void DriveSystem::Stop()
{
    driveTrain->StopMotor();
}

void DriveSystem::ChangeSpeed(int choice)
{
    if (choice == 1)
	{
        driveTrain->SetMaxOutput(LOW_OUTPUT);
    	debug("Slow speed\n");
	}
    else if (choice == 2)
	{
        driveTrain->SetMaxOutput(MAX_OUTPUT);
    	debug("Normal speed\n");
	}
    else if (choice == 3)
	{
        driveTrain->SetMaxOutput(HIGH_OUTPUT);
    	debug("Fast speed\n");
	}
}

double DriveSystem::TestGyro()
{
    return navxGyro->GetAngle();
}

void DriveSystem::ResetGyro()
{
	navxGyro->Reset();
}

void DriveSystem::DetermineTarget(std::string temp)
{
	if (temp == "Other") // Other mode angle determine
	{
		m_desiredAngle = OTHER_ANGLE[int((navxGyro->GetYaw() + 225)/90)];
	}
	else // Rocket & Hatch mode angle determine
	{
		m_desiredAngle = HATCH_ANGLE[int((navxGyro->GetYaw() + 180)/90)];
	}
}

void DriveSystem::TurnToTarget()
{
	if (CanTurn() == true)
	{
		if ((m_desiredAngle - 180) > navxGyro->GetYaw())
		{
			DriveWithoutGyro(m_stillPow, m_stillPow, m_rightAdjPow); // right turn
		}
		else if ((m_desiredAngle - 180) < navxGyro->GetYaw())
		{
			DriveWithoutGyro(m_stillPow, m_stillPow, m_leftAdjPow); // left turn
		}
	}
	else
		DriveWithoutGyro(m_stillPow, m_stillPow, m_stillPow); 
}

bool DriveSystem::CanTurn()
{
	if (abs((m_desiredAngle - 180) - navxGyro->GetYaw()) > ANGLE_THRESH)
		return true;
	else
		return false;
}
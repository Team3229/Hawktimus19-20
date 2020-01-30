// Author: Team 3229 Programming Subteam

#ifndef DRIVESYSTEM_H
#define DRIVESYSTEM_H

#include <iostream>
#include <memory>
#include <Math.h>
#include "Debug.h"

//Phoenix 
#include "ctre/Phoenix.h" // add vendor library
#include "ctre/phoenix/motorcontrol/can/TalonSRX.h" // new file WPI_TalonSRX is depricated

//FRC
#include <frc/GenericHID.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/SpeedController.h>

//navX
#include "AHRS.h"				//add vendor library

class DriveSystem
{
public:
    DriveSystem();
    ~DriveSystem();
    void Drive(double& Y, double& X, double& Z);
	void DriveWithoutGyro(double& Y, double& X, double& Z);
    void Stop();
    void ChangeSpeed(int choice);
    double TestGyro();
	void ResetGyro();
	void DetermineTarget(std::string temp);
	void TurnToTarget();
	bool CanTurn();

private:
    //TalonSRX's
	WPI_TalonSRX * leftLead; //Front left, leader
	WPI_TalonSRX * rightLead; //Front right, leader
	WPI_TalonSRX * leftFollower; //Back left, follower
	WPI_TalonSRX * rightFollower; //Back right, follower

	frc::MecanumDrive * driveTrain; //Drivetrain

    AHRS * navxGyro; //navX drive

	//Desired Gyro Angle
	double m_desiredAngle;
	const float HATCH_ANGLE[4] = {28.5, 151.5, 208.5, 331.5};
	const float OTHER_ANGLE[5] = {0.01, 90.0, 180.0, 270.0, 359.9};
	const float ANGLE_THRESH = 8.0;

    //Constants for ports and unique id
	const int LEFT_LEAD_ID = 1;
	const int LEFT_FOLLOWER_ID = 2;
	const int RIGHT_LEAD_ID = 3;
	const int RIGHT_FOLLOWER_ID = 4;

    //Changable powers
	const float MAX_OUTPUT = 0.7;
	const float LOW_OUTPUT = 0.4;
	const float HIGH_OUTPUT = 1.0;

	const float SMOOTH_TIME = 0.4;
	const float SAFETY_TIMEOUT = 2.0;

	//Turning powers
	double m_leftAdjPow = -0.3; //turn
    double m_rightAdjPow = 0.3; //turn
	double m_stillPow = 0.0;
};

#endif // DRIVESYSTEM_H
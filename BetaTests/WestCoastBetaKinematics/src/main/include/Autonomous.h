#pragma once

#include "Drivetrain.h"
#include <frc2/Timer.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

class Autonomous
{
    public:
        Autonomous(Drivetrain * drive);
        ~Autonomous();
        void AutoInit();
	    void ReadStation();
	    void AutoPeriodic();
	    void AddOptions();
	    void SetupAutoCommands();

	    bool autodone = false;
    private:
        Drivetrain * m_drive;

        frc2::Timer m_timer;

        frc::SendableChooser<int> * m_positionChooser;
        frc::SendableChooser<int> * m_targetChooser;
        
        enum Positions{LeftUpper,LeftLower,Center,RightUpper,RightLower};
        enum Targets{LeftRocket,LeftShip,FrontShip,RightShip,RightRocket};
        enum Movements{M0,M1,M2,M3,M4,M5,M6};
        enum Commands{SetEncoder,Done};

        struct cmd
        {
            Commands command;
            //SetEncoder()
            double setEncoder;
        };
        cmd autocommand [5][5][7];//positions | targets | movements
        int movement = 0;
        Positions positionEnum = LeftUpper;
        Targets targetEnum = LeftRocket;
        units::time::second_t m_timeLimit;
};
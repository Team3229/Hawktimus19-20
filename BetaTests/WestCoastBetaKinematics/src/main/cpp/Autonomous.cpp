#include <Autonomous.h>

Autonomous::Autonomous(Drivetrain * drive)
{
    m_positionChooser = new frc::SendableChooser<int>();
    m_targetChooser = new frc::SendableChooser<int>();
    m_drive = drive;
}

Autonomous::~Autonomous()
{
    delete m_positionChooser;
    delete m_targetChooser;
    delete m_drive;
}
/*
Auto presets
read selection
reset timer, movement, set auto to incomplete
*/
void Autonomous::AutoInit()
{
    ReadStation();
    m_timer.Reset();
    movement = 0;
    autodone = false;
}
/*
read selection for autocommands
*/
void Autonomous::ReadStation()
{
    int targetChoice = m_targetChooser->GetSelected();
    int positionChoice = m_positionChooser->GetSelected();

    if(targetChoice == 0)
        targetEnum = LeftRocket;
    else if(targetChoice == 1)
    	targetEnum = LeftShip;
    else if(targetChoice == 2)
    	targetEnum = FrontShip;
    else if(targetChoice == 3)
        targetEnum = RightShip;
    else if(targetChoice == 4)
    	targetEnum = RightRocket;
    
    if(positionChoice == 0)
        positionEnum = LeftUpper;
    else if(positionChoice == 1)
        positionEnum = LeftLower;
    else if(positionChoice == 2)
        positionEnum = Center;
    else if(positionChoice == 3)
        positionEnum = RightUpper;
    else if(positionChoice == 4)
        positionEnum = RightLower;
}
/*
Auto periodic with switch statement reading commands to perform tasks
schedule for timer based tasks
    1.  start timer
        set timelimit
    2.  timer <= timelimit
        perform task
    3.  else (timer > timelimit)
        stop timer
        reset timer
        increase movement for next task
    break
schedule for single time tasks
    1.  perform task
        increase movement
    break
*/
void Autonomous::AutoPeriodic()
{
    while(!autodone)
    {
        switch(autocommand[positionEnum][targetEnum][movement].command)
        {
            case SetEncoder:
                if(m_timer.Get()==0_s)
                {
                    m_timeLimit = 4_s;
                    m_timer.Start();
                }
                else if (m_timer.Get()<=m_timeLimit)
                    m_drive->SetEncoder(autocommand[positionEnum][targetEnum][movement].setEncoder);
                else
                {
                    m_timer.Stop();
                    m_timer.Reset();
                    movement++;
                }
                break;
            case Done:
                autodone = true;
                break;
        }
    }
}

/*
smartdashboard selections
*/
void Autonomous::AddOptions()
{
    int leftUpperOpt = 0;
    int leftLowerOpt = 1;
    int centerOpt = 2;
    int rightUpperOpt = 3;
    int rightLowerOpt = 4;
    m_positionChooser->SetDefaultOption("Left 2nd level",leftUpperOpt);
    m_positionChooser->AddOption("Left 1st level",leftLowerOpt);
    m_positionChooser->AddOption("Center",centerOpt);
    m_positionChooser->AddOption("Right 2nd level",rightUpperOpt);
    m_positionChooser->AddOption("Right 1st level",rightLowerOpt);
    frc::SmartDashboard::PutData("Starting Position",m_positionChooser);

    int leftRocketHatchOpt = 0;
    int leftShipCargoOpt = 1;
    int frontCargoHatchOpt = 2;
    int rightShipCargoOpt = 3;
    int rightRocketHatchOpt = 4;
    m_targetChooser->SetDefaultOption("Left Rocket Hatch",leftRocketHatchOpt);
    m_targetChooser->AddOption("Left Ship Cargo",leftShipCargoOpt);
    m_targetChooser->AddOption("Front Ship Hatch",frontCargoHatchOpt);
    m_targetChooser->AddOption("Right Ship Cargo",rightShipCargoOpt);
    m_targetChooser->AddOption("Right Rocket Hatch",rightRocketHatchOpt);
    frc::SmartDashboard::PutData("Targeted Area",m_targetChooser);
}

/*
all of the commands for different scenarios
1.  sets command
    sets data necessary to perform task
*remember to end with Done command
*/
void Autonomous::SetupAutoCommands()
{
    autocommand[LeftLower][LeftRocket][M0].command = SetEncoder;
    autocommand[LeftLower][LeftRocket][M0].setEncoder = 100;
    autocommand[LeftLower][LeftRocket][M1].command = SetEncoder;
    autocommand[LeftLower][LeftRocket][M1].setEncoder = 0;
    autocommand[LeftLower][LeftRocket][M2].command = SetEncoder;
    autocommand[LeftLower][LeftRocket][M2].setEncoder = 200;
    autocommand[LeftLower][LeftRocket][M3].command = Done;                                         
}
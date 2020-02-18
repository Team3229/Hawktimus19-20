#include "drive/DriveTrajectory.h"

DriveTrajectory::DriveTrajectory()
{
    m_trajConfig = new frc::TrajectoryConfig(Drivetrain::kMaxSpeed,kMaxAccel);
    m_ramseteControl = new frc::RamseteController(kB, kZeta);
    m_drive = new Drivetrain();

    m_ramseteControl->SetTolerance(errorPose);
}

DriveTrajectory::~DriveTrajectory()
{
    delete m_trajConfig;
    delete m_ramseteControl;
    delete m_drive;
}
//set trajectory config
void DriveTrajectory::setConfig(units::meters_per_second_t startVel,
                                units::meters_per_second_t endVel,
                                bool reverse)
{
    m_trajConfig->SetStartVelocity(startVel);
    m_trajConfig->SetEndVelocity(endVel);
    m_trajConfig->SetReversed(reverse);
}
/*
Generate clamped cubic splines with start point, end point and way points
###Remember to set configs before using this###

Clamped Cubic Splines - headings of start and end, heading during waypoints automatically generated for continous curvature
Quintic Hermite Splines - all headings of all way points (finer control during trajectory if needed)
*/
frc::Trajectory DriveTrajectory::generateClampedTraj(frc::Pose2d start,
                                                     frc::Pose2d end,
                                                     std::vector<frc::Translation2d>& waypoints)
{
    frc::Trajectory traj = frc::TrajectoryGenerator::GenerateTrajectory(start,waypoints,end,*m_trajConfig);
    return traj;
}

frc::Trajectory DriveTrajectory::generateHermiteTraj(std::vector<frc::Pose2d>& waypoints)
{
    frc::Trajectory traj = frc::TrajectoryGenerator::GenerateTrajectory(waypoints, *m_trajConfig);
    return traj;
}
/*
Follow Trajectory with velocity and angular velocity
*/
void DriveTrajectory::followTraj(units::second_t time,frc::Trajectory traj)
{
    //get the supposed position the robot should be at this time, added time for motor response
    m_nextPos = traj.Sample(time+(m_nextOperation/2));
    //get velocity from struct
    m_currentVelocity = m_nextPos.velocity;
    //calculate angular velocity from struct (velocity * rate of rotation of heading)
    m_currentRot = m_currentVelocity * m_nextPos.curvature;
    //drive
    m_drive->Drive(m_currentVelocity,m_currentRot);
}

/*
Follow trajectory with Ramsete Controller (adjust as following trajectory)
needs Gyro for the Pose2d
*/
void DriveTrajectory::followRamsete(units::second_t time,frc::Trajectory traj)
{
    m_nextPos = traj.Sample(time);
    frc::ChassisSpeeds adjustedSpeeds = m_ramseteControl->Calculate(m_drive->GetPose(),
                                                                  m_nextPos);
    m_drive->SetSpeeds(m_drive->GetKinematics().ToWheelSpeeds(adjustedSpeeds));
}

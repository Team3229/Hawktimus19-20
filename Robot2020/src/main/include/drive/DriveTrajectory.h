#ifndef DRIVETRAJECTORY_H
#define DRIVETRAJECTORY_H

#pragma once

#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/controller/RamseteController.h>

#include "Drivetrain.h"

class DriveTrajectory
{
    public:
        DriveTrajectory();
        ~DriveTrajectory();
        void setConfig(units::meters_per_second_t startVel,
                       units::meters_per_second_t endVel,
                       bool reverse = false);
        frc::Trajectory generateClampedTraj(frc::Pose2d start,
                                            frc::Pose2d end,
                                            std::vector<frc::Translation2d>& waypoints);
        frc::Trajectory generateHermiteTraj(std::vector<frc::Pose2d>& waypoints);

        void followTraj(units::second_t time,   frc::Trajectory traj);
        void followRamsete(units::second_t time,frc::Trajectory traj);
        
        units::second_t GetTotalTime(frc::Trajectory traj){return traj.TotalTime();}

        
    private:

        //constants
        const units::meters_per_second_squared_t kMaxAccel = 1_mps_sq;
        const units::second_t m_nextOperation = .05_s;
        const double kB = 2.0, kZeta = .7;
        const frc::Pose2d errorPose = frc::Pose2d(.1_m,.1_m,frc::Rotation2d(5_deg));
        //values from trajectory.Sample(time)
        units::meters_per_second_t m_currentVelocity;
        units::radians_per_second_t m_currentRot;
        frc::Pose2d m_setPos;
        //trajectory
        frc::TrajectoryConfig * m_trajConfig;
        frc::Trajectory::State m_nextPos;
        frc::RamseteController * m_ramseteControl;

        Drivetrain * m_drive;
};

#endif 
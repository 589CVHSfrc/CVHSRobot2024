// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

/** Add your docs here. */

public class SwerveControllerCommandMaker {
    private final ProfiledPIDController m_thetaController;
    private final DriveSubsystem m_robotDrive;
    private final PIDController m_xPIDController;
    private final PIDController m_yPIDController;

    public SwerveControllerCommandMaker(
            ProfiledPIDController thetaController, PIDController xPIDController, PIDController yPIDController,
            DriveSubsystem robotDrive) {
        m_thetaController = thetaController;
        m_xPIDController = xPIDController;
        m_yPIDController = yPIDController;
        m_robotDrive = robotDrive;

        
    }
    // public SwerveControllerCommandMaker(
    //         ProfiledPIDController thetaController, PIDController xPIDController, PIDController yPIDController,
    //         DriveSubsystem robotDrive, Trajectory trajectory) {
    //     m_thetaController = thetaController;
    //     m_xPIDController = xPIDController;
    //     m_yPIDController = yPIDController;
    //     m_robotDrive = robotDrive;
    //     m_trajectory = trajectory;

        
    // }


    public SwerveControllerCommand makeCommand(Trajectory trajectory) {
        return new SwerveControllerCommand(
                trajectory,
                m_robotDrive::getPose, // Functional interface to feed supplier
                DriveConstants.kDriveKinematics,

                // Position controllers
                m_xPIDController,
                m_yPIDController,
                m_thetaController,
                m_robotDrive::setModuleStates,
                m_robotDrive);
    }



    //////////////////////////////
    //////////////////////////////
    
    // Does not work as of now
    // public SwerveControllerCommand makeCommandPose(Pose2d pose, TrajectoryConfig trajectoryConfig) {
    //     Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    //             m_robotDrive.getPose(),
    //             List.of(),
    //             pose,
    //             trajectoryConfig);
    //             // new Thread().sleep();
    //     return new SwerveControllerCommand(
    //             trajectory,
    //             m_robotDrive::getPose, // Functional interface to feed supplier
    //             DriveConstants.kDriveKinematics,

    //             // Position controllers
    //             m_xPIDController,
    //             m_yPIDController,
    //             m_thetaController,
    //             m_robotDrive::setModuleStates,
    //             m_robotDrive);
    // }
}
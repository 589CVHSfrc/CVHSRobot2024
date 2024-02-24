// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.Optional;


import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.net.PortForwarder;
import frc.robot.Constants.VisualConstants;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class LimeLight {
    private static LimeLight m_limelight;
    // String m_tableName;
    // NetworkTable m_table;
    private AprilTagFieldLayout m_aprilTagLayout;
    private PhotonCamera m_photonCamera = new PhotonCamera("aprilcamera");
    private PhotonPoseEstimator m_estimator;

    private LimeLight() {
        try {
            m_aprilTagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);

        } catch (IOException e) {
            System.out.println("======Unable to load AprilTag Layout: ======");
            System.out.print(e);
        }
        m_aprilTagLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
        // m_photonCamera = new PhotonCamera(VisualConstants.kPhotonCameraName);
        m_estimator = new PhotonPoseEstimator(m_aprilTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                m_photonCamera,
                VisualConstants.kCameraRelativeToRobot);
        m_estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        PortForwarder.add(5800, "photonvision.local", 5800);

    }

    public static LimeLight get() {
        if (m_limelight == null) {
            m_limelight = new LimeLight();
            return m_limelight;
        }
        return m_limelight;
    }

    public void estimatePose(SwerveDrivePoseEstimator estimator, DriveSubsystem drive) {
        Optional<EstimatedRobotPose> OPestimation = m_estimator.update();

        if (OPestimation.isPresent()) {
            EstimatedRobotPose estimation = OPestimation.get();
            Pose2d estimatedPose2d = estimation.estimatedPose.toPose2d();
            estimator.addVisionMeasurement(estimatedPose2d, estimation.timestampSeconds);
            estimator.resetPosition(estimatedPose2d.getRotation(),
                    drive.getSwerveModulePositions(),
                    estimatedPose2d);
        }
    }

}
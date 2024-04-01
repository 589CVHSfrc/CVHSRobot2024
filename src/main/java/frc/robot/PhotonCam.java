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
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.VisualConstants;

public class PhotonCam {
    private static PhotonCam m_photonCam;
    public AprilTagFieldLayout m_aprilTagLayout;
    private PhotonCamera m_photonCamera = new PhotonCamera("aprilcamera");// CHANGE TO CONSTANT
    private PhotonPoseEstimator m_photonEstimator;

    private PhotonCam() {
        try {
            m_aprilTagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);

        } catch (IOException e) {
            System.out.println("======Unable to load AprilTag Layout: ======");
            System.out.println(e);
        }


        m_photonEstimator = new PhotonPoseEstimator(m_aprilTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                m_photonCamera,
                VisualConstants.kCameraRelativeToRobot);
    
        m_photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        PortForwarder.add(5800, "photonvision.local", 5800);
        PortForwarder.add(5800, "10.5.89.11", 5800);
        
    }

    /**
     * red = true
     * 
     * @return alliance as a bool
     */
    public boolean getAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {

            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }



    public static PhotonCam get() {
        if (m_photonCam == null) {
            m_photonCam = new PhotonCam();
        }
        return m_photonCam;
    }

    public void estimatePose(SwerveDrivePoseEstimator estimator) {
        // Gets Estimation from camera
        Optional<EstimatedRobotPose> OPestimation = m_photonEstimator.update();

        if (OPestimation.isPresent()) {
            EstimatedRobotPose estimation = OPestimation.get();
            Pose2d estimatedPose2d = estimation.estimatedPose.toPose2d();

            // if estimation exists, then add Vision Measurement, to odom in class
            estimator.addVisionMeasurement(estimatedPose2d, estimation.timestampSeconds);

            // estimator.resetPosition(estimatedPose2d.getRotation(),
            // drive.getSwerveModulePositions(),
            // estimatedPose2d);
        }
    }
    public void setAlliance(boolean alliance){
        if (alliance) {
            m_aprilTagLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

        } else {
            m_aprilTagLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);

        }
    }

}
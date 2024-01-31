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
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.net.PortForwarder;
import frc.robot.Constants.VisualConstants;

/** Add your docs here. */
public class LimeLight {

    // String m_tableName;
    // NetworkTable m_table;
    private AprilTagFieldLayout m_aprilTagLayout;
    private PhotonCamera m_photonCamera = new PhotonCamera("aprilcamera");
    private PhotonPoseEstimator m_estimator;
    private double m_estimatetime;

    public LimeLight() {
        // m_tableName = "limelight";
        // m_table = NetworkTableInstance.getDefault().getTable(m_tableName);
        try {
            m_aprilTagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);

        } catch (IOException e) {
            System.out.println("======Unable to load AprilTag Layout: ======");
            System.out.print(e);
        }
        // m_photonCamera = new PhotonCamera(VisualConstants.kPhotonCameraName);
        m_estimator = new PhotonPoseEstimator(m_aprilTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_RIO, m_photonCamera,
                VisualConstants.kCameraRelativeToRobot);
        PortForwarder.add(5800, "photonvision.local", 5800);

    }

    public double getTimestamp() {
        return m_estimatetime;
    }

    public Pose2d estimatePose(SwerveDrivePoseEstimator estimator) {
        Optional<EstimatedRobotPose> estimation = m_estimator.update();
        if (estimation.isPresent()) {
            m_estimatetime = estimation.get().timestampSeconds;
            System.out.print("ESTIMATION FOUND======================" + estimation.get().estimatedPose.toPose2d());
            return estimation.get().estimatedPose.toPose2d();
        }
        return estimator.getEstimatedPosition();
    }

    // public boolean getIsTargetFound() {
    // NetworkTableEntry tv = m_table.getEntry("tv");
    // double v = tv.getDouble(0);
    // if (v == 0.0f) {
    // return false;
    // } else {
    // return true;
    // }
    // }

    // /**
    // * tx Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
    // *
    // * @return
    // */
    // public double getdegRotationToTarget() {
    // NetworkTableEntry tx = m_table.getEntry("tx");
    // double x = tx.getDouble(0.0);
    // return x;
    // }

    // /**
    // * ty Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
    // *
    // * @return
    // */
    // public double getdegVerticalToTarget() {
    // NetworkTableEntry ty = m_table.getEntry("ty");
    // double y = ty.getDouble(0.0);
    // return y;
    // }

    // public double getSkew_Rotation() {
    // NetworkTableEntry ts = m_table.getEntry("ts");
    // double s = ts.getDouble(0.0);
    // return s;
    // }

    // /**
    // * tl The pipeline’s latency contribution (ms) Add at least 11ms for image
    // * capture latency.
    // *
    // * @return
    // */
    // public double getPipelineLatency() {
    // NetworkTableEntry tl = m_table.getEntry("tl");
    // double l = tl.getDouble(0.0);
    // return l;
    // }

}
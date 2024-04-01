package frc.utils;

import java.util.List;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveUtils {
    private DriveSubsystem m_drive;

    public DriveUtils(DriveSubsystem drive) {
        m_drive = drive;
        // m_drive.configureHolonomicAutoBuilder();
    }

    public Command driveToPoseORIG(Pose2d requestedPose) {
        Pose2d startingPose = m_drive.getPose();

        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startingPose, requestedPose);

        // Create the path using the bezier points created above
        PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                new PathConstraints(
                        DriveConstants.kMaxSpeedMetersPerSecond / 2,
                        AutoConstants.kMaxAccelerationMetersPerSecondSquared / 2,
                        DriveConstants.kMaxAngularSpeed,
                        AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared),
                new GoalEndState(0.1, requestedPose.getRotation()));

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;
        return AutoBuilder.followPath(path);
    }

    public Command driveToPose(Pose2d requestedPose, DoubleSupplier speed) {
        Pose2d startingPose = m_drive.getPose();
        double[] xy = { (requestedPose.getX() - startingPose.getX()),
                (requestedPose.getY() - startingPose.getY())  };
        Pose2d midPose = startingPose.plus(new Transform2d(xy[0] * .9, xy[1] * 0.6, new Rotation2d(0)));
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startingPose, midPose, requestedPose);

        // Create the path using the bezier points created above
        PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                new PathConstraints(
                        DriveConstants.kMaxSpeedMetersPerSecond * speed.getAsDouble(),
                        AutoConstants.kMaxAccelerationMetersPerSecondSquared,
                        DriveConstants.kMaxAngularSpeed,
                        AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared),
                new GoalEndState(0.05, requestedPose.getRotation()));

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;
        return AutoBuilder.pathfindToPose(requestedPose, new PathConstraints(
                        DriveConstants.kMaxSpeedMetersPerSecond * speed.getAsDouble(),
                        AutoConstants.kMaxAccelerationMetersPerSecondSquared,
                        DriveConstants.kMaxAngularSpeed,
                        AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared), 0);
        //return AutoBuilder.followPath(path);
    }

}
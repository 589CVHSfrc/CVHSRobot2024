package frc.utils;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
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

    public Command driveToPose(Pose2d requestedPose) {
        Pose2d startingPose = m_drive.getPose();

        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startingPose, requestedPose);

        // Create the path using the bezier points created above
        PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                new PathConstraints(
                        DriveConstants.kMaxSpeedMetersPerSecond,
                        AutoConstants.kMaxAccelerationMetersPerSecondSquared,
                        DriveConstants.kMaxAngularSpeed,
                        AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared),
                new GoalEndState(0.0, requestedPose.getRotation()));

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;
        return AutoBuilder.followPath(path);
    }
    
}
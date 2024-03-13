// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {

    public static final Pose2d kShootingPoseSpeakerRED = new Pose2d(14, 2, new Rotation2d(0));
    public static final Pose2d kShootingPoseSpeakerBLUE = new Pose2d(15.5, 2.6  ,
        new Rotation2d(Units.degreesToRadians(180)));

    public static final Pose2d kShootingPoseAmpRED = new Pose2d(14, 2, new Rotation2d(0));
    public static final Pose2d kShootingPoseAmpBLUE = new Pose2d(14, 2, new Rotation2d(0));

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.45;// ---> CHANGE 4.45 hello! 5
    public static final double kMaxAngularSpeed = 10.37; // --> CHANGE 10.32 hello!

    public static final double kDirectionSlewRate = 10; // 1.2 radians per second
    public static final double kMagnitudeSlewRate = 10; // 1.8 // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 10; // 2.0 percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(29);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(29);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    public static final int kFrontLeftDrivingCanId = 40;
    public static final int kRearLeftDrivingCanId = 20;
    public static final int kFrontRightDrivingCanId = 30;
    public static final int kRearRightDrivingCanId = 10;

    public static final int kFrontLeftTurningCanId = 41;
    public static final int kRearLeftTurningCanId = 21;
    public static final int kFrontRightTurningCanId = 31;
    public static final int kRearRightTurningCanId = 11;

    public static final double kAutoTimeDtSecondsAdjust = 0.02; // ?????????????????????

    public static final int kPigeon2CanId = 60;

    public static final boolean kGyroReversed = false;// false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth
    // will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.075797771765;// 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    // public static final double kDrivingP = 0.00006;// 0.04;
    public static final double kDrivingP = 0.04;// 0.04;
    public static final double kDrivingI = 0; // 0.0001;
    // public static final double kDrivingI = 0.0001; // 0.0001;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    // public static final double kTurningP = 0.55;// 0.5;
    public static final double kTurningP = 0.6;// 0.5;
    public static final double kTurningI = 0.0;
    public static final double kTurningD = 0.0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps

  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCODriverControllerPort = 1;
    public static final double kDriveDeadband = 0.1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4.45;// 4.45;// 1
    public static final double kMaxAccelerationMetersPerSecondSquared = 5;// 2
    public static final double kMaxAngularSpeedRadiansPerSecond = 10.37;// pi
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI * 5;// pi

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class VisualConstants {
    public static final Transform3d kCameraRelativeToRobot = new Transform3d(Units.inchesToMeters(-5), 0,
        Units.inchesToMeters(19.5), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(30), 0));// subject to change relative, x and
                                                                                    // y
    public static final String kPhotonCameraName = "limelight";
  }

  public static final class ArmConstants {

    public static final double kShootingAngleSpeaker = 65; // change this
    public static final double kShootingAngleAmp = 30; // change this
    public static final double kRestingAngle = 10; // change this
    public static final double kArmMaxRPM = 11000;
    public static final double kArmMinRPM = 0;
    public static final double kArmMaxAccel = 4000;
    public static final double kArmAllowedErr = 0.005;

    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kFF = 0;

    public static final int kAngleMotorCanID = 50;
    public static final int kDiscBrakeForwardID = 0;
    public static final int kDiscBrakeBackwardID = 1;

    // Physical Constants
    public static final double kStowSpeed = .2;// Change this
    public static final double kMinAngle = .01; // Change this
    public static final double kMaxArmAngle = .34;// Change this
    public static final int ArmScaleEncoder = 1; // Change this
    public static final int kArmGearRatio = 1; // Change this

  }

  public static final class ShooterConstants {

    public static final double kGatewayMotorSpeed = .2; // Change this
    public static final double kShooterMaxRPM = 11000;
    public static final double kShooterMinRPM = 0;
    public static final double kShooterMaxAccel = 4000;
    public static final double kShooterAllowedErr = 0.005;
    public static final double kShooterTime = 0.5;
    public static final double kShooterMaxVelocity = 50; // Change this

    public static final double kPt0 = 0.000003;
    public static final double kIt0 = 0.0000000009;
    public static final double kDt0 = 0;

    public static final double kPl0 = 0.1;
    public static final double kIl0 = 0.0000000009;
    public static final double kDl0 = 0;
    // public static final double kFF0t = 0;

    public static final int kShooterMotorTopCanID = 51;
    public static final int kShooterMotorLowCanID = 52;

    public static final int kTopGatewayWheelMotorID = 53;
    // public static final int kLowGatewayWheelMotorID = 54;

    // Physical Constants
    public static final double kShooterGearRatio = .5;

    public static final double kShooterSpeedSpeakerLow = -7800;//-7800
    public static final double kShooterSpeedSpeakerTop = -9500;//-9000

    public static final double kShooterSpeedAmpLow = -3500; //-3000 if want lower
    public static final double kShooterSpeedAmpTop = -2500; //-2000 if want lower
    // Change this - we need to make this hashmap/lookup table for dif
    // poses/dif objects -speaker amp
    public static final double kIntakeSpeed = 3000;// Change this
    public static final double kShooterSpeedSlow = .1;// Change this
    public static final boolean kShooterDirection = false; // false is spitting it out , true is takinShooter
  }

  public static final class ClimberConstants {
    public static final int kSmartCurrentLimitAmps = 30;

    // difference in rate refers to the difference between the left and right amps
    // over change in time;
    public static final double kEncoderIsRaised = 90; // Change this
    public static final double kDifferenceInRate = 2;
    public static final int kClimberLeftMotorCanID = 55;
    public static final int kClimberRightMotorCanID = 56;

    public static final double kLoweringClimbingSpeed = 0.6;// Pit: 0.1
    public static final double kClimberRaisingSpeed = 0.4;// Pit: 0.2

    public static final int kClimberLeftReverse = 2; //

    public static final int kClimberRightReverse = 5;// 5

    public static final int kClimberLeftForward = 3;

    public static final int kClimberRightForward = 4;// 4
    
  }
}

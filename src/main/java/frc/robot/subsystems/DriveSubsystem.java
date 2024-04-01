// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.PhotonCam;
import frc.utils.SwerveUtils;

public class DriveSubsystem extends SubsystemBase {

  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
  private double m_controllerXY = 1;
  private boolean m_first = true;
  // private double m_controllerYreq = 0;
  // private double m_controllerXreq = 0;

  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.001;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  private HolonomicPathFollowerConfig m_driveConfig = new HolonomicPathFollowerConfig(
      new PIDConstants(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI,
          ModuleConstants.kDrivingD),
      new PIDConstants(ModuleConstants.kTurningP, ModuleConstants.kTurningI,
          ModuleConstants.kDrivingD),
      AutoConstants.kMaxSpeedMetersPerSecond,
      DriveConstants.kDrivePlatformRadius,
      new ReplanningConfig(),
      0.02);
  private SwerveDrivePoseEstimator m_estimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
      new Rotation2d(Units.degreesToRadians(getGyroYawDeg())), getSwerveModulePositions(), getPose());
  private Field2d m_field = new Field2d();
  public DriveSubsystem() {

    configureHolonomicAutoBuilder();

  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    // System.out.println(m_estimator.getEstimatedPosition()+ " Estimated Pose");

    // CHANGE
    if (m_first) {
      m_first = false;
      return new Pose2d(0, 0, new Rotation2d(0));
    }
    return m_estimator.getEstimatedPosition();
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    };
  }

  public Pose2d getAutoPoseReversed() {
    if (m_first) {
      m_first = false;
      return new Pose2d(0, 0, new Rotation2d(0));
    }
    double autoPoseY = m_estimator.getEstimatedPosition().getY();
    double autoPoseX = m_estimator.getEstimatedPosition().getX() * -1;
    return new Pose2d(autoPoseX, autoPoseY,
        m_estimator.getEstimatedPosition().getRotation());

  }

  public Rotation2d currentRotation2d() {
    return m_gyro.getRotation2d();
  }

  public void resetOdometry(Pose2d pose) {
    System.out.println(" Reset Odom");
    m_estimator.resetPosition(new Rotation2d(), getSwerveModulePositions(), pose);

  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

    double xSpeedCommanded;
    double ySpeedCommanded;
    // m_controllerYreq = xSpeed;
    // m_controllerXreq = ySpeed;
    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral
      // acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        } else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      } else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(getGyroYawDeg()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void alignWheels() {
    m_frontRight.setDesiredState(new SwerveModuleState(0.01, Rotation2d.fromDegrees(0)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0.01, Rotation2d.fromDegrees(0)));
    m_rearRight.setDesiredState(new SwerveModuleState(0.01, Rotation2d.fromDegrees(0)));
    m_frontLeft.setDesiredState(new SwerveModuleState(0.01, Rotation2d.fromDegrees(0)));
  }
  // public void alignWheelsStop() {
  // m_frontRight.setDesiredState(new SwerveModuleState(0.00,
  // Rotation2d.fromDegrees(0)));
  // m_rearLeft.setDesiredState(new SwerveModuleState(0.00,
  // Rotation2d.fromDegrees(0)));
  // m_rearRight.setDesiredState(new SwerveModuleState(0.0,
  // Rotation2d.fromDegrees(0)));
  // m_frontLeft.setDesiredState(new SwerveModuleState(0.0,
  // Rotation2d.fromDegrees(0)));
  // }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = {
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState(),
        m_frontLeft.getState()
    };
    return states;

  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    ChassisSpeeds targetspeeds = ChassisSpeeds.discretize(speeds, DriveConstants.kAutoTimeDtSecondsAdjust);

    SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetspeeds);
    setModuleStates(targetStates);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  public void setGyro(double angle) {
    m_gyro.setAngleAdjustment(angle);
  }

  public double pitchAdjustVelocity() {
    if (m_gyro.getRoll() + 3 > 6) {
      return (m_gyro.getRoll() + 3) * 0.01;
    } else if (m_gyro.getRoll() + 3 < -6) {
      return (m_gyro.getRoll() + 3) * 0.01;
    } else {
      return 0;
    }
  }

  public ChassisSpeeds getChassisSpeeds() {
    ChassisSpeeds chassisspeed = DriveConstants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
    return chassisspeed;
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  public void flipHeading() {
    m_gyro.setAngleAdjustment(180);
  }

  public double getHeading() {
    return Rotation2d.fromDegrees(getGyroYawDeg()).getDegrees();
  }

  public double getTurnRate() {
    return getGyroYawDeg() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public double getGyroYawDeg() {
    return (m_gyro.getYaw()) * -1;
    // return (m_gyro.getYaw());
  }

  public double getEncoderMeters() {
    return m_frontRight.getPosition().distanceMeters;
  }

  public void controllerXYUpdate(double val) {
    m_controllerXY = val;
  }

  public void stopAllModules() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0,
        m_frontLeft.getState().angle));
    m_frontRight.setDesiredState(new SwerveModuleState(0,
        m_frontRight.getState().angle));
    m_rearLeft.setDesiredState(new SwerveModuleState(0,
        m_rearLeft.getState().angle));
    m_rearRight.setDesiredState(new SwerveModuleState(0,
        m_rearRight.getState().angle));
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

  public double distToSpeaker(Pose2d pose) {
    if (getAlliance() == true) { // ie, "red"
      return pose.getTranslation().getDistance(
          new Translation2d(DriveConstants.kShootingPoseSpeakerRED.getX(),
              DriveConstants.kShootingPoseSpeakerRED.getY()));
    } else { // ie, "blue"}
      return pose.getTranslation().getDistance(
          new Translation2d(DriveConstants.kShootingPoseSpeakerBLUE.getX(),
              DriveConstants.kShootingPoseSpeakerBLUE.getY()));
    }
  }

  // public double distToAmpORIG(Pose2d pose) {
  //   // Translation2d AmpPose = DriveConstants.kShootingPoseAmpBLUE.getTranslation();

  //   if (getAlliance() == true) { // ie, "red"
  //     return pose.getTranslation().getDistance(
  //         new Translation2d(DriveConstants.kShootingPoseAmpRED.getX(),
  //             DriveConstants.kShootingPoseSpeakerRED.getY()));
  //   } else { // ie, "blue"}
  //     return pose.getTranslation().getDistance(
  //         new Translation2d(DriveConstants.kShootingPoseAmpBLUE.getX(),
  //             DriveConstants.kShootingPoseSpeakerBLUE.getY()));
  //   }
  // }

  public double distToAmp(Pose2d pose) {
    // Translation2d AmpPose = DriveConstants.kShootingPoseAmpBLUE.getTranslation();

    if (getAlliance() == true) {// ie, "red"
      // System.out.println("RED AMP DIST");
      return pose.getTranslation().getDistance(
          PhotonCam.get().m_aprilTagLayout.getTagPose(5).get().toPose2d().getTranslation());

    } else { // ie, "blue"}
      // System.out.println("BLUE AMP DIST");
      return pose.getTranslation().getDistance(
          PhotonCam.get().m_aprilTagLayout.getTagPose(6).get().toPose2d().getTranslation());
    
      // return pose.getTranslation().getDistance(
      //     DriveConstants.kShootingPoseAmpBLUE.getTranslation());
    }
  }

  @Override
  public void periodic() {

    if (m_first) {
      System.out.println("Alliance Apriltag (red side = true):  " + getAlliance());
      PhotonCam.get().setAlliance(getAlliance());

    }

    Pose2d beforeCamAdded = m_estimator.update(currentRotation2d(), getSwerveModulePositions());

    PhotonCam.get().estimatePose(m_estimator);

    m_field.setRobotPose(beforeCamAdded);
    SmartDashboard.putNumber("Camera Adjustment",
        
            beforeCamAdded.getTranslation().getDistance(new Translation2d(getPose().getX(), getPose().getY())));

    SmartDashboard.putNumber("TURNING Encoder Position", m_frontRight.getTurningEncoder());
    SmartDashboard.putNumber("Angle Position", getGyroYawDeg());
    SmartDashboard.putNumber("DRIVING Encoder Position", getEncoderMeters());
    SmartDashboard.putString("CURRENT CHASSIS SPEED", getChassisSpeeds().toString());
    SmartDashboard.putNumber("CURRENT SPEED", m_frontRight.getCurrentSpeed());
    SmartDashboard.putString("CURRENT POSE", getPose().toString());
    SmartDashboard.putNumber("CURRENT ROLL", m_gyro.getRoll());
    SmartDashboard.putNumber("Requested Speed", m_controllerXY * DriveConstants.kMaxSpeedMetersPerSecond);

    // MRG -- Consider adding code below
    SmartDashboard.putNumber("Distance to Amp", distToAmp(getPose()));
    SmartDashboard.putNumber("Distance to Speaker", distToSpeaker(getPose()));
    SmartDashboard.putData(m_field);
    SmartDashboard.putBoolean("Alliance True = Red", getAlliance());
    ///////////
  }

  public void configureHolonomicAutoBuilder() {
    AutoBuilder.configureHolonomic(
        this::getPose, // getPose,
        this::resetOdometry,
        this::getChassisSpeeds,
        this::driveRobotRelative,
        m_driveConfig,
        this::getAlliance,
        this);
  }

}

// public Pose2d getAutoPose() {

// // double autoPoseY = m_odometry.getPoseMeters().getY();
// // double autoPoseX = m_odometry.getPoseMeters().getX();
// // return new Pose2d(autoPoseX, autoPoseY,
// // m_odometry.getPoseMeters().getRotation());
// // CHANGE
// return m_estimator.getEstimatedPosition();
// }

// private static void desaturateWheelSpeeds(
// SwerveModuleState state, double attainableMaxSpeedMetersPerSecond) {
// double realMaxSpeed = state.speedMetersPerSecond;
// if (realMaxSpeed > attainableMaxSpeedMetersPerSecond) {

// state.speedMetersPerSecond = state.speedMetersPerSecond / realMaxSpeed
// * attainableMaxSpeedMetersPerSecond;

// }
// }

// public void setFrontLeftModuleState(SwerveModuleState state) {
// desaturateWheelSpeeds(
// state, DriveConstants.kMaxSpeedMetersPerSecond);
// m_frontLeft.setDesiredState(state);
// }

// public void setRearRightModuleState(SwerveModuleState state) {
// desaturateWheelSpeeds(
// state, DriveConstants.kMaxSpeedMetersPerSecond);
// m_rearRight.setDesiredState(state);
// }

// public void setRearLeftModuleState(SwerveModuleState state) {
// desaturateWheelSpeeds(
// state, DriveConstants.kMaxSpeedMetersPerSecond);
// m_rearLeft.setDesiredState(state);

// }

// public void setFrontRightModuleState(SwerveModuleState state) {
// desaturateWheelSpeeds(
// state, DriveConstants.kMaxSpeedMetersPerSecond);
// m_frontRight.setDesiredState(state);

// }

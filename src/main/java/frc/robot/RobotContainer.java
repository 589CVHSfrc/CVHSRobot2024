// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.COMMAND_ARM.ArmStow;
import frc.robot.commands.COMMAND_ARM.BrakeArm;
import frc.robot.commands.COMMAND_ARM.BrakeArmRelease;
import frc.robot.commands.COMMAND_CLIMBER.MoveLeftClimber;
import frc.robot.commands.COMMAND_CLIMBER.MoveRightClimber;
import frc.robot.commands.COMMAND_DRIVE.DefaultDrive;
import frc.robot.commands.COMMAND_DRIVE.DrivePose;
import frc.robot.commands.COMMAND_DRIVE.ResetGyro;
import frc.robot.commands.COMMAND_DRIVE.Turn180;
import frc.robot.commands.COMMAND_SEQUENCE.ExtendClimbers;
import frc.robot.commands.COMMAND_SEQUENCE.HomeClimbers;
import frc.robot.commands.COMMAND_SEQUENCE.IntakeArmFloor;
import frc.robot.commands.COMMAND_SHOOTER.Intake;
import frc.robot.commands.COMMAND_SHOOTER.RevUpMotors;
import frc.robot.commands.COMMAND_SHOOTER.Shoot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GatewaySubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
        private final static DriveSubsystem m_robotDrive = new DriveSubsystem();
        private final static ArmSubsystem m_robotArm = new ArmSubsystem();
        private final static ShooterSubsystem m_robotShooter = new ShooterSubsystem();
        private final static GatewaySubsystem m_robotGateway = new GatewaySubsystem();
        private final static ClimberSubsystem m_robotClimber = new ClimberSubsystem();

        private final static XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
        private final static GenericHID m_coDriverSwitchBoard = new GenericHID(OIConstants.kCODriverControllerPort);
        private final static GenericHID m_coDriverJoystick = new GenericHID(3);

        // private SendableChooser<Command> m_autoChooser = new SendableChooser<>();

        public RobotContainer() {

                // NamedCommands.registerCommand("new pickup", new Pickup());

                // SmartDashboard.putData(m_autoChooser);

                configureButtonBindings();
                // m_autoChooser.setDefaultOption("Donuts", new PathPlannerAuto("Donuts"));
                // m_autoChooser.addOption("POS NEG TEST AUTO", new
                // PathPlannerAuto("PosNegTestAuto"));
                // m_autoChooser.addOption("5 Note Auto V1", new PathPlannerAuto("5 Note Auto
                // V1"));
                // m_autoChooser.addOption("5 Note Auto V2", new PathPlannerAuto("5 Note Auto
                // V2"));

                // m_robotArm.setDefaultCommand(new MoveArmSpeed(m_robotArm, () ->
                // m_testjoystick1.getRawAxis(1)*.1));

                m_robotDrive.setDefaultCommand(new DefaultDrive(m_robotDrive,
                                () -> -MathUtil.applyDeadband(m_driverController.getLeftY(),
                                                OIConstants.kDriveDeadband),
                                () -> -MathUtil.applyDeadband(m_driverController.getLeftX(),
                                                OIConstants.kDriveDeadband),
                                () -> -MathUtil.applyDeadband(m_driverController.getRawAxis(2) * .75,
                                                OIConstants.kDriveDeadband)));

                // // m_robotDrive.setDefaultCommand(new DefaultDrive(
                // m_robotDrive,
                // () -> {
                // double rawY = m_driverController.getLeftY();
                // double squaredY = Math.copySign(rawY * rawY, rawY);
                // return MathUtil.applyDeadband(squaredY, OIConstants.kDriveDeadband)*-1;
                // },
                // () -> {
                // double rawX = m_driverController.getLeftX();
                // double squaredX = Math.copySign(rawX * rawX, rawX);
                // return MathUtil.applyDeadband(squaredX, OIConstants.kDriveDeadband)*-1;
                // },
                // () -> -MathUtil.applyDeadband(m_driverController.getRawAxis(2),
                // OIConstants.kDriveDeadband)));

        }

        private void configureButtonBindings() {
                // ===================================DEBUG/DRIVE=========================================

                // RESET GYRO
                // new JoystickButton(m_driverController, 4)
                // .toggleOnTrue(new ResetGyro(m_robotDrive));

                // // RESET DRIVE MOTOR ENCODERS
                new JoystickButton(m_driverController, 4)
                                .whileTrue(new RunCommand(
                                                () -> m_robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d()))).alongWith(new ResetGyro(m_robotDrive)));

                new JoystickButton(m_driverController, 3)
                                .toggleOnTrue(new Turn180(m_robotDrive));

                // // DRIVE TO SPEAKER
                new JoystickButton(m_driverController, 2)
                                .whileTrue(new DrivePose(m_robotDrive).driveShootSpeaker());

                // (DRIVE + AIM), THEN SHOOT AT AMP
                // new JoystickButton(m_driverController, 5)
                // .toggleOnTrue(new DriveAimShootAmp(m_robotDrive, m_robotArm,
                // m_robotShooter));

                // (DRIVE + AIM), THEN SHOOT AT SPEAKER
                // new JoystickButton(m_coDriverSwitchBoard, 6)
                // .toggleOnTrue(new DriveAimShootSpeaker(m_robotDrive, m_robotArm,
                // m_robotShooter));

                // ===================================CLIMBER=========================================

                // EXTEND CLIMBERS
                new JoystickButton(m_coDriverSwitchBoard, 1)
                                .whileTrue(new ExtendClimbers(m_robotClimber));

                // EXTEND LEFT
                new JoystickButton(m_coDriverJoystick, 3)
                                .whileTrue(new MoveLeftClimber(m_robotClimber,
                                                () -> -ClimberConstants.kArmRaisingSpeed));
                // EXTEND RIGHT
                new JoystickButton(m_coDriverJoystick, 4)
                                .whileTrue(new MoveRightClimber(m_robotClimber,
                                                () -> ClimberConstants.kArmRaisingSpeed));

                // HOME CLIMBERS
                new JoystickButton(m_coDriverSwitchBoard, 2)
                                .toggleOnTrue(new HomeClimbers(m_robotClimber));

                // HOME LEFT
                new JoystickButton(m_coDriverJoystick, 5)
                                .toggleOnTrue(new MoveLeftClimber(m_robotClimber,
                                                () -> ClimberConstants.kLoweringClimbingSpeed));
                // HOME RIGHT
                new JoystickButton(m_coDriverJoystick, 6)
                                .toggleOnTrue(new MoveRightClimber(m_robotClimber,
                                                () -> -ClimberConstants.kLoweringClimbingSpeed));

                // ===================================SHOOTER/INTAKE==========================================
                // DOWN INTAKE STOW BRAKE
                new JoystickButton(m_coDriverSwitchBoard, 9)
                                .toggleOnTrue(new IntakeArmFloor(
                                                m_robotArm, m_robotShooter, m_robotGateway));
                // INTAKE SOURCE
                new JoystickButton(m_coDriverSwitchBoard, 5)
                                .toggleOnTrue(new Intake(m_robotShooter, m_robotGateway));

                // STOW ARM
                new JoystickButton(m_coDriverSwitchBoard, 6)
                                .toggleOnTrue(new ArmStow(m_robotArm, () -> ArmConstants.kStowSpeed));

                // RUN GATEWAYS
                new JoystickButton(m_coDriverSwitchBoard, 7)
                                .whileTrue(new Shoot(m_robotGateway));

                // SHOOT SPEAKER
                new JoystickButton(m_coDriverSwitchBoard, 8)
                                .whileTrue(new RevUpMotors(m_robotShooter));

                // SHOOT AMP
                new JoystickButton(m_coDriverSwitchBoard, 10)
                                .whileTrue(new RevUpMotors(m_robotShooter));
                // ===================================ARM=====================================================

                // BRAKE ARM
                new JoystickButton(m_coDriverSwitchBoard, 3)
                                .whileTrue(new BrakeArm(m_robotArm));
                // RELEASE ARM BRAKE
                new JoystickButton(m_coDriverSwitchBoard, 4)
                                .whileTrue(new BrakeArmRelease(m_robotArm));

                // THESE WORK=========================

                // // // RELEASE LEFT
                // new JoystickButton(m_coDriverSwitchBoard, 5)
                // .whileTrue(new ClimberSolenidTestLeft(m_robotClimber, true));
                // // LOCK LEFT
                // new JoystickButton(m_coDriverSwitchBoard, 3)
                // .whileTrue(new ClimberSolenidTestLeft(m_robotClimber, false));

                // // RELEASE RIGHT

                // new JoystickButton(m_coDriverSwitchBoard, 6)
                // .toggleOnTrue(new ClimberSolenidTestRight(m_robotClimber, true));
                // // LOCK RIGHT

                // new JoystickButton(m_coDriverSwitchBoard, 4)
                // .toggleOnTrue(new ClimberSolenidTestRight(m_robotClimber, false));
                // THESE WORK=========================

                // // new JoystickButton(m_testjoystick1, 7)
                // // .toggleOnTrue(new ShootDial(m_robotShooter, () ->
                // // m_testjoystick1.getRawAxis(3)*-1,
                // // () -> -m_testjoystick2.getRawAxis(3)));

        }

        public Command getAutonomousCommand() {
                return null;

                // try {

                // Pose2d startingpose = PathPlannerAuto
                // .getStaringPoseFromAutoFile(m_autoChooser.getSelected().getName());
                // m_robotDrive.resetOdometry(startingpose);
                // System.out.print("====================STARTING POSE: " + startingpose +
                // "====================");
                // return m_autoChooser.getSelected();

                // } catch (RuntimeException e) {
                // System.out.print("==================" + e);
                // System.out.print("COULD NOT FIND AUTO WITH SELECTED NAME"
                // + m_autoChooser.getSelected().getName());
                // return new WaitCommand(1);
                // }

        }

}
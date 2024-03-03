// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.COMMAND_ARM.BrakeArm;
import frc.robot.commands.COMMAND_ARM.BrakeArmRelease;
import frc.robot.commands.COMMAND_ARM.MoveArmJoystick;
import frc.robot.commands.COMMAND_ARM.MoveArmSpeed;
import frc.robot.commands.COMMAND_DRIVE.DefaultDrive;
import frc.robot.commands.COMMAND_DRIVE.DrivePose;
import frc.robot.commands.COMMAND_DRIVE.ResetGyro;
import frc.robot.commands.COMMAND_PARALLEL.IntakeDown;
import frc.robot.commands.COMMAND_SHOOTER.Intake;
import frc.robot.commands.COMMAND_SHOOTER.RevUpMotors;
import frc.robot.commands.COMMAND_SHOOTER.Shoot;
import frc.robot.commands.COMMAND_TESTING.ClimberSolenidTestLeft;
import frc.robot.commands.COMMAND_TESTING.ClimberSolenidTestRight;
import frc.robot.commands.COMMAND_TESTING.GatewayTest;
import frc.robot.commands.COMMAND_TESTING.ShootSmartDashboard;
import frc.robot.commands.COMMAND_TESTING.TestClimber;
import frc.robot.commands.COMMAND_TESTING.TestClimber2;
import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
    private final static DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final static ArmSubsystem m_robotArm = new ArmSubsystem();
    private final static ShooterSubsystem m_robotShooter = new ShooterSubsystem();
    private final static ClimberSubsystem m_robotClimb = new ClimberSubsystem();

    private final static XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    // private final static GenericHIdD m_coDriverSwitchBoard = new
    // GenericHID(OIConstants.kCODriverControllerPort);
    private final static GenericHID m_testjoystick1 = new GenericHID(3);
    private final static GenericHID m_testjoystick2 = new GenericHID(4);

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

        m_robotArm.setDefaultCommand(new MoveArmJoystick(m_robotArm, () -> m_testjoystick1.getRawAxis(1)));

        m_robotDrive.setDefaultCommand(new DefaultDrive(m_robotDrive,
                () -> -MathUtil.applyDeadband(m_driverController.getLeftY(),
                        OIConstants.kDriveDeadband),
                () -> -MathUtil.applyDeadband(m_driverController.getLeftX(),
                        OIConstants.kDriveDeadband),
                () -> -MathUtil.applyDeadband(m_driverController.getRawAxis(2),
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
        new JoystickButton(m_driverController, 4)
        .toggleOnTrue(new ResetGyro(m_robotDrive));

        // // RESET DRIVE MOTOR ENCODERS
        new JoystickButton(m_driverController, 3)
        .toggleOnTrue(new RunCommand(
        () -> m_robotDrive.resetOdometry(new Pose2d())));

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

        // new JoystickButton(m_testjoystick1, 5)
        // .whileTrue(new TestClimber2(m_robotClimb, ()->.2));
        // new JoystickButton(m_testjoystick1, 3)
        // .whileTrue(new TestClimber2(m_robotClimb, ()->-.2));

        // new JoystickButton(m_testjoystick1, 6)
        // .whileTrue(new TestClimber(m_robotClimb, ()->.2));
        // new JoystickButton(m_testjoystick1, 4)
        // .whileTrue(new TestClimber(m_robotClimb, ()->-.2));

        // ===================================CLIMBER=========================================
        // // CLIMB - RAISE
        // new JoystickButton(m_coDriverSwitchBoard, 6)
        // .toggleOnTrue(new RaiseClimber(m_robotClimb));

        // // CLIMB - LOWER
        // new JoystickButton(m_coDriverSwitchBoard, 5)
        // .toggleOnTrue(new LowerClimber(m_robotClimb));
        // new JoystickButton(m_driverController, 1)
        // .toggleOnTrue(new MoveClimber(m_robotClimb));

        // ===================================SHOOTER/INTAKE==========================================

        // PID TESTING FOR SHOOTER TOP + LOW
        // new JoystickButton(m_testjoystick1, 2)
        // .toggleOnTrue(new ShootDial(
        // m_robotShooter,
        // () -> -m_testjoystick1.getRawAxis(3),
        // () -> -m_testjoystick2.getRawAxis(3)));

        // PID TESTING FOR SHOOTER TOP + LOW
        new JoystickButton(m_testjoystick1, 1)
                .toggleOnTrue(new MoveArmSpeed(m_robotArm, ()->-0.2));
        
        // Automate Intake
        new JoystickButton(m_testjoystick1, 2)
                .toggleOnTrue(new IntakeDown(m_robotArm, m_robotShooter));

        // new JoystickButton(m_coDriverSwitchBoard, 9)
        // .toggleOnTrue(new Intake(m_robotShooter));

        // new JoystickButton(m_coDriverSwitchBoard, 10)
        // .toggleOnTrue(new Shoot(m_robotShooter));

        new JoystickButton(m_testjoystick1, 9)
                .toggleOnTrue(new Intake(m_robotShooter));

        new JoystickButton(m_testjoystick1, 10)
                .whileTrue(new Shoot(m_robotShooter));
        new JoystickButton(m_testjoystick1, 11)
                .whileTrue(new RevUpMotors(m_robotShooter));
        // ===================================ARM=====================================================

        // new JoystickButton(m_testjoystick1, 4)
        // .toggleOnTrue(new AimAngle(m_robotArm, 2));

        // RELEASE LEFT
        new JoystickButton(m_testjoystick2, 5)
                .toggleOnTrue(new ClimberSolenidTestLeft(m_robotClimb, true));
        // LOCK LEFT
        new JoystickButton(m_testjoystick2, 3)
                .toggleOnTrue(new ClimberSolenidTestLeft(m_robotClimb, false));

        // RELEASE RIGHT

        new JoystickButton(m_testjoystick2, 6)
                .toggleOnTrue(new ClimberSolenidTestRight(m_robotClimb, true));
        // LOCK RIGHT

        new JoystickButton(m_testjoystick2, 4)
                .toggleOnTrue(new ClimberSolenidTestRight(m_robotClimb, false));

        // // new JoystickButton(m_testjoystick1, 7)
        // // .toggleOnTrue(new ShootDial(m_robotShooter, () ->
        // // m_testjoystick1.getRawAxis(3)*-1,
        // // () -> -m_testjoystick2.getRawAxis(3)));

        new JoystickButton(m_testjoystick2, 10)
                .whileTrue(new TestClimber(m_robotClimb, () -> -.60));

        new JoystickButton(m_testjoystick2, 11)
                .whileTrue(new TestClimber(m_robotClimb, () -> .10));

        new JoystickButton(m_testjoystick2, 12)
                .whileTrue(new TestClimber2(m_robotClimb, () -> -.10));

        new JoystickButton(m_testjoystick2, 9)
                .whileTrue(new TestClimber2(m_robotClimb, () -> .60));

        new JoystickButton(m_testjoystick1, 3)
                .whileTrue(new BrakeArm(m_robotArm));
        new JoystickButton(m_testjoystick1, 4)
                .whileTrue(new BrakeArmRelease(m_robotArm));

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
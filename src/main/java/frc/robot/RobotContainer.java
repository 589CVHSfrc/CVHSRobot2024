// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
// import frc.robot.commands.CLIMBER.LowerArms;
import frc.robot.commands.DRIVE.DefaultDrive;
import frc.robot.commands.DRIVE.PIDTest;
import frc.robot.commands.DRIVE.DrivePose;
import frc.robot.commands.DRIVE.Pickup;
import frc.robot.commands.DRIVE.ResetGyro;
import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.DriveUtils;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems
        private final static DriveSubsystem m_robotDrive = new DriveSubsystem();
        private final static ArmSubsystem m_robotArm = new ArmSubsystem();
        // private final static ClimberSubsystem m_Climber = new ClimberSubsystem();
        // The driver's controller
        XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
        // XboxController m_codriverController = new
        // XboxController(OIConstants.kCODriverControllerPort);
        private SendableChooser<Command> m_autoChooser = new SendableChooser<>();
        private final Pose2d m_zero = new Pose2d();

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                // ArmSubsystem m_arm = new ArmSubsystem();
                // Pickup m_pickup = new Pickup();
                // NamedCommands.registerCommand(" pickup method", );
                NamedCommands.registerCommand("new pickup", new Pickup());

                NamedCommands.registerCommand("last pickup", new Pickup());
                // System.out.print("hi");

                // Command test = m_autoBuilder.fullAuto(new PathPlannerTrajectory());

                // m_autoChooser.addOption("Score TAXIIII", new ScoreTaxi(m_robotDrive,
                // m_robotSpin));
                // m_autoChooser.addOption("Score BALANCE", new ScoreBal(m_robotDrive,
                // m_robotSpin));
                // m_autoChooser.addOption("Score ONLY", new ScoreONLY(m_robotDrive,
                // m_robotSpin));
                // m_autoChooser.addOption("Score Backup Balance", new
                // ScoreBalance(m_robotDrive, m_robotSpin));
                // m_autoChooser.setDefaultOption("NOTHING", new NOTHING());

                SmartDashboard.putData(m_autoChooser);

                configureButtonBindings();
                // m_robotDrive.configureHolonomicAutoBuilder();
                m_autoChooser.setDefaultOption("Donuts", new PathPlannerAuto("Donuts"));
                m_autoChooser.addOption("POS NEG TEST AUTO", new PathPlannerAuto("PosNegTestAuto"));
                m_autoChooser.addOption("5 Note Auto V1", new PathPlannerAuto("5 Note Auto V1"));
                m_autoChooser.addOption("5 Note Auto V2", new PathPlannerAuto("5 Note Auto V2"));

                m_robotArm.setDefaultCommand(getAutonomousCommand());
                m_robotDrive.setDefaultCommand(new DefaultDrive(m_robotDrive,
                                () -> -MathUtil.applyDeadband(m_driverController.getLeftY(),
                                                OIConstants.kDriveDeadband),
                                () -> -MathUtil.applyDeadband(m_driverController.getLeftX(),
                                                OIConstants.kDriveDeadband),
                                () -> -MathUtil.applyDeadband(m_driverController.getRawAxis(2),
                                                OIConstants.kDriveDeadband)));
        }

        private void configureButtonBindings() {
                // whiletrue is when held toggle is toggle
                new JoystickButton(m_driverController, 4)
                                .toggleOnTrue(new ResetGyro(m_robotDrive));
                new JoystickButton(m_driverController, 3)
                                .toggleOnTrue(new RunCommand(
                                                () -> m_robotDrive.resetOdometry(m_zero)));
                // if (m_robotDrive.getAlliance()) {
                new JoystickButton(m_driverController, 2)
                                .whileTrue(new DrivePose(m_robotDrive).driveShoot());

                new JoystickButton(m_driverController, 4)
                                .whileTrue(new DrivePose(m_robotDrive).Shoot()); //this is me yapping, probably super inaccurate.
                // } else {
                // new JoystickButton(m_driverController, 2)
                // .whileTrue(new DriveUtils(m_robotDrive)
                // .driveToPose(DriveConstants.kShootingPoseBLUE));

                // }

        }

        public Command getAutonomousCommand() {

                try {

                        Pose2d startingpose = PathPlannerAuto
                                        .getStaringPoseFromAutoFile(m_autoChooser.getSelected().getName());
                        m_robotDrive.resetOdometry(startingpose);
                        System.out.print("====================POSE: " + startingpose + "==============");
                        // return new PathPlannerAuto("5 Note Auto V1");
                        // return new PathPlannerAuto("Testing");
                        return m_autoChooser.getSelected();

                } catch (RuntimeException e) {
                        System.out.print("==================" + e);
                        System.out.print("===COULD NOT FIND AUTO WITH SELECTED NAME===");
                        return new WaitCommand(1);
                }

        }

}

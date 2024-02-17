// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.OTHER;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ARMS.ShootAimSpeaker;
import frc.robot.commands.DRIVE.DrivePose;
import frc.robot.commands.SHOOTER.Shoot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveAimShoot extends SequentialCommandGroup {
  /** Creates a new DriveAimShoot. */

  public DriveAimShoot(DriveSubsystem robotDrive, ArmSubsystem arm, ShooterSubsystem shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new DrivePose(robotDrive).driveShoot().alongWith(new ShootAimSpeaker(arm)), new Shoot(shooter));
    
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_SEQUENCE;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.COMMAND_ARM.AimSpeaker;
import frc.robot.commands.COMMAND_DRIVE.DrivePose;
import frc.robot.commands.COMMAND_SHOOTER.RevUpMotors;
import frc.robot.commands.COMMAND_SHOOTER.Shoot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class DriveAimShootSpeaker extends SequentialCommandGroup {

  public DriveAimShootSpeaker(DriveSubsystem robotDrive, ArmSubsystem arm, ShooterSubsystem shooter) {
    addCommands(

        new DrivePose(robotDrive).driveShootSpeaker()
        .alongWith(
          new AimSpeaker(arm))
        .alongWith(
          new RevUpMotors(shooter)),

        new Shoot(shooter));
  }
}

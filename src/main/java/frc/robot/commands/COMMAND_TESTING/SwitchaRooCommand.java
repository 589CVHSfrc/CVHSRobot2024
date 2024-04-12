// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_TESTING;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.DriveUtils;

public class SwitchaRooCommand extends Command {
  /** Creates a new WrapperCommand. */
  Command m_command;
  DriveSubsystem m_drive;

  public SwitchaRooCommand(DriveSubsystem drive) {
    m_drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_drive.getAlliance()) {
      // m_command = new PRINTRED();
      
      m_command = new DriveUtils(m_drive).driveToPose(DriveConstants.kShootingPoseAmpRED, () -> 1)
          .andThen(new DriveUtils(m_drive).driveToPose(DriveConstants.kShootingPoseAmpRED, () -> .1));

    } else {
      // m_command = new PRINTBLUE();

      m_command = new DriveUtils(m_drive).driveToPose(DriveConstants.kShootingPoseAmpBLUE, () -> 1)
          .andThen(new DriveUtils(m_drive).driveToPose(DriveConstants.kShootingPoseAmpBLUE, () -> .1));
    }
    m_command.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_command.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_command.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_command.isFinished();
  }
}

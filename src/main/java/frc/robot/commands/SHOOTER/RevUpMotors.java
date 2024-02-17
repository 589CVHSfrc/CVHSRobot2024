// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SHOOTER;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RevUpMotors extends Command {
  /** Creates a new RevUpMotors. */
  ShooterSubsystem m_rev = new ShooterSubsystem();
  ArmSubsystem m_armAimer = new ArmSubsystem();
  DriveSubsystem m_driveToPose = new DriveSubsystem();

  public RevUpMotors(ShooterSubsystem rev) {
    m_rev = rev;
    addRequirements(m_rev);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_rev.shoot();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return m_rev.isRampedUp();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_DRIVE;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDrive extends Command {
  /** Creates a new DefaultDrive. */
  private DriveSubsystem m_drive;
  private DoubleSupplier m_xspeed;
  private DoubleSupplier m_yspeed;
  private DoubleSupplier m_rotspeed;
  public DefaultDrive(DriveSubsystem drive, DoubleSupplier joyleftx, DoubleSupplier joylefty, DoubleSupplier joyrightx) {
    m_drive = drive;
    m_xspeed = joyleftx;
    m_yspeed = joylefty;
    m_rotspeed = joyrightx;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.drive(m_xspeed.getAsDouble(), m_yspeed.getAsDouble(), m_rotspeed.getAsDouble(), true, true);
    m_drive.controllerXYUpdate((m_xspeed.getAsDouble()));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}

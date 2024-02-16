// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ARMS;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class AutoRestAngle extends Command {
  /** Creates a new autoRestAngle. */
  ArmSubsystem m_armSubsystem = new ArmSubsystem();
  public AutoRestAngle(ArmSubsystem arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armSubsystem = arm;
    addRequirements(m_armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armSubsystem.moveArm(ArmConstants.kRestingAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    m_armSubsystem.AngleReached();
    return false;
  }
}

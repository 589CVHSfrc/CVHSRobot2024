// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_CLIMBER;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class MoveClimber extends Command {
  /** Creates a new MoveClimber. */
  public ClimberSubsystem m_ClimberSubsystem;
  public MoveClimber(ClimberSubsystem climberSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ClimberSubsystem = climberSubsystem;
    addRequirements(m_ClimberSubsystem);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ClimberSubsystem.resetMaxAmps();
    m_ClimberSubsystem.startClimb();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ClimberSubsystem.climbingOrder();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ClimberSubsystem.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

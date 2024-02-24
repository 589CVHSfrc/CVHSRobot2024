// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_CLIMBER;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class LowerClimber extends Command {
  /** Creates a new LowerArms. */
  ClimberSubsystem m_climberSubsystem;
  public LowerClimber(ClimberSubsystem Climb) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climberSubsystem = Climb;
    addRequirements(m_climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climberSubsystem.startClimb();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climberSubsystem.climbingOrder();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climberSubsystem.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

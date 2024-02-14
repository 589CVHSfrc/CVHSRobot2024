// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CLIMBER;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class RaiseArms extends Command {
  /** Creates a new RaiseArms. */
  ClimberSubsystem m_Climber;
  public RaiseArms(ClimberSubsystem armsRaise) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Climber = armsRaise;
    addRequirements(m_Climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Climber.release();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Climber.raiseArms();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Climber.getSwitchStatus();
  }
}

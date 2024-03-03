// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_TESTING;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberSolenidTestRight extends Command {
  /** Creates a new ClimberSolenidTestLeft. */
  ClimberSubsystem m_climber;
  boolean m_release;
  public ClimberSolenidTestRight(ClimberSubsystem climber, boolean release) {
    m_climber = climber;
    m_release=release;
    addRequirements(m_climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_release){
      m_climber.releaseRight();
    }else{
      m_climber.brakeRight();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_CLIMBER;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class MoveLeftClimber extends Command {
  ClimberSubsystem m_climber;
  DoubleSupplier m_speed;

  public MoveLeftClimber(ClimberSubsystem climber, DoubleSupplier speed) {
    m_climber = climber;
    m_speed = speed;
  }

  @Override
  public void initialize() {
    m_climber.releaseLeft();
  }

  @Override
  public void execute() {
    m_climber.moveLeftClimber(m_speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_climber.moveLeftClimber(() -> 0);
    m_climber.brakeLeft();
  }

  @Override
  public boolean isFinished() {
    // return false;
    return m_climber.getLeftSwitchStatus();
  }
}

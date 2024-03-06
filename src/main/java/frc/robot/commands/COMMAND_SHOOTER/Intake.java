// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_SHOOTER;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GatewaySubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Intake extends Command {
  ShooterSubsystem m_shooter;
  GatewaySubsystem m_gate;

  public Intake(ShooterSubsystem shoot, GatewaySubsystem gate) {
    m_shooter = shoot;
    m_gate = gate;
    addRequirements(m_shooter, m_gate);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_shooter.intake();
    m_gate.intakeGateway();
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.stopShoot();
    m_gate.stopGateway();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

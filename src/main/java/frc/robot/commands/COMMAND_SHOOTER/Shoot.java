// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_SHOOTER;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GatewaySubsystem;

public class Shoot extends Command {
  GatewaySubsystem m_gate;
  public Shoot(GatewaySubsystem gate) {
    m_gate = gate;
    addRequirements(gate);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_gate.shootGateway();
  }

  @Override
  public void end(boolean interrupted) {
    m_gate.stopGateway();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

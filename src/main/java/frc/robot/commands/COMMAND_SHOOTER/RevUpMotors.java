// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_SHOOTER;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class RevUpMotors extends Command {
  ShooterSubsystem m_shooter;
  public RevUpMotors(ShooterSubsystem shooter) {
    m_shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    m_shooter.shoot();
  }

  @Override
  public void execute() {
    // m_shooter.shoot();
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.stopShoot();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

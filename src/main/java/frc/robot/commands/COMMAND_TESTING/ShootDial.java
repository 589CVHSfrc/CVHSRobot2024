// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_TESTING;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootDial extends Command {
  /** Creates a new ShootDial. */
  ShooterSubsystem m_shooter;
  DoubleSupplier m_leftDial;
  DoubleSupplier m_rightDial;

  public ShootDial(ShooterSubsystem shoot, DoubleSupplier left, DoubleSupplier right){
    m_shooter = shoot;
    m_leftDial = left;
    m_rightDial = right;
    addRequirements(m_shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      // m_shooter.shootLow(1, m_leftDial.getAsDouble());
      // m_shooter.shootTop(1, m_rightDial.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopShoot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

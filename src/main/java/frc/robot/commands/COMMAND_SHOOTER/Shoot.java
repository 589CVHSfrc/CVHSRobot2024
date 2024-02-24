// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_SHOOTER;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends Command {
  /** Creates a new Shoot. */
  ShooterSubsystem m_shooter = new ShooterSubsystem();
  // Timer m_timer = new Timer();

  public Shoot(ShooterSubsystem shooter) {
    m_shooter = shooter;
    addRequirements();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_shooter.isRampedUp()) {
      m_shooter.spinGateway(); // change the boolean value later.
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.slowDownToZero();
    m_shooter.stopGateway(); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (m_timer.hasElapsed(ShooterConstants.kShooterTime)) {
    //   return true;
    // }
    return false;
  }
}

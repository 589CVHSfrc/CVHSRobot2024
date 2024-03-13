// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_SHOOTER;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GatewaySubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class TimedRevShooterAMP extends Command {
  Timer m_timer;
  ShooterSubsystem m_shooter;
  GatewaySubsystem m_gate;
  public TimedRevShooterAMP(ShooterSubsystem shooter, GatewaySubsystem gate) {
    m_timer = new Timer();
    m_gate = gate;
    m_shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {

    m_timer.reset();
    m_timer.start();

  }

  @Override
  public void execute() {

    m_shooter.shootAmp();
    if(m_timer.hasElapsed(1)){
      m_gate.shootGateway();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.stopShoot();
    m_timer.reset();
    m_gate.stopGateway();

  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(1.6);
  }
}

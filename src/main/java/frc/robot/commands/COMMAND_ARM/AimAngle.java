// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_ARM;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class AimAngle extends Command {
  /** Creates a new ShootAimAmp. */
  ArmSubsystem m_arm = new ArmSubsystem();

  public AimAngle(ArmSubsystem arm, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = arm;
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.moveArm(ArmConstants.kShootingAngleAmp);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.stopArm();
    m_arm.armBrake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_arm.AngleReached();
  }
}

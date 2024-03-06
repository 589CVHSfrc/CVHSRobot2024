// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_ARM;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmFloor extends Command {
  ArmSubsystem m_arm;
  DoubleSupplier m_speed;

  public ArmFloor(ArmSubsystem arm, DoubleSupplier speed) {
    m_arm = arm;
    m_speed = speed;
    addRequirements(m_arm);
  }

  @Override
  public void initialize() {
    m_arm.armRelease();
  }

  @Override
  public void execute() {
    m_arm.moveArmSpeed(m_speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_arm.stopArm();
    // m_arm.armBrake();
  }

  @Override
  public boolean isFinished() {
    
    return m_arm.isForwardLimitReached();
  }
}

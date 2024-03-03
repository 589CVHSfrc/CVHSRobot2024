// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_PARALLEL;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.COMMAND_ARM.BrakeArmRelease;
import frc.robot.commands.COMMAND_ARM.MoveArmSpeed;
import frc.robot.commands.COMMAND_SHOOTER.Intake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeDown extends ParallelCommandGroup {
  /** Creates a new IntakeDown. */
  ArmSubsystem m_ArmSubsystem;
  ShooterSubsystem m_ShooterSubsystem;
  public IntakeDown(ArmSubsystem arm, ShooterSubsystem shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_ArmSubsystem = arm;
    m_ShooterSubsystem = shooter;
    addCommands(
      //new BrakeArmRelease(m_ArmSubsystem),
      new MoveArmSpeed(m_ArmSubsystem, ()->.1),
      new Intake(m_ShooterSubsystem)
    );
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_SEQUENCE;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.COMMAND_ARM.ArmStow;
import frc.robot.commands.COMMAND_SHOOTER.IntakeLimitSwitch;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GatewaySubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeArmSource extends SequentialCommandGroup {
  public IntakeArmSource(ArmSubsystem arm, ShooterSubsystem shooter, GatewaySubsystem gate) {
    addCommands(

        new ArmStow(arm, () -> .1).alongWith(new IntakeLimitSwitch(shooter, gate))

    );
  }
}

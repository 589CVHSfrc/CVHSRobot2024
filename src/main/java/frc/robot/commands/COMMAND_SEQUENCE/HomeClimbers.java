// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_SEQUENCE;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ClimberConstants;
import frc.robot.commands.COMMAND_CLIMBER.MoveLeftClimber;
import frc.robot.commands.COMMAND_CLIMBER.MoveRightClimber;
import frc.robot.subsystems.ClimberSubsystem;

public class HomeClimbers extends ParallelCommandGroup {
  public HomeClimbers(ClimberSubsystem climber) {
    addCommands(
        new MoveRightClimber(climber, () -> -ClimberConstants.kLoweringClimbingSpeed),
        new MoveLeftClimber(climber, () -> ClimberConstants.kLoweringClimbingSpeed)

    );
  }
}

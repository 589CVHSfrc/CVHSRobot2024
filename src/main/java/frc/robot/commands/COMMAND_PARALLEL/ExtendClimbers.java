// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_PARALLEL;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ClimberConstants;
import frc.robot.commands.COMMAND_CLIMBER.MoveLeftClimber;
import frc.robot.commands.COMMAND_CLIMBER.MoveRightClimber;
import frc.robot.subsystems.ClimberSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ExtendClimbers extends ParallelCommandGroup {
  public ExtendClimbers(ClimberSubsystem climber) {
    addCommands(
        new MoveRightClimber(climber, () -> ClimberConstants.kClimberRaisingSpeed),
        new MoveLeftClimber(climber, () -> -ClimberConstants.kClimberRaisingSpeed)

    );
  }
}

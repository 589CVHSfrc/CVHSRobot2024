// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_PARALLEL;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ClimberConstants;
import frc.robot.commands.COMMAND_CLIMBER.MoveLeftClimberHOLD;
import frc.robot.commands.COMMAND_CLIMBER.MoveRightClimberHOLD;
import frc.robot.subsystems.ClimberSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ExtendClimbersHOLD extends ParallelCommandGroup {
  public ExtendClimbersHOLD(ClimberSubsystem climber) {
    addCommands(
        new MoveRightClimberHOLD(climber, () -> ClimberConstants.kArmRaisingSpeed),
        new MoveLeftClimberHOLD(climber, () -> -ClimberConstants.kArmRaisingSpeed)

    );
  }
}

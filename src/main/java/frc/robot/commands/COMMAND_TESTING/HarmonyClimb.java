// package frc.robot.commands.COMMAND_TESTING;
// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_TESTING;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class HarmonyClimb extends Command {
    /** Creates a new LowerArms. */
    ClimberSubsystem m_climber;

    public HarmonyClimb(ClimberSubsystem Climb) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_climber = Climb;
        addRequirements(m_climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_climber.startClimb();
        m_climber.releaseLeft();
        m_climber.releaseRight();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_climber.climbingOrder();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_climber.brakeLeft();
        m_climber.brakeRight();
        m_climber.stopMotors();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_climber.getLeftSwitchStatus() || m_climber.getRightSwitchStatus();
    }
}

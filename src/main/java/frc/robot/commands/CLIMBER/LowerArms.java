// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.CLIMBER;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.ClimberSubsystem;

// public class LowerArms extends Command {
//   /** Creates a new LowerArms. */
//   ClimberSubsystem m_Climb;
//   public LowerArms(ClimberSubsystem Climb) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     m_Climb = Climb;
//     addRequirements(m_Climb);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     m_Climb.startClimb();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     m_Climb.leftClimb(m_Climb.ClimbingSide()[0]);
//     m_Climb.rightClimb(m_Climb.ClimbingSide()[1]);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }

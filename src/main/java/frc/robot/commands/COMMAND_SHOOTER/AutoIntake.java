// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.COMMAND_SHOOTER;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.ShooterConstants;
// import frc.robot.subsystems.ShooterSubsystem;

// public class AutoIntake extends Command {
//   /** Creates a new Intake. */
//   // ShooterSubsystem m_shooter = new ShooterSubsystem();
//   Timer m_timer = new Timer();
//   public AutoIntake(ShooterSubsystem shooter) {
//     m_shooter = shooter;
//     addRequirements(m_shooter);
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     m_timer.start();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     m_shooter.intake();
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     // m_shooter.slowDownToZero();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     if(m_timer.hasElapsed(ShooterConstants.kShooterTime)){ //change this seconds value
//       return true;
//     }
//     return false;
//   }
// }

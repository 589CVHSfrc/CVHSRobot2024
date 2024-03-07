// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.COMMAND_SEQUENCE;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.commands.COMMAND_SHOOTER.RevUpMotors;
// import frc.robot.commands.COMMAND_SHOOTER.Shoot;
// import frc.robot.commands.COMMAND_SHOOTER.TimedRevShooter;
// import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.subsystems.GatewaySubsystem;
// import frc.robot.subsystems.ShooterSubsystem;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class TimedShoot extends SequentialCommandGroup {
//   /** Creates a new TimedShoot. */
//   public TimedShoot(ArmSubsystem arm, ShooterSubsystem shooter, GatewaySubsystem gate) {
//     // Add your commands in the addCommands() call, e.g.
//     // addCommands(new FooCommand(), new BarCommand());
//     addCommands(
//       new TimedRevShooter(shooter),
//       new Shoot(gate)
//     );
//   }
// }

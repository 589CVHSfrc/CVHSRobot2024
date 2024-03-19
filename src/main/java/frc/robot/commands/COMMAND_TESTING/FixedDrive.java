// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_TESTING;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class FixedDrive extends Command {
  Timer m_timer;
  private DriveSubsystem m_drive;
  public FixedDrive(DriveSubsystem drive) {
    m_timer = new Timer();

    m_drive = drive;
    
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer = new Timer();

    m_drive.alignWheels();
    // m_drive.alignWheelsStop();

    m_timer.reset();
    m_timer.start();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_timer.hasElapsed(2)){
      m_drive.drive(.5, 0, 0, true, false);
    }
    }

  @Override
  public void end(boolean interrupted) {
        m_drive.drive(0, 0, 0, true, false);

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

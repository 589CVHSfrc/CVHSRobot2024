// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ARMS;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimeLight;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class Aim extends Command {
  /** Creates a new Aim. */
  ArmSubsystem m_arm = new ArmSubsystem();
  DriveSubsystem m_drive = new DriveSubsystem();
  double xDifference;
  public Aim(ArmSubsystem arm, DriveSubsystem drive) {
    m_arm = arm;
    m_drive = drive;
    addRequirements(m_drive);
    addRequirements(m_arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //true is red, else is blue
    // if(m_drive.getAlliance()){
    // xDifference = m_drive.getPose().getX() - DriveConstants.kShootingPoseRED.getX();
    // m_arm.moveArm(xDifference + ArmConstants.kShootingAngleAmp);
    // }
    // else{
    // xDifference = m_drive.getPose().getX() - DriveConstants.kShootingPoseBLUE.getX();
    // m_arm.moveArm(xDifference + ArmConstants.kShootingAngleAmp);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

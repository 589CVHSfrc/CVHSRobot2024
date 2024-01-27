// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ShooterConstants;

public class ClimberSubsystem extends SubsystemBase {
  private CANSparkMax m_leftMotor, m_rightMotor;
  private RelativeEncoder m_leftEncoder, m_rightEncoder;
  private double m_rightMotorPreviousAmps, m_leftMotorPreviousAmps;
  private double m_initialClimbTime;
  private boolean[] readySide;
  private boolean bstartupTime;
  private double m_previousTime;

  public ClimberSubsystem() {
   m_leftMotor = new CANSparkMax(ClimberConstants.kClimberLeftMotorCanID, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(ClimberConstants.kClimberRightMotorCanID, MotorType.kBrushless);
    m_rightEncoder = m_rightMotor.getEncoder();
    m_leftEncoder  = m_leftMotor.getEncoder();
    m_rightMotorPreviousAmps = 0;
    m_leftMotorPreviousAmps = 0;
    readySide = new boolean [2];
  }
//true is right, false is left.
  public boolean[] ClimbingSide(){

    double timeSinceStart = Timer.getFPGATimestamp() - m_initialClimbTime;
    double deltaTime = Timer.getFPGATimestamp() - m_previousTime;
    if ( timeSinceStart >= .25 ){
      bstartupTime = true;
    }
    double leftAmps = m_leftMotor.getOutputCurrent();
    double rightAmps = m_rightMotor.getOutputCurrent();

    double leftDifference = (leftAmps - m_leftMotorPreviousAmps)*deltaTime;
    double rightDifference = (rightAmps - m_rightMotorPreviousAmps)*deltaTime;

    m_rightMotorPreviousAmps = rightAmps;
    m_leftMotorPreviousAmps = leftAmps;

    if(bstartupTime){
      if(leftDifference>= ClimberConstants.kDifferenceInRate){
        readySide[0] = true;
      }

      if(rightDifference>= ClimberConstants.kDifferenceInRate){
        readySide[1] = true;
      }
    }
    m_previousTime = Timer.getFPGATimestamp();
    return readySide;
  }

  public void startClimb(){
    bstartupTime = false;
    m_previousTime = m_initialClimbTime = Timer.getFPGATimestamp();
    readySide[0] = false;
    readySide[1] = false;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

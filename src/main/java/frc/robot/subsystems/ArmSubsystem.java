// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private CANSparkMax m_angleMotor;
  private double m_desiredAngle;
  private AbsoluteEncoder m_armEncoder;
  private double m_bufferZoneTop;
  private double m_bufferZoneBottom;
  //top/bottom depends on hardware, can change later.
  private SparkLimitSwitch m_limitSwitchTop;
  private SparkLimitSwitch m_limitSwitchBottom; 

  public ArmSubsystem() {
    m_angleMotor = new CANSparkMax(ArmConstants.kAngleMotorCanID, MotorType.kBrushless);
    m_armEncoder = m_angleMotor.getAbsoluteEncoder(Type.kDutyCycle);
    m_desiredAngle = 0;
    m_bufferZoneTop = ArmConstants.kMaxArmAngle;
    m_bufferZoneBottom = ArmConstants.kMinAngle;
  }

  //CHANGE
  public double getAbsoluteAngle(){
    return m_armEncoder.getPosition();
  }

  public void moveArm(double desiredAngle){
      m_desiredAngle = desiredAngle;
      if((m_desiredAngle > m_bufferZoneTop && m_desiredAngle < m_bufferZoneBottom) || 
          (m_limitSwitchBottom.isPressed() || m_limitSwitchTop.isPressed())){
         m_angleMotor.set(0);
      }
      else{
        if(m_desiredAngle > getAbsoluteAngle()){
         m_angleMotor.set(ArmConstants.kRaisingSpeed);
        }
        else{
          m_angleMotor.set((ArmConstants.kRaisingSpeed)*-1);
        }
      }
  }

  public boolean AngleReached(){
    return getAbsoluteAngle() >= m_desiredAngle-5 && getAbsoluteAngle() <= m_desiredAngle+5;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

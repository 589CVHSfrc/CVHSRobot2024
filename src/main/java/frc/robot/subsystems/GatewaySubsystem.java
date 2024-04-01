// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class GatewaySubsystem extends SubsystemBase {
  private CANSparkMax m_gatewayMotor;
  private SparkLimitSwitch m_gatewayLimitSwitch;
  private SparkLimitSwitch m_gatewayReverseLimitSwitch;

  public GatewaySubsystem() {
    m_gatewayMotor = new CANSparkMax(ShooterConstants.kTopGatewayWheelMotorID, MotorType.kBrushless);
    m_gatewayMotor.setInverted(false);
    m_gatewayLimitSwitch = m_gatewayMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    m_gatewayLimitSwitch.enableLimitSwitch(true);
    m_gatewayReverseLimitSwitch = m_gatewayMotor.getReverseLimitSwitch(Type.kNormallyOpen);
    m_gatewayLimitSwitch.enableLimitSwitch(false);
    m_gatewayMotor.setIdleMode(IdleMode.kBrake);
    m_gatewayMotor.burnFlash();

    
  }

  public boolean isSwitchPressed() {
    return m_gatewayLimitSwitch.isPressed();
  }


  public void stopGateway() {
    m_gatewayMotor.set(0);
  }

  public void IDLEGateway() {
    m_gatewayMotor.set(0);
  }
  public void intakeGateway() {
    m_gatewayMotor.set(ShooterConstants.kGatewayMotorSpeed);
  }

  public void shootGateway() {
    //System.out.println("Setting");
    m_gatewayMotor.set(-ShooterConstants.kGatewayMotorSpeed);
  }

  @Override
  public void periodic() {
    
  }
}
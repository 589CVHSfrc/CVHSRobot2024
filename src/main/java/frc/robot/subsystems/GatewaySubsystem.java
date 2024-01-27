// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class GatewaySubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private CANSparkMax m_gatewayMotor;
  private RelativeEncoder m_gatewayEncoder;

  public GatewaySubsystem() {
    m_gatewayMotor = new CANSparkMax(ShooterConstants.kShooterMotorTopCanID, MotorType.kBrushless);
    m_gatewayEncoder = m_gatewayMotor.getEncoder();
  }

  public void shoot(boolean direction) {
      m_gatewayMotor.set(Constants.ShooterConstants.kShooterSpeed);
  }

  public void stopShoot() {
    m_gatewayMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

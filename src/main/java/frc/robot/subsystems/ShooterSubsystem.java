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

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private CANSparkMax m_topMotor;
  private CANSparkMax m_lowMotor;
  private RelativeEncoder m_topEncoder;
  private RelativeEncoder m_lowEncoder;

  public ShooterSubsystem() {
    m_topMotor = new CANSparkMax(ShooterConstants.kShooterMotorTopCanID, MotorType.kBrushless);
    m_lowMotor = new CANSparkMax(ShooterConstants.kShooterMotorLowCanID, MotorType.kBrushless);
    m_topEncoder = m_topMotor.getEncoder();
    m_lowEncoder = m_lowMotor.getEncoder();
    m_lowMotor.follow(m_topMotor);
    m_lowMotor.setInverted(true);
  }

  public void shoot(boolean direction) {
    if (direction) {
      m_topMotor.set(Constants.ShooterConstants.kShooterSpeed);

    } else {
      m_topMotor.set(-Constants.ShooterConstants.kShooterSpeed);
    }
  }

  public void stopShoot() {
    m_topMotor.set(0);
    m_lowMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

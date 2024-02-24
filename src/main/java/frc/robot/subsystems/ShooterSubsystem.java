// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private CANSparkMax m_topMotor;
  private CANSparkMax m_lowMotor;
  private CANSparkMax m_gatewayMotor;

  private SparkPIDController m_topMotorPidController;
  private SparkPIDController m_lowMotorPidController;
  private RelativeEncoder m_topEncoder;
  private RelativeEncoder m_lowEncoder;
  private RelativeEncoder m_gatewayEncoder;
  double cVelocity, cVelocity2;

  public ShooterSubsystem() {
    m_topMotor = new CANSparkMax(ShooterConstants.kShooterMotorTopCanID, MotorType.kBrushless);
    m_lowMotor = new CANSparkMax(ShooterConstants.kShooterMotorLowCanID, MotorType.kBrushless);

    m_gatewayMotor = new CANSparkMax(ShooterConstants.kLowGatewayWheelMotorID, MotorType.kBrushless);

    m_topEncoder = m_topMotor.getEncoder();
    m_lowEncoder = m_lowMotor.getEncoder();

    m_gatewayEncoder = m_gatewayMotor.getEncoder();

    m_lowMotor.follow(m_topMotor, true);
    cVelocity = 0;
    cVelocity2 = 0;
    // m_lowMotor.follow(m_topMotor);
    // m_lowMotor.setInverted(true);
  }

  public void shootSmartVelocity(double velocity) {
    m_topMotorPidController.setReference(velocity, ControlType.kSmartVelocity, 0, lookupFF(velocity),
        ArbFFUnits.kPercentOut);
    m_lowMotorPidController.setReference(velocity, ControlType.kSmartVelocity, 0, lookupFF(velocity),
        ArbFFUnits.kPercentOut);
  }

  public void shootSmartVelocity(double velocitytop, double velocitylow) {
    m_topMotorPidController.setReference(velocitytop, ControlType.kSmartVelocity, 0, lookupFF(velocitytop),
        ArbFFUnits.kPercentOut);
    m_lowMotorPidController.setReference(velocitylow, ControlType.kSmartVelocity, 0, lookupFF(velocitylow),
        ArbFFUnits.kPercentOut);
  }

  public void shoot() {
    shootSmartVelocity(ShooterConstants.kShooterSpeed);
  }

  public void intake() {
    shootSmartVelocity(-ShooterConstants.kIntakeSpeed);
  }

  public void stopShoot() {
    m_topMotor.set(0);
    m_lowMotor.set(0);
  }

  public void stopGateway() {
    m_gatewayMotor.set(0);
  }

  // this is to be called in execute in a command.
  // we are equating the current rpm to the constant rpm as a placeholder, should
  // not be max.

  public boolean isRampedUp() {
    return m_topEncoder.getVelocity() > ShooterConstants.kShooterMaxVelocity;
    // Please Collin come back and check this - we are just yapping.
  }

  public double lookupFF(double speed) {
    double FFPercent = Math.min(8.48 * Math.pow(10, -5) * (speed) + .0143, 1.0);
    return FFPercent;
  }
  
  public int lookupSlot(double speed) {
    return 0;
  }

  public void shootSmartDashboard() { // change name from shootsmartdashboard
    m_lowMotorPidController.setReference(cVelocity, ControlType.kSmartVelocity, 0);
    m_lowMotorPidController.setReference(cVelocity, ControlType.kSmartVelocity, 0);
  }

  public void shootTwoSmartDashboard() { // change from twosmart dashboard to other name
    m_topMotorPidController.setReference(cVelocity, ControlType.kSmartVelocity, 0);
    m_lowMotorPidController.setReference(cVelocity2, ControlType.kSmartVelocity, 0);
  }

  public void shootTwoSmartDashboardFF() {
    m_topMotorPidController.setReference(cVelocity, ControlType.kSmartVelocity, 0, lookupFF(cVelocity),
        ArbFFUnits.kPercentOut);
    m_lowMotorPidController.setReference(cVelocity2, ControlType.kSmartVelocity, 0, lookupFF(cVelocity),
        ArbFFUnits.kPercentOut);
  }

  public void slowDownToZero() {
    m_topMotorPidController.setReference(0, ControlType.kSmartVelocity, 0, lookupFF(0), ArbFFUnits.kPercentOut);
    m_lowMotorPidController.setReference(0, ControlType.kSmartVelocity, 0, lookupFF(0), ArbFFUnits.kPercentOut);
  }

  // This method actually ramps it up once ready to shoot.
  public void spinGateway() {
    m_gatewayMotor.set(ShooterConstants.kGatewayMotorSpeed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Gateway Wheels Speed", m_gatewayEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter Wheels Speed", m_topEncoder.getVelocity());
    // This method will be called once per scheduler run
  }
}

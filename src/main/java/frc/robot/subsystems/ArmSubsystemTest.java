// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystemTest extends SubsystemBase {
  private CANSparkMax m_angleMotor;
  private double m_desiredAngle;
  private AbsoluteEncoder m_armEncoder;
  private SparkLimitSwitch m_limitSwitchTop;
  private SparkLimitSwitch m_limitSwitchBottom;
  private DoubleSolenoid m_discBrake;
  private ArmFeedforward m_feedforward;

  public ArmSubsystemTest() {
    m_angleMotor = new CANSparkMax(ArmConstants.kAngleMotorCanID, MotorType.kBrushless);
    m_angleMotor.setInverted(true);
    m_limitSwitchTop = m_angleMotor.getForwardLimitSwitch(com.revrobotics.SparkLimitSwitch.Type.kNormallyOpen);
    m_limitSwitchBottom = m_angleMotor.getReverseLimitSwitch(com.revrobotics.SparkLimitSwitch.Type.kNormallyOpen);

    m_armEncoder = m_angleMotor.getAbsoluteEncoder(Type.kDutyCycle);

    m_desiredAngle = 0;

    m_discBrake = new DoubleSolenoid(PneumaticsModuleType.REVPH, ArmConstants.kDiscBrakeForwardID,
        ArmConstants.kDiscBrakeBackwardID);

    m_feedforward = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);

  }
  public boolean isForwardLimitReached() {
    return m_limitSwitchTop.isPressed();
  }

  public boolean isReverselimitReached() {
    return m_limitSwitchBottom.isPressed();
  }

  public double getAbsoluteAngle() {
    return m_armEncoder.getPosition();
  }

  public void armBrake() {
    m_discBrake.set(DoubleSolenoid.Value.kForward);
  }

  public void armRelease() {
    m_discBrake.set(DoubleSolenoid.Value.kReverse);
  }

  public void stopArm() {
    m_angleMotor.set(0);
  }

  public void moveArm(double desiredAngle) {

    m_desiredAngle = desiredAngle;

    if (m_desiredAngle > getAbsoluteAngle()) {
      m_angleMotor.set(ArmConstants.kStowSpeed);
    } else {
      m_angleMotor.set((ArmConstants.kStowSpeed) * -1);
    }
  }

  public boolean AngleReached() {
    return getAbsoluteAngle() >= m_desiredAngle - 5 && getAbsoluteAngle() <= m_desiredAngle + 5;
  }

  public void moveArmJoystick(double speed) {
    m_angleMotor.set(speed * .25);
  }

  public void moveArmSpeed(double speed) {
    m_angleMotor.set(speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Angle Reached?", AngleReached());
    SmartDashboard.putNumber("Absolute Angle", getAbsoluteAngle());
    SmartDashboard.putBoolean("Forward Limit Switch", m_limitSwitchTop.isPressed());
    SmartDashboard.putBoolean("Reverse Limit Switch", m_limitSwitchBottom.isPressed());
  }
}

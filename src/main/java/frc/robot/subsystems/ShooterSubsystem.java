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

import edu.wpi.first.wpilibj.Timer;
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
  private double cVelocityTop, cVelocityLow;
  // private RelativeEncoder m_gatewayEncoder;
  double p, i, d, velocity, velocity2, cP, cI, cD, cVelocity, cVelocity2, m_dial1, m_dial2, maxRPMTop, maxRPMLow,
      initialTimeTop, initialTimeLow, riseTimeTop, riseTimeLow, startOscillationTimeTop, startOscillationTimeLow,
      finalOscillationTimeTop, finalOscillationTimeLow;
  // boolean bOscillatingTop, bOscillatingLow;
  // Timer oscillationStopWatchTop;
  // Timer oscillationStopWatchLow;

  public ShooterSubsystem() {
    m_topMotor = new CANSparkMax(ShooterConstants.kShooterMotorTopCanID, MotorType.kBrushless);
    m_lowMotor = new CANSparkMax(ShooterConstants.kShooterMotorLowCanID, MotorType.kBrushless);
    m_topMotor.setInverted(true);
    m_lowMotor.setInverted(true);

    m_gatewayMotor = new CANSparkMax(ShooterConstants.kTopGatewayWheelMotorID, MotorType.kBrushless);

    m_gatewayMotor.setInverted(false);

    m_topEncoder = m_topMotor.getEncoder();
    m_lowEncoder = m_lowMotor.getEncoder();
    m_topEncoder.setVelocityConversionFactor(ShooterConstants.kShooterGearRatio);
    m_lowEncoder.setVelocityConversionFactor(ShooterConstants.kShooterGearRatio);

    m_topMotorPidController = m_topMotor.getPIDController();
    m_lowMotorPidController = m_lowMotor.getPIDController();

    // m_gatewayEncoder = m_gatewayMotor.getEncoder();

    cVelocity = 0;
    cVelocity2 = 0;
    // m_dial1 = 0;
    // m_dial2 = 0;
    // maxRPMTop = 0;
    // maxRPMLow = 0;
    // // oscillationStopWatchLow = new Timer();
    // // oscillationStopWatchTop = new Timer();

    // SmartDashboard.putNumber("Set Velocity top", velocity);
    // SmartDashboard.putNumber("Set Velocity low", velocity2);
    // SmartDashboard.putNumber("P Gain", ShooterConstants.kPt0);
    // SmartDashboard.putNumber("I Gain", ShooterConstants.kIt0);
    // SmartDashboard.putNumber("D Gain", ShooterConstants.kDt0);

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

  // public void initShoot() {
  // maxRPMTop = 0;
  // maxRPMLow = 0;
  // initialTimeTop = Timer.getFPGATimestamp();
  // initialTimeLow = Timer.getFPGATimestamp();
  // riseTimeTop = 0;
  // riseTimeLow = 0;
  // finalOscillationTimeTop = 0;
  // finalOscillationTimeLow = 0;
  // bOscillatingTop = false;
  // bOscillatingLow = false;
  // }

  public void shoot() {
    shootSmartVelocity(ShooterConstants.kShooterSpeedTop, ShooterConstants.kShooterSpeedLow);
  }

  // public void shootTop(double speed, double dial) {
  // m_dial1 = (dial + 1) / 2;
  // m_topMotor.set(speed * m_dial1);
  // }

  // public void shootLow(double speed, double dial) {
  // m_dial2 = (dial + 1) / 2;
  // m_lowMotor.set(speed * m_dial2);
  // }

  public double getTopMotorSpeed() {
    return m_topEncoder.getVelocity();
  }

  public double getLowMotorSpeed() {
    return m_lowEncoder.getVelocity();
  }

  public void intake() {

    shootSmartVelocity(ShooterConstants.kIntakeSpeed);
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

  public void intakeGateway() {
    m_gatewayMotor.set(ShooterConstants.kGatewayMotorSpeed);
  }

  public void shootGateway() {
    m_gatewayMotor.set(-ShooterConstants.kGatewayMotorSpeed);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Percent Shooter Motor 1", m_dial1);
    // SmartDashboard.putNumber("Percent Shooter Motor 2", m_dial2);
    SmartDashboard.putNumber("Top Motor", getTopMotorSpeed());
    SmartDashboard.putNumber("Low Motor", getLowMotorSpeed());
    // SmartDashboard.putNumber("maxRPMTop", maxRPMTop);
    // SmartDashboard.putNumber("maxRPMLow", maxRPMLow);

    // SmartDashboard.putNumber("Rise Time Top", riseTimeTop);
    // SmartDashboard.putNumber("Rise Time Low", riseTimeLow);
    // SmartDashboard.putNumber("Stable Oscillation Time Top",
    // finalOscillationTimeTop);
    // SmartDashboard.putNumber("Stable Oscillation Time Low",
    // finalOscillationTimeLow);
    // if (getLowMotorSpeed() > maxRPMLow) {
    // maxRPMLow = getLowMotorSpeed();
    // }

    // if (getLowMotorSpeed() >= cVelocity2 && riseTimeLow == 0) {
    // riseTimeLow = Timer.getFPGATimestamp() - initialTimeLow;
    // startOscillationTimeLow = Timer.getFPGATimestamp();
    // oscillationStopWatchLow.start();
    // bOscillatingLow = true;
    // }
    // if (bOscillatingLow) {
    // if (getLowMotorSpeed() < cVelocity2 + 10 && getLowMotorSpeed() > cVelocity2 -
    // 10) {
    // oscillationStopWatchLow.start();
    // if (oscillationStopWatchLow.get() >= .5) {
    // finalOscillationTimeLow = Timer.getFPGATimestamp() - startOscillationTimeLow
    // - .5;
    // bOscillatingLow = false;
    // }
    // } else {
    // oscillationStopWatchLow.stop();
    // oscillationStopWatchLow.reset();
    // }
    // }

    // if (getTopMotorSpeed() > maxRPMTop) {
    // maxRPMTop = getTopMotorSpeed();
    // }

    // if (getTopMotorSpeed() >= cVelocity && riseTimeTop == 0) {
    // riseTimeTop = Timer.getFPGATimestamp() - initialTimeTop;
    // startOscillationTimeTop = Timer.getFPGATimestamp();
    // oscillationStopWatchTop.start();
    // bOscillatingTop = true;
    // }
    // if (bOscillatingTop) {
    // if (getTopMotorSpeed() < cVelocity + 10 && getTopMotorSpeed() > cVelocity -
    // 10) {
    // oscillationStopWatchTop.start();
    // if (oscillationStopWatchTop.get() >= .5) {
    // finalOscillationTimeTop = Timer.getFPGATimestamp() - startOscillationTimeTop
    // - .5;
    // bOscillatingTop = false;
    // }
    // } else {
    // oscillationStopWatchTop.stop();
    // oscillationStopWatchTop.reset();
    // }
    // }

    // p = SmartDashboard.getNumber("P Gain", 0);
    // i = SmartDashboard.getNumber("I Gain", 0);
    // d = SmartDashboard.getNumber("D Gain", 0);
    // velocity = SmartDashboard.getNumber("Set Velocity", 0);
    // velocity2 = SmartDashboard.getNumber("Set Velocity2", 0);

    if (p != cP) {
      m_topMotorPidController.setP(p);
      m_lowMotorPidController.setP(p);
      cP = p;
    }
    if (i != cI) {
      m_topMotorPidController.setI(i);
      m_lowMotorPidController.setI(i);
      cI = i;
    }
    if (d != cD) {
      m_topMotorPidController.setD(d);
      m_lowMotorPidController.setD(d);
      cD = d;
    }
    if (velocity != cVelocity) {
      cVelocity = velocity;
    }
    if (velocity2 != cVelocity2) {
      cVelocity2 = velocity2;
    }

  }
}
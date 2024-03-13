// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
// import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private Climber m_rightClimber;
  private Climber m_leftClimber;
  private double m_maxRightAmps;
  private double m_maxLeftAmps;

  public ClimberSubsystem() {
    m_leftClimber = new Climber(ClimberConstants.kClimberLeftMotorCanID, ClimberConstants.kClimberLeftForward,
        ClimberConstants.kClimberLeftReverse);
    m_rightClimber = new Climber(ClimberConstants.kClimberRightMotorCanID, ClimberConstants.kClimberRightForward,
        ClimberConstants.kClimberRightReverse);
    m_maxRightAmps = 0;
    m_maxLeftAmps = 0;

    m_leftClimber.setInverted(true);

  }

  public void moveRightClimber(DoubleSupplier req) {
    m_rightClimber.setMotor(req.getAsDouble());
  }

  public void moveLeftClimber(DoubleSupplier req) {
    m_leftClimber.setMotor(req.getAsDouble());
  }

  public void stopMotors() {
    m_leftClimber.setMotor(0);
    m_rightClimber.setMotor(0);
  }

  public void startClimb() {
    m_leftClimber.startClimb();
    m_rightClimber.startClimb();

  }

  public void resetMaxAmps() {
    m_maxLeftAmps = 0;
    m_maxRightAmps = 0;
  }

  public double getRightMaxAmps() {
    if (m_rightClimber.getAmps() > m_maxRightAmps) {
      m_maxRightAmps = m_rightClimber.getAmps();
    }
    return m_maxRightAmps;
  }

  public double getLeftMaxAmps() {
    if (m_leftClimber.getAmps() > m_maxLeftAmps) {
      m_maxLeftAmps = m_leftClimber.getAmps();
    }
    return m_maxLeftAmps;
  }

  public void finishClimb() {
    System.out.println(m_leftClimber.checkHit() && m_rightClimber.checkHit());
    if (m_leftClimber.checkHit() && m_rightClimber.checkHit()) {
      m_rightClimber.setMotor(ClimberConstants.kLoweringClimbingSpeed);
      m_leftClimber.setMotor(ClimberConstants.kLoweringClimbingSpeed);
      if (m_leftClimber.isFinishedLowering() || m_rightClimber.isFinishedLowering()) {
        System.out.println("Left Climber: " + m_leftClimber.isFinishedLowering());
        System.out.println("Right Climber: " + m_rightClimber.isFinishedLowering());
        m_leftClimber.stop();
        m_rightClimber.stop();
      }
    }
  }

  public void raiseArms() {
    m_leftClimber.setMotor(ClimberConstants.kClimberRaisingSpeed);
    m_rightClimber.setMotor(ClimberConstants.kClimberRaisingSpeed);
  }

  public boolean getSwitchStatus() {
    return (m_leftClimber.getForwardSwitch().isPressed() && m_rightClimber.getForwardSwitch().isPressed());
  }

  public boolean getLeftSwitchStatus() {
    return m_leftClimber.getForwardSwitch().isPressed();
  }

  public boolean getRightSwitchStatus() {
    return m_rightClimber.getReverseSwitch().isPressed();
  }

  public double getPositionRight() {
    return m_rightClimber.getEncoder();
  }

  public double getPositionLeft() {
    return m_leftClimber.getEncoder();
  }

  public void release() {
    m_leftClimber.release();
    m_rightClimber.release();
  }

  /////////////// Testing
  public void releaseLeft() {
    m_leftClimber.release();
  }

  public void releaseRight() {
    m_rightClimber.release();
  }

  public void brakeLeft() {
    m_leftClimber.brake();
  }

  public void brakeRight() {
    m_rightClimber.brake();
  }
  /////////////////////

  public void climbingOrder() {

    if (m_leftClimber.checkHit() && m_rightClimber.checkHit()) {
      m_leftClimber.release();
      m_rightClimber.release();
      m_leftClimber.setMotor(ClimberConstants.kLoweringClimbingSpeed);
      m_rightClimber.setMotor(-ClimberConstants.kLoweringClimbingSpeed);
      return;
    }


    if (m_leftClimber.checkHit()) {
      m_leftClimber.stop();
      m_leftClimber.brake();
    } else {
      m_leftClimber.setMotor(ClimberConstants.kLoweringClimbingSpeed);
    }
    if (m_rightClimber.checkHit()) {
      m_rightClimber.stop();
      m_rightClimber.brake();
    } else {
      m_rightClimber.setMotor(-ClimberConstants.kLoweringClimbingSpeed);
    }
    // m_leftClimber.hitChain(m_leftClimber.checkHit());
    // m_rightClimber.hitChain(m_rightClimber.checkHit());
    // finishClimb();
  }

  @Override
  public void periodic() {
    if (m_leftClimber.getForwardSwitch().isPressed()) {
      m_leftClimber.resetEncoder();
    }
    if (m_rightClimber.getReverseSwitch().isPressed()) {
      m_rightClimber.resetEncoder();
    }

    SmartDashboard.putNumber("Left Climber amps", m_leftClimber.getAmps());
    SmartDashboard.putNumber("Right Climber amps", m_rightClimber.getAmps());

    SmartDashboard.putBoolean("Is hit left", m_leftClimber.getIsReady());
    SmartDashboard.putBoolean("Is hit right", m_rightClimber.getIsReady());

    SmartDashboard.putNumber("Left climber max amps", getLeftMaxAmps());
    SmartDashboard.putNumber("Right climber max amps", getRightMaxAmps());

    SmartDashboard.putNumber("Left climber current relative pos ", m_leftClimber.getEncoder());
    SmartDashboard.putNumber("Right climber current relative pos ", m_rightClimber.getEncoder());

    SmartDashboard.putBoolean("CLIMBER Right Forward Limit Switch", m_rightClimber.getForwardSwitch().isPressed());
    SmartDashboard.putBoolean("CLIMBER Left Forward Limit Switch", m_leftClimber.getForwardSwitch().isPressed());
    SmartDashboard.putBoolean("CLIMBER Right Reverse Limit Switch", m_rightClimber.getReverseSwitch().isPressed());
    SmartDashboard.putBoolean("CLIMBER Left Reverse Limit Switch", m_leftClimber.getReverseSwitch().isPressed());
  }

  public class Climber {
    private CANSparkMax m_motor;
    // private RelativeEncoder m_encoder;
    private boolean m_bIsInverted;
    private double m_previousAmps;
    // private double m_previousTime;
    public boolean m_ready;
    private double m_timeSinceStart;
    private double m_initialClimbTime;
    private int m_hitCounter;
    private DoubleSolenoid m_brake;
    private SparkLimitSwitch m_limitSwitchForward, m_limitSwitchReverse;
    private RelativeEncoder m_relativeEncoder;

    public Climber(int CanID, int forward, int reverse) {
      m_motor = new CANSparkMax(CanID, MotorType.kBrushless);
      m_motor.setSmartCurrentLimit(ClimberConstants.kSmartCurrentLimitAmps);
      // m_encoder = m_motor.getEncoder();
      m_limitSwitchForward = m_motor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
      m_limitSwitchReverse = m_motor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
      m_limitSwitchForward.enableLimitSwitch(true);
      m_limitSwitchReverse.enableLimitSwitch(true);
      m_previousAmps = 0;
      // m_previousTime = m_initialClimbTime = Timer.getFPGATimestamp();
      m_ready = false;
      m_timeSinceStart = 0;
      m_brake = new DoubleSolenoid(PneumaticsModuleType.REVPH, forward, reverse);
      m_relativeEncoder = m_motor.getEncoder();
      m_hitCounter = 0;

      m_bIsInverted = false;
      // if inverted then forward limit switch and motor + is down and encoder reads
      // negative going up
      // if not inverted then reverse limit switch and motor - is down and encoder
      // reads positive going up
    }

    public void setInverted(boolean inverted) {
      m_bIsInverted = inverted;
      // m_relativeEncoder.setInverted(m_bIsInverted);
    }

    public SparkLimitSwitch getForwardSwitch() {
      // if (m_bIsInverted) {
      // return m_limitSwitchReverse;
      // }
      // return m_limitSwitchForward;
      return m_limitSwitchForward;
      // could be reverse limit switch, depends on hardware
    }

    public void resetEncoder() {
      m_relativeEncoder.setPosition(0);
    }

    public double getEncoder() {
      return m_relativeEncoder.getPosition();
    }

    public SparkLimitSwitch getReverseSwitch() {
      return m_limitSwitchReverse;
      // if (m_bIsInverted) {
      // return m_limitSwitchForward;
      // }
      // return m_limitSwitchReverse;
    }

    public boolean isFinishedLowering() {

      return getReverseSwitch().isPressed();
      // check which switch it top and bottom, need bottom here

      // February 12th, 11:18am - 2024
      // We impressed Collin very much with this, when he saw we found the isPressed()
      // method he said "I am very impressed" and smiled :3
    }

    public boolean isFinishedRaising() {

      return ClimberConstants.kEncoderIsRaised <= Math.abs(m_relativeEncoder.getPosition());
    }

    public void hitChain(boolean hasHit) {
      if (hasHit) {
        m_motor.set(0);
      } else {

        if (m_bIsInverted) {
          m_motor.set((ClimberConstants.kLoweringClimbingSpeed) * -1);
        }

        m_motor.set(ClimberConstants.kLoweringClimbingSpeed);

      }
    }

    public void setMotor(double constant) {
      m_motor.set(constant);
    }

    public void stop() {
      // brake.set(DoubleSolenoid.Value.kForward);
      m_motor.set(0);
    }

    public void release() {
      m_brake.set(DoubleSolenoid.Value.kReverse);
    }

    public void brake() {
      m_brake.set(DoubleSolenoid.Value.kForward);
    }

    public double getDifference(double previousAmps) {
      double difference = (getAmps() - previousAmps);
      m_previousAmps = getAmps();
      return difference;
    }

    public void startClimb() {
      m_hitCounter = 0;
      // m_previousTime = m_initialClimbTime = Timer.getFPGATimestamp();
      m_ready = false;
      // brake();
      m_timeSinceStart = 0;
    }

    // checkHit returns a boolean (whether chain is contacted or not) which hitChain
    // then takes in.
    public boolean checkHit() {
      // System.out.println(m_timeSinceStart + "Time Since Start");
      // System.out.println(m_initialClimbTime + "Initial Climb Time");
      // m_timeSinceStart = Timer.getFPGATimestamp() - m_initialClimbTime;
      // if (m_timeSinceStart >= 1) {
      // System.out.println("TIMER DONE!!!");
      if (getDifference(m_previousAmps) > ClimberConstants.kDifferenceInRate) {
        if (getAmps() > 12) {
          m_hitCounter++;
          if (m_hitCounter > 5) {
            m_ready = true;
          }

          return m_ready;
        }
        m_hitCounter = 0;
      }
      // }
      return m_ready;
    }

    public double getAmps() {
      return m_motor.getOutputCurrent();
    }

    public boolean getIsReady() {
      return m_ready;
    }

    public boolean isAtRest() {
      if (isFinishedLowering() && m_brake.get() == DoubleSolenoid.Value.kForward) {
        return true;
      } else {
        return false;
      }
    }
  }
  // Cant Zero, Find Difference
}

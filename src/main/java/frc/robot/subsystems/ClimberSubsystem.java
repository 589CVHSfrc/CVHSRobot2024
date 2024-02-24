// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
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
      m_rightClimber.setMotor(ClimberConstants.kClimbingSpeed);
      m_leftClimber.setMotor(ClimberConstants.kClimbingSpeed);
      if (m_leftClimber.isFinishedLowering() || m_rightClimber.isFinishedLowering()) {
        System.out.println("Left Climber: " + m_leftClimber.isFinishedLowering());
        System.out.println("Right Climber: " + m_rightClimber.isFinishedLowering());
        m_leftClimber.stop();
        m_rightClimber.stop();
      }
    }
  }

  public void raiseArms() {
    m_leftClimber.setMotor(ClimberConstants.kArmRaisingSpeed);
    m_rightClimber.setMotor(ClimberConstants.kArmRaisingSpeed);
  }

  public boolean getSwitchStatus() {
    return (m_leftClimber.getForwardSwitch().isPressed() && m_rightClimber.getForwardSwitch().isPressed());
  }

  public void release() {
    m_leftClimber.release();
    m_rightClimber.release();
  }

  public void climbingOrder() {
    m_leftClimber.hitChain(m_leftClimber.checkHit());
    m_rightClimber.hitChain(m_rightClimber.checkHit());
    finishClimb();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Climber amps", m_leftClimber.getAmps());
    SmartDashboard.putNumber("Right Climber amps", m_rightClimber.getAmps());
    SmartDashboard.putBoolean("Is hit left", m_leftClimber.getIsReady());
    SmartDashboard.putBoolean("Is hit right", m_rightClimber.getIsReady());
    SmartDashboard.putNumber("Left climber max amps", getLeftMaxAmps());
    SmartDashboard.putNumber("Right climber max amps", getRightMaxAmps());
    SmartDashboard.putBoolean("Right Fordward LimitForardSwitch hit", m_rightClimber.getForwardSwitch().isPressed());
    SmartDashboard.putBoolean("Left Forward LimitForwardSwitch hit", m_leftClimber.getForwardSwitch().isPressed());
    SmartDashboard.putBoolean("Right Reverse LimitSwitch hit", m_rightClimber.getReverseSwitch().isPressed());
    SmartDashboard.putBoolean("Left Reverse LimitSwitch hit", m_leftClimber.getReverseSwitch().isPressed());
  }

  public class Climber {
    private CANSparkMax m_motor;
    // private RelativeEncoder m_encoder;
    private double m_previousAmps;
    private double m_previousTime;
    private double m_deltaTime;
    public boolean m_ready;
    private double m_timeSinceStart;
    private double m_initialClimbTime;
    private boolean bstartupTime;
    private DoubleSolenoid brake;
    private SparkLimitSwitch m_limitSwitchForward, m_limitSwitchReverse;
    private boolean m_isTimerReached;

    public Climber(int CanID, int forward, int reverse) {
      m_motor = new CANSparkMax(CanID, MotorType.kBrushless);
      // m_encoder = m_motor.getEncoder();
      m_limitSwitchForward = m_motor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
      m_limitSwitchReverse = m_motor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
      m_limitSwitchForward.enableLimitSwitch(true);
      m_limitSwitchReverse.enableLimitSwitch(true);
      m_previousAmps = 0;
      m_previousTime = m_initialClimbTime = Timer.getFPGATimestamp();
      m_deltaTime = Timer.getFPGATimestamp() - m_previousTime;
      m_ready = false;
      m_timeSinceStart = 0;
      m_isTimerReached = false;
      bstartupTime = false;
      brake = new DoubleSolenoid(PneumaticsModuleType.REVPH, forward, reverse);

    }

    public SparkLimitSwitch getForwardSwitch() {
      return m_limitSwitchForward;
      // could be reverse limit switch, depends on hardware
    }

    public SparkLimitSwitch getReverseSwitch() {
      return m_limitSwitchReverse;
    }

    public boolean isFinishedLowering() {
      return m_limitSwitchReverse.isPressed();
      // check which switch it top and bottom, need bottom here

      // February 12th, 11:18am - 2024
      // We impressed Collin very much with this, when he saw we found the isPressed()
      // method he said "I am very impressed" and smiled :3
    }

    public boolean isFinishedRaising() {
      return m_limitSwitchForward.isPressed();
    }

    public void hitChain(boolean hasHit) {
      if (hasHit) {
        m_motor.set(0);
      } else {
        m_motor.set(ClimberConstants.kClimbingSpeed);
      }
    }

    public void setMotor(double constant) {
      m_motor.set(constant);
    }

    public void stop() {
      //brake.set(DoubleSolenoid.Value.kForward);
      m_motor.set(0);
    }

    public void release() {
      brake.set(DoubleSolenoid.Value.kReverse);
    }

    public void brake(){
      brake.set(DoubleSolenoid.Value.kForward);
    }

    public double getDifference(double previousAmps) {
      double difference = (getAmps() - previousAmps);
      m_previousAmps = getAmps();
      return difference;
    }

    public void startClimb() {

      bstartupTime = false;
      m_previousTime = m_initialClimbTime = Timer.getFPGATimestamp();
      m_ready = false;
      brake();
      m_timeSinceStart = 0;
    }

    // checkHit returns a boolean (whether chain is contacted or not) which hitChain
    // then takes in.
    public boolean checkHit() {
      // System.out.println(m_timeSinceStart + "Time Since Start");
      // System.out.println(m_initialClimbTime + "Initial Climb Time");
      m_timeSinceStart = Timer.getFPGATimestamp() - m_initialClimbTime;
      if (m_timeSinceStart >= 1) {
        m_isTimerReached = true;
        // System.out.println("TIMER DONE!!!");
        if (getDifference(m_previousAmps) > ClimberConstants.kDifferenceInRate) {
          if (getAmps() > 12) {
            m_ready = true;
            return m_ready;
          } else {
            System.out.println("Waiting for Timer");
          }
        }
      }
      return m_ready;
    }

    public double getAmps() {
      return m_motor.getOutputCurrent();
    }

    public boolean getIsReady() {
      return m_ready;
    }
  }
}

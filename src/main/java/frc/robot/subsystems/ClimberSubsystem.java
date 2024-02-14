// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private Climber m_rightClimber;
  private Climber m_leftClimber;

  public ClimberSubsystem() {
    m_leftClimber = new Climber(ClimberConstants.kClimberLeftMotorCanID, ClimberConstants.kClimberLeftForward, ClimberConstants.kClimberLeftReverse);
    m_rightClimber = new Climber(ClimberConstants.kClimberRightMotorCanID, ClimberConstants.kClimberRightForward, ClimberConstants.kClimberRightReverse);
  }


 public void startClimb(){
    m_leftClimber.startClimb();
    m_rightClimber.startClimb();

 }

  public void finishClimb(){
    if(m_leftClimber.checkHit() == true && m_rightClimber.checkHit() == true){
      m_rightClimber.setMotor(ClimberConstants.kClimbingSpeed);
      m_leftClimber.setMotor(ClimberConstants.kClimbingSpeed);
      if(m_leftClimber.isFinishedLowering() && m_rightClimber.isFinishedLowering()){
        m_leftClimber.brake();
        m_rightClimber.brake();
      }
    }
  }

  public void raiseArms(){
    m_leftClimber.setMotor(ClimberConstants.kArmRaisingSpeed);
    m_rightClimber.setMotor(ClimberConstants.kArmRaisingSpeed);
  }

  public boolean getSwitchStatus() {
    return(m_leftClimber.getSwitch().isPressed() && m_rightClimber.getSwitch().isPressed());
  }



  public void release(){
    m_leftClimber.release();
    m_rightClimber.release();
  }

  public void climbingOrder(){
    m_leftClimber.hitChain(m_leftClimber.checkHit());
    m_rightClimber.hitChain(m_rightClimber.checkHit());
    finishClimb();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public class Climber{
    CANSparkMax m_ArmMotor;
    RelativeEncoder m_encoder;
    private double m_previousAmps;
    private double m_previousTime;
    private double m_deltaTime;
    public boolean m_ready;
    private double m_timeSinceStart;
    private double m_initialClimbTime;
    private boolean bstartupTime;
    private DoubleSolenoid brake;
    private SparkLimitSwitch m_limitSwitchForward, m_limitSwitchReverse;

    public Climber(int CanID, int forward, int reverse){
      m_ArmMotor = new CANSparkMax(CanID, MotorType.kBrushless);
      m_encoder = m_ArmMotor.getEncoder();
      m_limitSwitchForward = m_ArmMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
      m_limitSwitchReverse = m_ArmMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
      m_limitSwitchForward.enableLimitSwitch(true);
      m_limitSwitchReverse.enableLimitSwitch(true);
      m_previousAmps = 0;
      m_previousTime = m_initialClimbTime = Timer.getFPGATimestamp();
      m_deltaTime = Timer.getFPGATimestamp() - m_previousTime;
      m_ready = false;
      m_timeSinceStart = 0;
      bstartupTime = false;
      brake = new DoubleSolenoid(PneumaticsModuleType.REVPH, forward, reverse);

    }
    
    public SparkLimitSwitch getSwitch() {
      return m_limitSwitchForward;
      //could be reverse limit switch, depends on hardware
    }
    public boolean isFinishedLowering(){
      return m_limitSwitchForward.isPressed();
      //check which switch it top and bottom, need bottom here

      //February 12th, 11:18am - 2024
      //We impressed Collin very much with this, when he saw we found the isPressed() method he said "I am very impressed" and smiled :3
    }

    public void hitChain(boolean hasHit){
        if(hasHit){
          m_ArmMotor.set(0);
        }
        else{
          m_ArmMotor.set(ClimberConstants.kClimbingSpeed);
        }
    }

    public void setMotor(double constant){
      m_ArmMotor.set(constant);
    }

    public void brake(){
      brake.set(DoubleSolenoid.Value.kForward);
    }
    
    public void release(){
      brake.set(DoubleSolenoid.Value.kReverse);
    }

    public double getDifference(double previousAmps){
      double difference = (getAmps()-previousAmps)*m_deltaTime;
      m_previousAmps = getAmps();
      return difference;
    }
    
    public void startClimb(){
    bstartupTime = false;
    m_previousTime = m_initialClimbTime = Timer.getFPGATimestamp();
    m_ready = false;
    brake();
    }

    //checkHit returns a boolean (whether chain is contacted or not) which hitChain then takes in.
    public boolean checkHit(){
      m_timeSinceStart = Timer.getFPGATimestamp() - m_initialClimbTime;
      if ( m_timeSinceStart >= .25 ){
       bstartupTime = true;
      }
      if(bstartupTime){
        if(getDifference(m_previousAmps)>ClimberConstants.kDifferenceInRate){
          m_ready = true;
          return m_ready;
        }
      }
      return false;
      
    }

    public double getAmps(){
      return m_ArmMotor.getOutputCurrent();
    }
  }
}

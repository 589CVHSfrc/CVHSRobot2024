package frc.robot.commands.COMMAND_DRIVE;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.DriveUtils;

public class DrivePose {
    private DriveSubsystem m_drive;
    private DoubleSupplier m_speed;
    public DrivePose(DriveSubsystem drive, DoubleSupplier speed){
        m_drive = drive;
        m_speed = speed;
    }
    public Command driveShootSpeaker(){
        //RED IS TRUE
        if(m_drive.getAlliance()){
            
            return new DriveUtils(m_drive).driveToPose(DriveConstants.kShootingPoseSpeakerRED, m_speed);
        }
        return new DriveUtils(m_drive).driveToPose(DriveConstants.kShootingPoseSpeakerBLUE, m_speed); 
    }
    public Command driveShootAmp(){
                //System.out.println("RED IS TRUE "+ m_drive.getAlliance() );

        if(m_drive.getAlliance()){
            System.out.println("============RED");
            return new DriveUtils(m_drive).driveToPose(DriveConstants.kShootingPoseAmpRED, m_speed);
        }
        System.out.println("=================BLUE");
        return new DriveUtils(m_drive).driveToPose(DriveConstants.kShootingPoseAmpBLUE, m_speed); 
    }

    public Command driveSource(){
        //RED IS TRUE
        if(m_drive.getAlliance()){
            
            return new DriveUtils(m_drive).driveToPose(DriveConstants.kIntakeSourceRED, m_speed);
        }
        return new DriveUtils(m_drive).driveToPose(DriveConstants.kIntakeSourceBLUE, m_speed); 
    }

}

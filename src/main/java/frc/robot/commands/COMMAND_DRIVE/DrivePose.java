package frc.robot.commands.COMMAND_DRIVE;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.DriveUtils;

public class DrivePose {
    private DriveSubsystem m_drive;
    public DrivePose(DriveSubsystem drive){
        m_drive = drive;
    }
    public Command driveShootSpeaker(){
        //RED IS TRUE
        if(m_drive.getAlliance()){
            return new DriveUtils(m_drive).driveToPose(DriveConstants.kShootingPoseSpeakerRED);
        }
        return new DriveUtils(m_drive).driveToPose(DriveConstants.kShootingPoseSpeakerBLUE); 
    }
    public Command driveShootAmp(){
        if(m_drive.getAlliance()){
            return new DriveUtils(m_drive).driveToPose(DriveConstants.kShootingPoseAmpRED);
        }
        return new DriveUtils(m_drive).driveToPose(DriveConstants.kShootingPoseAmpBLUE); 
    }

}

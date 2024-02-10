package frc.robot.commands.DRIVE;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.DriveUtils;

public class DrivePose {
    private DriveSubsystem m_drive;
    public DrivePose(DriveSubsystem drive){
        m_drive = drive;
    }
    public Command driveShoot(){
        if(m_drive.getAlliance()){
            return new DriveUtils(m_drive).driveToPose(DriveConstants.kShootingPoseRED);
        }
        return new DriveUtils(m_drive).driveToPose(DriveConstants.kShootingPoseBLUE); 
    }
}

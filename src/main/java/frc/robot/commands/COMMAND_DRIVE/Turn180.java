// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_DRIVE;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class Turn180 extends Command {
    /** Creates a new DefaultDrive. */
    private DriveSubsystem m_drive;
    private double m_origAngle;

    public Turn180(DriveSubsystem drive) {
        m_drive = drive;
        m_origAngle = m_drive.getGyroYaw();
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_origAngle = m_drive.getGyroYaw();

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println(m_drive.getGyroYaw());

        if (m_origAngle >= 0) {
            m_drive.updateRot(-.2);
        } else {
            m_drive.updateRot(.2);

        }

        // if (m_origAngle <= 90 && m_origAngle >= 0) {
        // m_drive.drive(m_drive.getXreq(), m_drive.getYreq(), .5, true, true);
        // } else if (m_origAngle >= -90 && m_origAngle <= 0) {
        // m_drive.drive(m_drive.getXreq(), m_drive.getYreq(), -.5, true, true);

        // } else if (m_origAngle > 90 && m_origAngle <= 180) {
        // m_drive.drive(m_drive.getXreq(), m_drive.getYreq(), -.5, true, true);

        // } else if (m_origAngle < -90 && m_origAngle > -180) {
        // m_drive.drive(m_drive.getXreq(), m_drive.getYreq(), .5, true, true);

        // }
        // m_drive.controllerXYUpdate((m_xspeed.getAsDouble()));
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.updateRot(0);
    }

    @Override
    public boolean isFinished() {
        if (m_origAngle > 0) {
            return m_drive.getGyroYaw() <= 15;
        } else {
            return m_drive.getGyroYaw() >= -15;

        }
    }
}

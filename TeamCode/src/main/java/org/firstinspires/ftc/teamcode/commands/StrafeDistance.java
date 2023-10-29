package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

public class StrafeDistance extends CommandBase {

    private final DriveSubsystem m_drive;
    private final double m_distance;
    private final double m_speed;

    /**
     * Creates a new DriveDistance.
     *
     * @param cm The number of cm the robot will drive
     * @param speed  The speed at which the robot will drive
     * @param drive  The drive subsystem on which this command will run
     */
    public StrafeDistance(double cm, double speed, DriveSubsystem drive) {
        m_distance = cm;
        m_speed = speed;
        m_drive = drive;
    }

    @Override
    public void initialize() {
        m_drive.resetEncoders();
        m_drive.drive(1, 0, 0, m_speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0, 0, 0, 0);
    }


    @Override
    public boolean isFinished() {
        return Math.abs(m_drive.getAverageEncoderDistance()) >= m_distance;
    }

}

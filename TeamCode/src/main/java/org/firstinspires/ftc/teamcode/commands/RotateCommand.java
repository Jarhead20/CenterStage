package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

public class RotateCommand extends CommandBase {

    private final DriveSubsystem m_drive;
    private final double m_angle;
    private final double m_speed;
    private final double rotate;


    /**
     * Creates a new RotateCommand.
     *
     * @param angle The angle the robot will rotate in radians
     * @param speed  The speed at which the robot will rotate
     * @param drive  The drive subsystem on which this command will run
     */
    public RotateCommand(double angle, double rotate, double speed, DriveSubsystem drive) {
        m_angle = angle;
        m_speed = speed;
        m_drive = drive;
        this.rotate = rotate;
    }

    @Override
    public void initialize() {
//        m_drive.resetEncoders();
        m_drive.drive(0, 0, rotate, m_speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0, 0, 0, 0);
    }


    @Override
    public boolean isFinished() {
        return Math.abs(m_drive.getHeading()) >= m_angle || rotate == 0;
    }

}


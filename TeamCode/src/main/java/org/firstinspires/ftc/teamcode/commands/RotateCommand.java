package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
public class RotateCommand extends CommandBase {

    private final DriveSubsystem m_drive;
    private final double m_angle;
    private final double m_speed;
    private final double rotate;
    public static double angleTolerance = 5;

    public static double kP = 4;
    public static double kI = 0.02;
    public static double kD = 0.25;

    private PIDController pid;

    private ElapsedTime timer;
    private ElapsedTime timeout;

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
        this.pid = new PIDController(kP, kI, kD);
        this.timer = new ElapsedTime();
        this.timeout = new ElapsedTime();
    }

    @Override
    public void initialize() {
//        m_drive.resetEncoders();
        timeout.reset();
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0, 0, 0, 0);
    }

    @Override
    public void execute(){
        pid.setPID(kP, kI, kD);
        m_drive.drive(0, 0, rotate, Math.min(pid.calculate(m_drive.getHeading(), m_angle), m_speed));
    }

    @Override
    public boolean isFinished() {
        if(!(Math.abs(m_drive.getHeading()-m_angle) < Math.toRadians(angleTolerance)) || rotate == 0)
            timer.reset();
        return timer.seconds() > 0.2 || timeout.seconds() > 5;
    }

}


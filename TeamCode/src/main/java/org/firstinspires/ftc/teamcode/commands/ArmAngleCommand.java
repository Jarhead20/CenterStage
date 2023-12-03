package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ArmAngleCommand extends CommandBase {
    private final ArmSubsystem arm;
    private final double armAngle;
    private final double wristAngle;
    private ElapsedTime timer;
    private final boolean wait;

    public ArmAngleCommand(ArmSubsystem arm, double armAngle, double wristAngle, boolean wait) {
        this.arm = arm;
        this.armAngle = armAngle;
        this.wristAngle = wristAngle;
        this.wait = wait;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        timer = new ElapsedTime();
        timer.reset();
    }

    @Override
    public void execute() {
        arm.armAngle = armAngle;
        arm.wristAngle = wristAngle;
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        if(!wait)
            return true;
        return timer.milliseconds() > 300;
    }
}

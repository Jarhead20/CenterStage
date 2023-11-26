package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

public class GrabCommand extends CommandBase {
    private final ArmSubsystem arm;
    private ElapsedTime timer;

    public GrabCommand(ArmSubsystem arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        timer = new ElapsedTime();
        timer.reset();
        arm.closeLeftGripper();
        arm.closeRightGripper();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds()<100;
    }
}

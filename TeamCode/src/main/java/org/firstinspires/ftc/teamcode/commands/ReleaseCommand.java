package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ReleaseCommand extends CommandBase {
    private final ArmSubsystem arm;
    private ElapsedTime timer;
    private boolean left;

    public ReleaseCommand(ArmSubsystem arm, boolean left) {
        this.arm = arm;
        addRequirements(arm);
        this.left = left;
    }

    @Override
    public void initialize() {
        timer = new ElapsedTime();
        timer.reset();
        if(left)
            arm.openLeftGripper();
        else arm.openRightGripper();
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

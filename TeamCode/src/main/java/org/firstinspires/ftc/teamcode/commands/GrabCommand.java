package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

public class GrabCommand extends CommandBase {
    private final ArmSubsystem arm;
    private ElapsedTime timer;
    private boolean waitOverride;

    public GrabCommand(ArmSubsystem arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {

        waitOverride = !arm.gripperOpen; // if gripper is already closed, don't wait
        arm.closeBothGrippers();
        timer = new ElapsedTime();
        timer.reset();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds()>300 || waitOverride;
    }
}

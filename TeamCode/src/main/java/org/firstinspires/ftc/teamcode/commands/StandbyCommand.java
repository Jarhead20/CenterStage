package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
@Config
public class StandbyCommand extends CommandBase {
    private final LiftSubsystem lift;
    private final ArmSubsystem arm;
    SequentialCommandGroup commandGroup;
    public static double armAngleUp = 0.2;

    public static double wristTransitionAngle = 1;

    public StandbyCommand(LiftSubsystem lift, ArmSubsystem arm) {
        this.lift = lift;
        this.arm = arm;
        addRequirements(lift, arm);
    }

    @Override
    public void initialize() {
        commandGroup = new SequentialCommandGroup(
                new GrabCommand(arm),
                new ArmAngleCommand(arm, armAngleUp, wristTransitionAngle, true)
        );
        commandGroup.schedule();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

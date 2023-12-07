package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
@Config
public class StandbyCommand extends CommandBase {
    private final LiftSubsystem lift;
    private final ArmSubsystem arm;
    SequentialCommandGroup commandGroup;
    public static double armAngleUp = 0.05;

    public static double wristTransitionAngle1 = 0.9;
    public static double wristTransitionAngle2 = 0.8;
    public static double armAngleMiddle = 0.15;

    public StandbyCommand(LiftSubsystem lift, ArmSubsystem arm) {
        this.lift = lift;
        this.arm = arm;
        addRequirements(lift, arm);
    }

    @Override
    public void initialize() {
        commandGroup = new SequentialCommandGroup(
                new GrabCommand(arm),
                new ArmAngleCommand(arm, armAngleMiddle, wristTransitionAngle1, true),
                new ArmAngleCommand(arm, armAngleUp, wristTransitionAngle2, true),
                new LiftCommand(lift, 0)
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

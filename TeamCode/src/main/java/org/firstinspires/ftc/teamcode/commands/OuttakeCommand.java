package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
@Config
public class OuttakeCommand extends CommandBase {
    private final LiftSubsystem lift;
    private final ArmSubsystem arm;
    SequentialCommandGroup commandGroup;
    public static double wristAngle = 0.4;
    public static double armAngleUp = 0.12;
    public static double armAngleDepo = 0.82;
    public static double wristTransitionAngle = 1;

    public OuttakeCommand(LiftSubsystem lift, ArmSubsystem arm) {
        this.lift = lift;
        this.arm = arm;
        addRequirements(lift, arm);
    }

    @Override
    public void initialize() {
        commandGroup = new SequentialCommandGroup(
                new GrabCommand(arm),
                new ArmAngleCommand(arm, armAngleUp, wristTransitionAngle, true),
                new LiftCommand(lift, 600),
                new ArmAngleCommand(arm, armAngleDepo, wristAngle, false)
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

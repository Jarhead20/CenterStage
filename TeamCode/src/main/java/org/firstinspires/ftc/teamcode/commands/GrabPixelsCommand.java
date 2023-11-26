package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

public class GrabPixelsCommand extends CommandBase {
    private final LiftSubsystem lift;
    private final IntakeSubsystem intake;
    private final ArmSubsystem arm;
    SequentialCommandGroup commandGroup;

    public GrabPixelsCommand(LiftSubsystem lift, IntakeSubsystem intake, ArmSubsystem arm) {
        this.lift = lift;
        this.intake = intake;
        this.arm = arm;
        addRequirements(lift, intake, arm);
    }

    @Override
    public void initialize() {
        commandGroup = new SequentialCommandGroup(
                new LiftCommand(lift, 0),
                new ArmAngleCommand(arm, 0, 0.3, true),
                new GrabCommand(arm),
                new ArmAngleCommand(arm, 0.3, 0, false),
                new LiftCommand(lift, 40)
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
        return commandGroup.isFinished();
    }
}

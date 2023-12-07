package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
@Config
public class GrabPixelsCommand extends CommandBase {
    private final LiftSubsystem lift;
    private final IntakeSubsystem intake;
    private final ArmSubsystem arm;
    SequentialCommandGroup commandGroup;
    public static double armAngleDown = 0.195;
    public static double armAngleTransition = 0.;
    public static double wristAngleTransition = 1.0;
    public static double secondWristTransition = 0.65;
    public static double wristGrabAngle = 0.65;


    public GrabPixelsCommand(LiftSubsystem lift, IntakeSubsystem intake, ArmSubsystem arm) {
        this.lift = lift;
        this.intake = intake;
        this.arm = arm;
        addRequirements(lift, intake, arm);
    }

    @Override
    public void initialize() {
        commandGroup = new SequentialCommandGroup(

                new ArmAngleCommand(arm, armAngleTransition, wristAngleTransition, true),
                new LiftCommand(lift, 0),
                new ArmAngleCommand(arm, armAngleDown, secondWristTransition, true),
                new ArmAngleCommand(arm, armAngleDown, wristGrabAngle, true),
                new ReleaseCommand(arm, true),
                new ReleaseCommand(arm, false)
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

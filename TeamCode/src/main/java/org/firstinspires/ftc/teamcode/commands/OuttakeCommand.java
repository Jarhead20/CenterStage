package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

@Config
public class OuttakeCommand extends CommandBase {
    private final LiftSubsystem lift;
    private final ArmSubsystem arm;
    SequentialCommandGroup commandGroup;
    public static double wristAngle = 0.3;
    public static double armAngleUp = 0.05;
    public static double armAngleDepo = 0.82;
    public static double wristTransitionAngle = 0.9;
    private boolean auto;
    public static double autoArmAngle = 0.55;
    public static double autoWristAngle = 0.4;
    public static double autoDepositDistance = 220;


    public OuttakeCommand(LiftSubsystem lift, ArmSubsystem arm) {
        this(lift, arm, false);
    }

    public OuttakeCommand(LiftSubsystem lift, ArmSubsystem arm, boolean auto){
        this.lift = lift;
        this.arm = arm;
        addRequirements(lift, arm);
        this.auto = auto;
    }

    @Override
    public void initialize() {
        commandGroup = new SequentialCommandGroup(
                new GrabCommand(arm),
                new ArmAngleCommand(arm, armAngleUp, wristTransitionAngle, true),
                new ArmAngleCommand(arm, armAngleUp, 0.8, true),
                new LiftCommand(lift, 600),
                new ArmAngleCommand(arm, armAngleDepo, wristAngle, false)
        );
        if(auto)
            commandGroup.addCommands(
                    new WaitCommand(200),
                    new LiftCommand(lift, autoDepositDistance),
                    new ArmAngleCommand(arm, autoArmAngle, autoWristAngle, false)
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

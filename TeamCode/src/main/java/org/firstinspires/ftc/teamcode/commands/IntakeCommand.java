package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeCommand extends CommandBase {
    IntakeSubsystem intake;
    ElapsedTime timer = new ElapsedTime();
    double power;
    public IntakeCommand(double power, IntakeSubsystem intake) {
        this.intake = intake;
        this.power = power;
    }

    @Override
    public void initialize() {
        timer.reset();
        intake.setPower(power);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPower(0);
    }
    @Override
    public boolean isFinished() {
        return timer.seconds() > 3;
    }
}

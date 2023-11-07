package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.DriveDistance;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.OpModeTemplate;

@Config
@Autonomous
public class CommandAutoSimple extends OpModeTemplate {
    public static double driveDistance = 60;
    public static double driveSpeed = 0.5;
    @Override
    public void initialize() {
        initHardware(true);
        waitForStart();
        schedule(new SequentialCommandGroup(
                new DriveDistance(driveDistance, 1, 0, driveSpeed, drive),
                new IntakeCommand(-1, intake),
                new DriveDistance(5, -1, 0, driveSpeed, drive)
        ));
    }
}

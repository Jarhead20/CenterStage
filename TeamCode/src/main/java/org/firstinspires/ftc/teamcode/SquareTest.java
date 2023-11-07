package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.PrintCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.DriveDistance;
import org.firstinspires.ftc.teamcode.commands.OpModeTemplate;
import org.firstinspires.ftc.teamcode.commands.RotateCommand;

import java.util.Collections;

@Config
@Autonomous
public class SquareTest extends OpModeTemplate {

    public static double distance = 50;
    public static double angle = 90;
    @Override
    public void initialize() {
        initHardware(true);
        waitForStart();
        schedule(new SequentialCommandGroup(
//                new PerpetualCommand(() -> {
//                    telemetry.addData("distance", drive.getAverageEncoderDistance());
//                    telemetry.update();
//                    return Collections.singleton(drive);
//                }),
//                new DriveDistance(distance, 1,0,0.5, drive),
//                new DriveDistance(distance, 0,1,0.5, drive),
//                new DriveDistance(distance, -1,0,0.5, drive),
//                new DriveDistance(distance, 0,-1,0.5, drive),
                new DriveDistance(distance, 1,0,0.5, drive),
                new RotateCommand(angle, 1, 0.5, drive),
                new DriveDistance(distance, 1,0,0.5, drive),
                new RotateCommand(angle, 1, 0.5, drive),
                new DriveDistance(distance, 1,0,0.5, drive),
                new RotateCommand(angle, 1, 0.5, drive),
                new DriveDistance(distance, 1,0,0.5, drive),
                new RotateCommand(angle, 1, 0.5, drive)
        ));
    }
}

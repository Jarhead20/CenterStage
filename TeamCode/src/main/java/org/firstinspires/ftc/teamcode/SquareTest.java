package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.DriveDistance;
import org.firstinspires.ftc.teamcode.commands.OpModeTemplate;

public class SquareTest extends OpModeTemplate {
    @Override
    public void initialize() {
        initHardware(true);
        schedule(new SequentialCommandGroup(
                new DriveDistance(50, 1,0,0.5, drive),
                new DriveDistance(50, 1,0,0.5, drive),
                new DriveDistance(50, 1,0,0.5, drive),
                new DriveDistance(50, 1,0,0.5, drive)

        ));
    }
}

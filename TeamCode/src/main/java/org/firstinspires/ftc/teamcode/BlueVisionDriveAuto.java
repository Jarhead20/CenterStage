package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name = "BlueVisionDriveAuto", group = "Autonomous", preselectTeleOp = "TeleOp")
public class BlueVisionDriveAuto extends VisionDriveAuto{
    public BlueVisionDriveAuto() {
        super(Alliance.BLUE);
    }
}

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Point;

@Autonomous(name = "RedVisionDriveAuto", group = "Autonomous", preselectTeleOp = "TeleOp")
public class RedVisionDriveAuto extends VisionDriveAuto{
    public RedVisionDriveAuto() {
        super(Alliance.RED);
    }
}

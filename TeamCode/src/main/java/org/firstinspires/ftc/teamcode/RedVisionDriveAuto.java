package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Point;

@Config
@Autonomous
public class RedVisionDriveAuto extends VisionDriveAuto{
    public static double r1x = 25;
    public static double r1y = 90;
    public static double r2x = 150;
    public static double r2y = 60;
    public static double r3x = 260;
    public static double r3y = 90;
    public RedVisionDriveAuto() {
        super(Alliance.RED,
                new Point(r1x, r1y),
                new Point(r2x, r2y),
                new Point(r3x, r3y));
    }
}

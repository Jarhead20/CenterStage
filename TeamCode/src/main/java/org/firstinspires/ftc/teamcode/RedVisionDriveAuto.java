package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Point;

@Config
@Autonomous
public class RedVisionDriveAuto extends VisionDriveAuto{
    public static double r1x = 0;
    public static double r1y = 0;
    public static double r2x = 50;
    public static double r2y = 0;
    public static double r3x = 100;
    public static double r3y = 0;
    public RedVisionDriveAuto() {
        super(Alliance.RED,
                new Point(r1x, r1y),
                new Point(r2x, r2y),
                new Point(r3x, r3y));
    }
}

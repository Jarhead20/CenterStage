package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.DriveSubsystem;
import org.firstinspires.ftc.teamcode.commands.IntakeSubsystem;

@Config
@Autonomous(name = "TestAuto")
public class TestAuto extends LinearOpMode {

    public static double driveTime = 0.8;
    public static double ejectTime = 0.5;
    public static double driveSpeed = -0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveSubsystem drive = new DriveSubsystem(hardwareMap, telemetry);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);

        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        while(opModeIsActive()){
            if(timer.seconds() < driveTime)
                drive.autoDrive(driveSpeed);
            else if (timer.seconds() < (driveTime+ejectTime) && timer.seconds() > driveTime) {
                intake.outtake();
                drive.autoDrive(0);
            } else {
                intake.stop();
                drive.autoDrive(0);
            }
        }
    }
}

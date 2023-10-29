package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
@Autonomous(name = "TestAuto")
public class TestAuto extends LinearOpMode {

    public static double driveTime = 0.8;
    public static double ejectTime = 0.5;
    public static double driveSpeed = -0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        GamepadEx driverOp = new GamepadEx(gamepad1);
        GamepadEx auxDriver = new GamepadEx(gamepad2);
        DriveSubsystem drive = new DriveSubsystem(hardwareMap, driverOp);
        LiftSubSystem lift = new LiftSubSystem("leftLift", "rightLift", hardwareMap, telemetry, auxDriver);

        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        while(opModeIsActive()){
            if(timer.seconds() < driveTime)
                drive.autoDrive(driveSpeed);
            else if (timer.seconds() < (driveTime+ejectTime) && timer.seconds() > driveTime) {
                lift.eject();
                drive.autoDrive(0);
            } else {
                lift.stopIntake();
                drive.autoDrive(0);
            }

        }

    }
}

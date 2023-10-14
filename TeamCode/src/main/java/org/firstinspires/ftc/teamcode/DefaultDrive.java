package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.function.DoubleSupplier;

/**
 * A command to drive the robot with joystick input
 * (passed in as {@link DoubleSupplier}s). Written
 * explicitly for pedagogical purposes.
 */
@TeleOp(name = "defaultDrive")
public class DefaultDrive extends CommandOpMode {

//    private DriveSubsystem drive;


    @Override
    public void initialize() {

        GamepadEx driverOp = new GamepadEx(gamepad1);
//        drive = new DriveSubsystem(hardwareMap);
        LiftSubSystem lift = new LiftSubSystem("leftLift", "rightLift", hardwareMap, telemetry, driverOp);

        schedule(new RunCommand(lift::printPos));
        schedule(new RunCommand(telemetry::update));

    }

//    @Override
//    public void runOpMode(){
//        waitForStart();
//        while(opModeInInit()){
//            telemetry.addData("test2", "test");
//        }
//        while(opModeIsActive()){
//            telemetry.addData("test", "test");
//        }
//    }
}
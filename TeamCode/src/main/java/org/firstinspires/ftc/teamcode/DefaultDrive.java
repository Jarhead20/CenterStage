package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
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
        GamepadEx auxDriver = new GamepadEx(gamepad2);
        DriveSubsystem drive = new DriveSubsystem(hardwareMap, driverOp);
        LiftSubSystem lift = new LiftSubSystem("leftLift", "rightLift", hardwareMap, telemetry, auxDriver);
        PlaneSubsystem plane = new PlaneSubsystem(hardwareMap, auxDriver);
        schedule(new RunCommand(lift::update));
        schedule(new RunCommand(telemetry::update));
        schedule(new RunCommand(drive::drive));
        schedule(new RunCommand(plane::update));
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
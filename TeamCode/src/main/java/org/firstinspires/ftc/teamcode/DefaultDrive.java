package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import java.util.function.DoubleSupplier;

/**
 * A command to drive the robot with joystick input
 * (passed in as {@link DoubleSupplier}s). Written
 * explicitly for pedagogical purposes.
 */
public class DefaultDrive extends CommandOpMode {

    private DriveSubsystem drive;
    private GamepadEx driverOp;

    @Override
    public void runOpMode() throws InterruptedException {
        drive.drive(
                driverOp.getLeftX(),
                driverOp.getLeftY(),
                driverOp.getRightY()
        );


    }

    @Override
    public void initialize() {
        driverOp = new GamepadEx(gamepad1);
        drive = new DriveSubsystem(hardwareMap);
    }
}
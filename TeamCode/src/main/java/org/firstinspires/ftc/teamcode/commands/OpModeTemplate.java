package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

abstract public class OpModeTemplate extends CommandOpMode {
    protected DriveSubsystem drive;
    protected LiftSubsystem lift;
    protected IntakeSubsystem intake;
    protected PlaneSubsystem plane;
    protected ClimbSubsystem climb;
    protected GamepadEx driverGamepad;
    protected GamepadEx secondaryGamepad;

    protected void initHardware(boolean isAuto) {

        drive = new DriveSubsystem(hardwareMap);
        lift = new LiftSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap);
        climb = new ClimbSubsystem(hardwareMap);
        plane = new PlaneSubsystem(hardwareMap);

        register(intake, drive, lift, plane, climb);

        driverGamepad = new GamepadEx(gamepad1);
        secondaryGamepad = new GamepadEx(gamepad2);
    }

    public enum Alliance {
        RED,
        BLUE;
        public double adjust(double input) {
            return this == RED ? input : -input;
        }
    }
}

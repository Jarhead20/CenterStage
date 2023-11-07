package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveSubsystem;
import org.firstinspires.ftc.teamcode.commands.LiftSubsystem;
import org.firstinspires.ftc.teamcode.commands.OpModeTemplate;
import org.firstinspires.ftc.teamcode.commands.PlaneSubsystem;

import java.util.function.DoubleSupplier;

/**
 * A command to drive the robot with joystick input
 * (passed in as {@link DoubleSupplier}s). Written
 * explicitly for pedagogical purposes.
 */
@Config
@TeleOp(name = "defaultDrive")
public class MainTeleop extends OpModeTemplate {

    public static double slowMode = 0.5;

    @Override
    public void initialize() {
        initHardware(false);

        new GamepadButton(secondaryGamepad, GamepadKeys.Button.X).whenPressed(() -> intake.intake()).whenReleased(() -> intake.stop());
        new GamepadButton(secondaryGamepad, GamepadKeys.Button.Y).whenPressed(() -> intake.outtake()).whenReleased(() -> intake.stop());

        new GamepadButton(secondaryGamepad, GamepadKeys.Button.DPAD_UP).whenPressed(() -> lift.liftOffset += 5);
        new GamepadButton(secondaryGamepad, GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> lift.liftOffset -= 5);
        new GamepadButton(secondaryGamepad, GamepadKeys.Button.DPAD_LEFT).toggleWhenPressed(()-> {
            lift.climbAttach = true;
            lift.climbGrab = false;
        }, () -> {
            lift.climbAttach = false;
            lift.climbGrab = false;
        });
        new GamepadButton(secondaryGamepad, GamepadKeys.Button.DPAD_RIGHT).toggleWhenPressed(()-> {
            lift.climbGrab = true;
            lift.climbAttach = false;
        }, () -> {
            lift.climbGrab = false;
            lift.climbAttach = false;
        });
        new GamepadButton(secondaryGamepad, GamepadKeys.Button.RIGHT_BUMPER).toggleWhenPressed(() -> lift.openGripper(), () -> lift.closeGripper());
        new GamepadButton(secondaryGamepad, GamepadKeys.Button.A).whenPressed(() -> lift.bottom());
        new GamepadButton(secondaryGamepad, GamepadKeys.Button.B).whenPressed(() -> lift.top());

        new GamepadButton(secondaryGamepad, GamepadKeys.Button.BACK).toggleWhenPressed(() -> plane.launch(),  () -> plane.reset());
    }

    @Override
    public void run(){
        super.run();

        drive.drive(
                -gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                -gamepad1.right_stick_x,
                gamepad1.right_bumper ? slowMode : 1
        );

        telemetry.addData("angle", drive.getHeading());

        lift.updateTarget(-gamepad2.left_stick_y, gamepad2.right_stick_y);

        climb.control(gamepad2.left_trigger - gamepad2.right_trigger);
    }
}
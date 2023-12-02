package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SensorColor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ArmAngleCommand;
import org.firstinspires.ftc.teamcode.commands.DriveSubsystem;
import org.firstinspires.ftc.teamcode.commands.GrabCommand;
import org.firstinspires.ftc.teamcode.commands.GrabPixelsCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommand;
import org.firstinspires.ftc.teamcode.commands.LiftSubsystem;
import org.firstinspires.ftc.teamcode.commands.OpModeTemplate;
import org.firstinspires.ftc.teamcode.commands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.commands.PlaneSubsystem;
import org.firstinspires.ftc.teamcode.commands.ReleaseCommand;
import org.firstinspires.ftc.teamcode.commands.StandbyCommand;

import java.util.function.DoubleSupplier;

/**
 * A command to drive the robot with joystick input
 * (passed in as {@link DoubleSupplier}s). Written
 * explicitly for pedagogical purposes.
 */
@Config
@TeleOp(name = "TeleOp")
public class MainTeleop extends OpModeTemplate {

    public static double slowMode = 0.5;
    @Override
    public void initialize() {
        initHardware(false);

        OuttakeCommand outtake = new OuttakeCommand(lift, arm);
        GrabPixelsCommand grab = new GrabPixelsCommand(lift, intake, arm);

        new GamepadButton(driverGamepad, GamepadKeys.Button.X).whenPressed(() -> intake.intake()).whenReleased(() -> intake.stop());
        new GamepadButton(driverGamepad, GamepadKeys.Button.Y).whenPressed(() -> intake.outtake()).whenReleased(() -> intake.stop());

        new GamepadButton(secondaryGamepad, GamepadKeys.Button.DPAD_UP).whenPressed(() -> lift.liftOffset += 5);
        new GamepadButton(secondaryGamepad, GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> lift.liftOffset -= 5);
        new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER).whenPressed(new ReleaseCommand(arm, true));
        new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new ReleaseCommand(arm, false));

        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN).whenPressed(grab).cancelWhenPressed(outtake);
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP).whenPressed(outtake).cancelWhenPressed(grab);
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_LEFT).whenPressed(new StandbyCommand(lift, arm));
        new GamepadButton(driverGamepad, GamepadKeys.Button.BACK).toggleWhenPressed(() -> plane.launch(),  () -> plane.reset());



    }

    @Override
    public void run(){
        super.run();

        drive.drive(
                gamepad1.left_stick_x,
                -gamepad1.left_stick_y,
                -gamepad1.right_stick_x,
                gamepad1.right_bumper ? slowMode : 1
        );
//
//        telemetry.addData("angle", drive.getHeading());
//
        if(intake.pixelsReady()){
            driverGamepad.gamepad.rumble(100);
            schedule(new StandbyCommand(lift, arm));
        }
        telemetry.addData("rumble", driverGamepad.gamepad.isRumbling());
        lift.updateTarget(gamepad1.right_trigger - gamepad1.left_trigger);
        arm.update(gamepad2.left_stick_y*0.05, gamepad2.right_stick_y*0.05);
        climb.control(gamepad2.left_trigger - gamepad2.right_trigger);


    }
}
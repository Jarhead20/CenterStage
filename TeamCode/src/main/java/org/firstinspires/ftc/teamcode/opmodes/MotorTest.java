package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class MotorTest extends OpMode {
    DcMotor motor;
    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "intake");
    }

    @Override
    public void loop() {
        motor.setPower(gamepad1.left_stick_y);
        telemetry.addData("test", motor.getCurrentPosition());
    }
}

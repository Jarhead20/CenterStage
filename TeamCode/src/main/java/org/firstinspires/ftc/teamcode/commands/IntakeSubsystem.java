package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem extends SubsystemBase {
    private final Motor intakeMotor;
    private final Servo leftFlap;
    private final Servo rightFlap;

    ColorRangeSensor colorSensorLeft;
    ColorRangeSensor colorSensorRight;


    public static final double leftFlapOpen = 0.5;
    public static final double leftFlapClosed = 0.0;
    public static final double rightFlapOpen = 0.5;
    public static final double rightFlapClosed = 0.0;




    public IntakeSubsystem(HardwareMap hMap){
        intakeMotor = new Motor(hMap, "intake");
        leftFlap = hMap.get(Servo.class, "leftFlap");
        rightFlap = hMap.get(Servo.class, "rightFlap");
        rightFlap.setDirection(Servo.Direction.REVERSE);
        intakeMotor.setInverted(true);

        colorSensorLeft = hMap.get(ColorRangeSensor.class, "leftSensor");
        colorSensorRight = hMap.get(ColorRangeSensor.class, "rightSensor");


    }
    @Override
    public void register() {
        super.register();
    }

    @Override
    public void periodic() {

    }

    public void intake() {
        setPower(1.0);
    }

    public void outtake() {
        setPower(-1.0);
    }

    public void stop() {
        setPower(0.0);
    }
    public void slow(){
        setPower(-0.1);
    }

    public void setPower(double power) {
        intakeMotor.set(power);
    }
}

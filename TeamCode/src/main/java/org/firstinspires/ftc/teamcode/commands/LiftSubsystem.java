package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class LiftSubsystem extends SubsystemBase {

    public Motor leftMotor;
    public Motor rightMotor;
    public Servo leftServo;
    public Servo rightServo;
    public Servo gripper;
    public Servo gripperNotif;
    public Servo pixelNotif;

    public MotorGroup liftGroup;
    Telemetry telemetry;
    public static double multiplier = 6;
    public static int top = 700;
    public static int bottom = 300;
    public static double pCoefficient = 0.06;
    public static double motorPower = 0.3;
    public static double servoFlipPos = 0.4;
    public static double gripperNotifBottom = 0;
    public static double gripperNotifTop = 0.5;
    public static double pixelNotifBottom = 0;
    public static double pixelNotifTop = 0.5;
    double target = 0;
    public double liftOffset = 0;
    private double pitch = 0;

    ToggleButtonReader xReader;
    ToggleButtonReader bumperReader;
    ButtonReader upReader;
    ButtonReader downReader;

    ColorRangeSensor colorSensorLeft;
    ColorRangeSensor colorSensorRight;

    public enum LiftPos{
        UP,
        DOWN
    }
    public LiftSubsystem(HardwareMap hMap, Telemetry telemetry){
        this.telemetry = telemetry;

        leftServo = hMap.get(Servo.class, "leftServo");
        rightServo = hMap.get(Servo.class, "rightServo");
        gripper = hMap.get(Servo.class, "gripper");
        gripperNotif = hMap.get(Servo.class, "gripperNotif");
        pixelNotif = hMap.get(Servo.class, "pixelNotif");
        colorSensorLeft = hMap.get(ColorRangeSensor.class, "leftSensor");
        colorSensorRight = hMap.get(ColorRangeSensor.class, "rightSensor");


        leftMotor = new MotorEx(hMap,"leftLift", Motor.GoBILDA.RPM_312);
        rightMotor = new MotorEx(hMap, "rightLift", Motor.GoBILDA.RPM_312);
        rightMotor.setInverted(true);
        leftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        leftMotor.setRunMode(Motor.RunMode.PositionControl);
        rightMotor.setRunMode(Motor.RunMode.PositionControl);

        leftMotor.resetEncoder();
        rightMotor.resetEncoder();

        leftMotor.setTargetPosition(0);
        rightMotor.setTargetPosition(0);
        leftMotor.setDistancePerPulse(1);
        rightMotor.setDistancePerPulse(1);
    }

    @Override
    public void periodic(){
        telemetry.addData("target", target);
        leftMotor.setPositionCoefficient(pCoefficient);
        rightMotor.setPositionCoefficient(pCoefficient);

        telemetry.addData("leftSensor", colorSensorLeft.getDistance(DistanceUnit.CM));
        telemetry.addData("rightSensor", colorSensorRight.getDistance(DistanceUnit.CM));

        target = Range.clip(target, 0 + liftOffset, 900 + liftOffset);
        leftMotor.setTargetDistance(target);
        rightMotor.setTargetDistance(target);
        leftMotor.set(motorPower);
        rightMotor.set(motorPower);

        if(colorSensorLeft.getDistance(DistanceUnit.CM) < 2 && colorSensorRight.getDistance(DistanceUnit.CM) < 2)
            pixelNotif.setPosition(pixelNotifTop);
        else
            pixelNotif.setPosition(pixelNotifBottom);

        if(leftMotor.getDistance() >= 700){
            leftServo.setPosition(servoFlipPos + (pitch*0.1));
            rightServo.setPosition(1.0 - (servoFlipPos +(pitch*0.1)));
        } else {
            leftServo.setPosition(0.05 + (pitch*0.05));
            rightServo.setPosition(1.0 - (0.05 + pitch*0.05));
        }
    }

    public void closeGripper(){
        gripper.setPosition(0.9);
        gripperNotif.setPosition(gripperNotifBottom);
    }

    public void openGripper(){
        gripper.setPosition(1);
        gripperNotif.setPosition(gripperNotifTop);
    }

    public void updateTarget(double value, double pitch){
        target += value * multiplier;
        this.pitch = pitch;
    }
}

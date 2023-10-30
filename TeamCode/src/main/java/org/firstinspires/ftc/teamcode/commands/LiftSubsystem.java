package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
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
    public static double pCoefficient = 0.06;
    public static double motorPower = 0.3;
    public static double servoFlipPos = 0.4;
    public static double gripperNotifBottom = 0.4;
    public static double gripperNotifTop = 0;
    public static double pixelNotifBottom = 0.5;
    public static double pixelNotifTop = 0.9;
    double target = 0;
    public double liftOffset = 0;
    private double pitch = 0;
    private PIDFController pidf;
    public static double kP = 0.05;
    public static double kI = 0.001;
    public static double kD = 0.02;
    public static double kF = 0.05;
    public static boolean disable = true;
    public boolean climbGrab = false;
    public boolean climbAttach = false;
    public static double grabPos = 0.5;
    public static double attachPos = 1;

    ColorRangeSensor colorSensorLeft;
    ColorRangeSensor colorSensorRight;
    public static double sensorDistance = 1; //cm

    public enum LiftPos{
        UP,
        DOWN
    }
    public LiftSubsystem(HardwareMap hMap, Telemetry telemetry){
        this.telemetry = telemetry;

        pidf = new PIDFController(kP, kI, kD, kF);

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
        leftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftMotor.setRunMode(Motor.RunMode.RawPower);
        rightMotor.setRunMode(Motor.RunMode.RawPower);

        leftMotor.resetEncoder();
        rightMotor.resetEncoder();
    }

    @Override
    public void periodic(){
        pidf.setPIDF(kP, kI, kD, kF);

        telemetry.addData("target", target);
        telemetry.addData("leftSensor", colorSensorLeft.getDistance(DistanceUnit.CM));
        telemetry.addData("rightSensor", colorSensorRight.getDistance(DistanceUnit.CM));

        target = Range.clip(target, 0 + liftOffset, 900 + liftOffset);

        double power = pidf.calculate(leftMotor.getCurrentPosition(), target);
        telemetry.addData("power", power);
        if(!disable){
            leftMotor.set(power);
            rightMotor.set(power);
        } else{
            leftMotor.set(0);
            rightMotor.set(0);
        }

        if(colorSensorLeft.getDistance(DistanceUnit.CM) < sensorDistance && colorSensorRight.getDistance(DistanceUnit.CM) < sensorDistance)
            pixelNotif.setPosition(pixelNotifTop);
        else
            pixelNotif.setPosition(pixelNotifBottom);

        if(climbGrab){
            tilt(grabPos + (pitch*0.05));
        } else if (climbAttach) {
            tilt(attachPos + (pitch*0.05));
        } else if(leftMotor.getDistance() >= 700){
            tilt(servoFlipPos + (pitch*0.1));
        } else {
            tilt(0.05 + (pitch*0.05));
        }
        telemetry.update();
    }

    public void tilt(double pitch){
        leftServo.setPosition(pitch);
        rightServo.setPosition(1.0 - pitch);
    }

    public void top(){
        target = top+liftOffset;
    }

    public void bottom(){
        top = (int) target;
        target = liftOffset;
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

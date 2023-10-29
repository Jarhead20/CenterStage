package org.firstinspires.ftc.teamcode;

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
public class LiftSubSystem extends SubsystemBase {

    public Motor leftMotor;
    public Motor rightMotor;
    public Motor intake;
    public Servo leftServo;
    public Servo rightServo;
    public Servo gripper;
    public Servo gripperNotif;

    public MotorGroup liftGroup;
    Telemetry telemetry;
    public static double multiplier = 6;
    public static int top = 700;
    public static int bottom = 300;
    public static double pCoefficient = 0.06;
    public static double motorPower = 0.3;
    public static double servoFlipPos = 0.4;
    public static double servoNotifBottom = 0;
    public static double servoNotifTop = 0.5;
    double target = 0;
    GamepadEx gamepadEx;
    public double liftOffset = 0;

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
    public LiftSubSystem(String leftLift, String rightLift, HardwareMap hMap, Telemetry telemetry, GamepadEx gamepadEx){

        this.gamepadEx = gamepadEx;
        this.telemetry = telemetry;

        intake = new Motor(hMap, "intake");
        leftServo = hMap.get(Servo.class, "leftServo");
        rightServo = hMap.get(Servo.class, "rightServo");
        gripper = hMap.get(Servo.class, "gripper");
        gripperNotif = hMap.get(Servo.class, "gripperNotif");
        colorSensorLeft = hMap.get(ColorRangeSensor.class, "leftSensor");
        colorSensorRight = hMap.get(ColorRangeSensor.class, "rightSensor");

        leftMotor = new MotorEx(hMap,leftLift, Motor.GoBILDA.RPM_312);
        rightMotor = new MotorEx(hMap, rightLift, Motor.GoBILDA.RPM_312);
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



        xReader = new ToggleButtonReader(gamepadEx, GamepadKeys.Button.X);
        bumperReader = new ToggleButtonReader(gamepadEx, GamepadKeys.Button.RIGHT_BUMPER);
        upReader = new ButtonReader(gamepadEx, GamepadKeys.Button.DPAD_UP);
        downReader = new ButtonReader(gamepadEx, GamepadKeys.Button.DPAD_DOWN);


    }

    public void update(){
        telemetry.addData("target", target);
        leftMotor.setPositionCoefficient(pCoefficient);
        rightMotor.setPositionCoefficient(pCoefficient);

        telemetry.addData("leftSensor", colorSensorLeft.getDistance(DistanceUnit.CM));
        telemetry.addData("rightSensor", colorSensorRight.getDistance(DistanceUnit.CM));

        if(gamepadEx.getButton(GamepadKeys.Button.LEFT_BUMPER)){
            target += gamepadEx.getLeftY() * multiplier;
            if(upReader.wasJustPressed()){
                liftOffset += 5;
            }
            if(downReader.wasJustPressed()){
                liftOffset -= 5;
            }
            target = Range.clip(target, 0 + liftOffset, 900 + liftOffset);
            leftMotor.setTargetDistance(target);
            rightMotor.setTargetDistance(target);
        }

        leftMotor.set(motorPower);
        rightMotor.set(motorPower);

        if(bumperReader.getState()) {
            gripper.setPosition(0.9);
            gripperNotif.setPosition(servoNotifBottom);
        }
        else {
            gripper.setPosition(1);
            gripperNotif.setPosition(servoNotifTop);
        }
        stopIntake();
        if(gamepadEx.getButton(GamepadKeys.Button.X)) collect();
        if(gamepadEx.getButton(GamepadKeys.Button.Y)) eject();

        if(leftMotor.getDistance() >= 700){
            leftServo.setPosition(servoFlipPos + (gamepadEx.getRightY()*0.1));
            rightServo.setPosition(1.0 - (servoFlipPos +(gamepadEx.getRightY()*0.1)));
        } else {
            leftServo.setPosition(0.05 + (gamepadEx.getRightY()*0.05));
            rightServo.setPosition(1.0 - (0.05 + gamepadEx.getRightY()*0.05));
        }

        xReader.readValue();
        bumperReader.readValue();
        upReader.readValue();
        downReader.readValue();
    }

    public void eject(){
        intake.set(1);
    }

    public void stopIntake(){
        intake.set(0);
    }

    public void collect(){
        intake.set(-1);
    }

    public void lift(LiftPos pos){
        switch (pos){
            case UP:
//                target = top;
//                leftMotor.set(1);
//                rightMotor.set(1);
//                liftOffset += 5;
                break;
            case DOWN:
//                target = bottom;
//                liftOffset -= 5;
//                leftMotor.set(1);
//                rightMotor.set(1);
                break;
            default:
                break;
        }
    }

}

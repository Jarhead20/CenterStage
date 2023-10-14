package org.firstinspires.ftc.teamcode;

import android.telecom.TelecomManager;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.HardwareDevice;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class LiftSubSystem extends SubsystemBase {

    public Motor leftMotor;
    public Motor rightMotor;
    public Motor intake;
    public MotorGroup liftGroup;
    Telemetry telemetry;
    public static double multiplier = 3;
    public static int top = 700;
    public static int bottom = 400;
    public static double pCoefficient = 0.06;
    public static double motorPower = 0.5;
    double target = 0;
    GamepadEx gamepadEx;

    public enum LiftPos{
        UP,
        DOWN
    }
    public LiftSubSystem(String leftLift, String rightLift, HardwareMap hMap, Telemetry telemetry, GamepadEx gamepadEx){
        leftMotor = new MotorEx(hMap,leftLift, Motor.GoBILDA.RPM_312);
        rightMotor = new MotorEx(hMap, rightLift, Motor.GoBILDA.RPM_312);
        rightMotor.setInverted(true);
        leftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        leftMotor.setRunMode(Motor.RunMode.PositionControl);
        rightMotor.setRunMode(Motor.RunMode.PositionControl);

        leftMotor.resetEncoder();
        rightMotor.resetEncoder();
        this.gamepadEx = gamepadEx;
        leftMotor.setTargetPosition(0);
        rightMotor.setTargetPosition(0);
        leftMotor.setDistancePerPulse(1);
        rightMotor.setDistancePerPulse(1);
        intake = new Motor(hMap, "intake");

//        leftMotor.set(0.05);
//        rightMotor.set(0.05);
        this.telemetry = telemetry;

    }

    public void printPos(){

        telemetry.addData("leftLift", leftMotor.getDistance() + " " + leftMotor.get());
        telemetry.addData("rightLift",rightMotor.getDistance() + " " + rightMotor.get());

        telemetry.addData("target", target);
        leftMotor.setPositionCoefficient(pCoefficient);
        rightMotor.setPositionCoefficient(pCoefficient);

        if(gamepadEx.getButton(GamepadKeys.Button.LEFT_BUMPER)){
            target += gamepadEx.getLeftY() * multiplier;
            leftMotor.setTargetDistance(target);
            rightMotor.setTargetDistance(target);

        }
        else {
            if(gamepadEx.getButton(GamepadKeys.Button.A)){
                lift(LiftPos.UP);
            } else if (gamepadEx.getButton(GamepadKeys.Button.B)) {
                lift(LiftPos.DOWN);
            }
        }
        leftMotor.set(motorPower);
        rightMotor.set(motorPower);



//        moveLeft((int) target);
    }

    public void lift(LiftPos pos){
        switch (pos){
            case UP:
                leftMotor.setTargetDistance(top);
                rightMotor.setTargetDistance(top);
//                leftMotor.set(1);
//                rightMotor.set(1);
                break;
            case DOWN:
                leftMotor.setTargetDistance(bottom);
                rightMotor.setTargetDistance(bottom);
//                leftMotor.set(1);
//                rightMotor.set(1);
                break;
            default:
                break;
        }
    }

}

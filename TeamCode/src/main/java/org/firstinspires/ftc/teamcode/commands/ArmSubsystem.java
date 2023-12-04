package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class ArmSubsystem extends SubsystemBase {
    Servo leftArmServo;
    Servo rightArmServo;
    Servo wristServo;
    Servo leftGripper;
    Servo rightGripper;

    public static double leftGripperOpen = 0;
    public static double rightGripperOpen = 0;
    public static double leftGripperClosed = 0.8;
    public static double rightGripperClosed = 0.9;
    public static double servoFlipPos = 0.4;
    public static double armAngle = 0.24;
    public static double wristAngle = 0.9;
    public static double leftOffset = 0.0;
    public static double rightOffset = 0.0;
    public static double armOffset = 0.0;
    public static double wristOffset = 0.0;

    public static boolean gripperOpen;
    public static double minimumSlidePos = 300.0;
    public static double minimumArmAngle = 0.25;

    public ArmSubsystem(HardwareMap hMap){
        leftArmServo = hMap.get(Servo.class, "leftArm");
        rightArmServo = hMap.get(Servo.class, "rightArm");
        wristServo = hMap.get(Servo.class, "wrist");
        leftGripper = hMap.get(Servo.class, "leftGripper");
        rightGripper = hMap.get(Servo.class, "rightGripper");
        rightArmServo.setDirection(Servo.Direction.REVERSE);
        rightGripper.setDirection(Servo.Direction.REVERSE);
        gripperOpen = false;
    }

    public void tilt(double pitch){
        leftArmServo.setPosition(pitch + leftOffset);
        rightArmServo.setPosition(pitch + rightOffset);
    }

    @Override
    public void periodic() {
        tilt(armAngle+armOffset);
        wristServo.setPosition(wristAngle+wristOffset);
    }

    public void update(double armOffset, double wristOffset){
        this.armOffset = armOffset;
        this.wristOffset = wristOffset;
        tilt(armAngle+armOffset);
        wristServo.setPosition(wristAngle+wristOffset);
    }

    public void closeLeftGripper(){leftGripper.setPosition(leftGripperClosed);}

    public void openLeftGripper(){
        leftGripper.setPosition(leftGripperOpen);
        gripperOpen = true;
    }
    public void closeRightGripper(){
        rightGripper.setPosition(rightGripperClosed);
    }

    public void openRightGripper(){
        rightGripper.setPosition(rightGripperOpen);
        gripperOpen = true;
    }
    public void closeBothGrippers(){
        closeLeftGripper();
        closeRightGripper();
        gripperOpen = false;
    }

    public boolean goingToCollide(double slidePos){
        return (slidePos < minimumSlidePos && armAngle < minimumArmAngle);
    }

}

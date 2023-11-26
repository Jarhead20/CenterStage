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

    public static double leftGripperOpen = 0.5;
    public static double rightGripperOpen = 0.5;
    public static double leftGripperClosed = 0.0;
    public static double rightGripperClosed = 0.0;
    public static double servoFlipPos = 0.4;
    public double armAngle = 0;
    public double wristAngle = 0;

    public ArmSubsystem(HardwareMap hMap){
        leftArmServo = hMap.get(Servo.class, "leftArm");
        rightArmServo = hMap.get(Servo.class, "rightArm");
        wristServo = hMap.get(Servo.class, "wrist");
        leftGripper = hMap.get(Servo.class, "leftGripper");
        rightGripper = hMap.get(Servo.class, "rightGripper");
        rightArmServo.setDirection(Servo.Direction.REVERSE);
        rightGripper.setDirection(Servo.Direction.REVERSE);
    }

    public void tilt(double pitch){
        leftArmServo.setPosition(pitch);
        rightArmServo.setPosition(pitch);
    }

    @Override
    public void periodic() {
        tilt(armAngle);
        wristServo.setPosition(wristAngle);
    }

    public void update(){
        tilt(armAngle);
        wristServo.setPosition(wristAngle);
    }

    public void closeLeftGripper(){leftGripper.setPosition(leftGripperClosed);}

    public void openLeftGripper(){
        leftGripper.setPosition(leftGripperOpen);
    }

    public void closeRightGripper(){
        rightGripper.setPosition(rightGripperClosed);
    }

    public void openRightGripper(){
        rightGripper.setPosition(rightGripperOpen);
    }

}

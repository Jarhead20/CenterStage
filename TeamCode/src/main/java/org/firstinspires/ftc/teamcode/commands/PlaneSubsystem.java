package org.firstinspires.ftc.teamcode.commands;

import android.widget.ToggleButton;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class PlaneSubsystem extends SubsystemBase {

    Servo servo;
    public static double planeMin = 0.2;
    public static double planeMax = 0.7;

    public PlaneSubsystem(HardwareMap hMap){
        servo = hMap.get(Servo.class, "plane");
        servo.setPosition(planeMin);
    }
    public void launch(){
        servo.setPosition(planeMax);
    }

    public void reset(){
        servo.setPosition(planeMin);
    }



}

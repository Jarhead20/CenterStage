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
    public static double planeMin = 0.3;
    public static double planeMax = 1;
    public static boolean toggle = false;

    public PlaneSubsystem(HardwareMap hMap){
        servo = hMap.get(Servo.class, "plane");
        servo.setPosition(planeMin);
    }
    public void toggle(){
        toggle = !toggle;

    }

    @Override
    public void periodic(){
        if(toggle)
            servo.setPosition(planeMin);
        else
            servo.setPosition(planeMax);
    }



}

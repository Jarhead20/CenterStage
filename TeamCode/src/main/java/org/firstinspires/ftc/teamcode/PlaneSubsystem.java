package org.firstinspires.ftc.teamcode;

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
    GamepadEx gamepadEx;
    public static double planeMin = 0.2;
    public static double planeMax = 0.75;
    ToggleButtonReader backReader;

    public PlaneSubsystem(HardwareMap hMap, GamepadEx gamepadEx){
        servo = hMap.get(Servo.class, "plane");
        this.gamepadEx = gamepadEx;
        servo.setPosition(planeMin);
        backReader = new ToggleButtonReader(gamepadEx, GamepadKeys.Button.BACK);
    }

    public void update(){
        if(backReader.getState()){
            servo.setPosition(planeMax);
        } else {
            servo.setPosition(planeMin);
        }

        backReader.readValue();
    }

}

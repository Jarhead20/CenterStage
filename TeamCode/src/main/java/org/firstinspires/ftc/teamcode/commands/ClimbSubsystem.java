package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ClimbSubsystem extends SubsystemBase {
    private Motor climbMotor;
    public ClimbSubsystem(HardwareMap hMap){
        climbMotor = new Motor(hMap, "climb");
    }

    public void control(double power){
        climbMotor.set(power);
    }
}

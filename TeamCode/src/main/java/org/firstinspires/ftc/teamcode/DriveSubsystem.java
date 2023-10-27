package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveSubsystem extends SubsystemBase {
    MecanumDrive drive;
    private final MotorEx fL;
    private final MotorEx fR;
    private final MotorEx bL;
    private final MotorEx bR;
    private GamepadEx gamepadEx;
    public DriveSubsystem(final HardwareMap hMap, GamepadEx gamepadEx){
        fL = new MotorEx(hMap, "frontLeft", Motor.GoBILDA.RPM_312);
        fR = new MotorEx(hMap, "frontRight", Motor.GoBILDA.RPM_312);
        bL = new MotorEx(hMap, "backLeft", Motor.GoBILDA.RPM_312);
        bR = new MotorEx(hMap, "backRight", Motor.GoBILDA.RPM_312);
        drive = new MecanumDrive(fL, fR, bL, bR);
        this.gamepadEx = gamepadEx;
    }

    public void drive(){
        drive(-gamepadEx.getLeftX(), -gamepadEx.getLeftY(), -gamepadEx.getRightX(), 0.5);
    }

    public void drive(double forward, double strafe, double rotation, double speed){
        drive.driveRobotCentric(forward*speed, strafe*speed, rotation*speed);
    }
}
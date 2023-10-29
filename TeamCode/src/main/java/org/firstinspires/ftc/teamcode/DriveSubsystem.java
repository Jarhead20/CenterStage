package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
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
        drive(-gamepadEx.getLeftX(), -gamepadEx.getLeftY(), -gamepadEx.getRightX(), gamepadEx.getButton(GamepadKeys.Button.RIGHT_BUMPER) ? 0.5 : 1);
    }

    public void autoDrive(double speed){
        drive(0, 1, 0, speed);
    }

    public void drive(double strafe, double forward, double rotation, double speed){
        drive.driveRobotCentric(strafe*speed, forward*speed, rotation*speed);
    }
}
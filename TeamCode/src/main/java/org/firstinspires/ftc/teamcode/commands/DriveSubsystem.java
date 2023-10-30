package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class DriveSubsystem extends SubsystemBase {
    MecanumDrive drive;
    private final MotorEx fL;
    private final MotorEx fR;
    private final MotorEx bL;
    private final MotorEx bR;
    public static double WHEEL_DIAMETER = 10.33; //cm effective wheel diameter with 26:28 ratio on the dt motors

    private final Motor.Encoder fLE, fRE, bLE, bRE;
    public DriveSubsystem(final HardwareMap hMap){
        fL = new MotorEx(hMap, "frontLeft", Motor.GoBILDA.RPM_312);
        fR = new MotorEx(hMap, "frontRight", Motor.GoBILDA.RPM_312);
        bL = new MotorEx(hMap, "backLeft", Motor.GoBILDA.RPM_312);
        bR = new MotorEx(hMap, "backRight", Motor.GoBILDA.RPM_312);

        fLE = fL.encoder;
        fRE = fR.encoder;
        bLE = bL.encoder;
        bRE = bR.encoder;
        drive = new MecanumDrive(fL, fR, bL, bR);
    }

    public void resetEncoders(){
        fL.resetEncoder();
        fR.resetEncoder();
        bL.resetEncoder();
        bR.resetEncoder();
    }

    public double getAverageEncoderDistance() {
        return (((fLE.getPosition() + fRE.getPosition() + bLE.getPosition() + bRE.getPosition()) / 4.0) * WHEEL_DIAMETER * Math.PI)/fL.getCPR();
    }

    public void autoDrive(double speed){
        drive(0, 1, 0, speed);
    }

    public void drive(double strafe, double forward, double rotation, double speed){
        drive.driveRobotCentric(strafe*speed, forward*speed, rotation*speed);
    }
}
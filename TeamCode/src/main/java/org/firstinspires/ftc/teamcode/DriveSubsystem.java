package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveSubsystem extends SubsystemBase {
    MecanumDrive drive;
    private MotorEx fL, fR, bL, bR;
    public DriveSubsystem(final HardwareMap hMap){
        fL = new MotorEx(hMap, "fL", Motor.GoBILDA.RPM_312);
        fR = new MotorEx(hMap, "fR", Motor.GoBILDA.RPM_312);
        bL = new MotorEx(hMap, "bL", Motor.GoBILDA.RPM_312);
        bR = new MotorEx(hMap, "bR", Motor.GoBILDA.RPM_312);
        drive = new MecanumDrive(fL, fR, bL, bR);
    }

    public void drive(double forward, double strafe, double rotation){
        drive.driveRobotCentric(forward, strafe, rotation);
    }
}
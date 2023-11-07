package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem extends SubsystemBase {
    private final Motor intakeMotor;
    public IntakeSubsystem(HardwareMap hMap){
        intakeMotor = new Motor(hMap, "intake");
        intakeMotor.setInverted(true);
    }
    @Override
    public void register() {
        super.register();
    }

    @Override
    public void periodic() {

    }

    public void intake() {
        setPower(1.0);
    }

    public void outtake() {
        setPower(-1.0);
    }

    public void stop() {
        setPower(0.0);
    }
    public void slow(){
        setPower(-0.1);
    }

    public void setPower(double power) {
        intakeMotor.set(power);
    }
}

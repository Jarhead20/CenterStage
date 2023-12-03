package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Config
public class IntakeSubsystem extends SubsystemBase {
    private final Motor intakeMotor;

    public ColorRangeSensor colorSensorLeft;
    public ColorRangeSensor colorSensorRight;
    ElapsedTime rumbleTimer;

    public static double rumbleWaitTimeMS = 1000;


    public IntakeSubsystem(HardwareMap hMap){
        intakeMotor = new Motor(hMap, "intake");
        intakeMotor.setInverted(true);

        colorSensorLeft = hMap.get(ColorRangeSensor.class, "leftSensor");
        colorSensorRight = hMap.get(ColorRangeSensor.class, "rightSensor");
        rumbleTimer = new ElapsedTime();

    }
    @Override
    public void register() {
        super.register();
    }

    @Override
    public void periodic() {
    }
    public boolean pixelsReady(){
        boolean sensors = colorSensorLeft.getDistance(DistanceUnit.CM) < 1 && colorSensorRight.getDistance(DistanceUnit.CM) < 1;
        if(sensors && (rumbleTimer.milliseconds() > rumbleWaitTimeMS)){
            rumbleTimer.reset();
            return true;
        }
        return false;
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

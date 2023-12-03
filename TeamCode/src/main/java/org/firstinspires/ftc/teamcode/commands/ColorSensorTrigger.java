package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorSensorTrigger extends Trigger {

    ColorRangeSensor sensor;
    private double distance;

    /**
     * @param sensor the color sensor to use
     * @param distance the distance to trigger at (CM)
     */
    public ColorSensorTrigger(ColorRangeSensor sensor, double distance){
        this.sensor = sensor;
        this.distance = distance;
    }
    @Override
    public boolean get() {
        return sensor.getDistance(DistanceUnit.CM) < distance;
    }
}

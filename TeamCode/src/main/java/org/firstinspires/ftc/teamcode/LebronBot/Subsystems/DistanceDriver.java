package org.firstinspires.ftc.teamcode.LebronBot.Subsystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceDriver {
    private DistanceSensor sensor;
    private double cachedDistance = -1;
    private long lastRead = 0;
    private int cycles = 0;

    public DistanceDriver (DistanceSensor sensor){
        this.sensor = sensor;
    }
    public double getDistance(DistanceUnit unit){
        if(System.currentTimeMillis() - lastRead > 33){
            cachedDistance = sensor.getDistance(DistanceUnit.INCH);
            lastRead = System.currentTimeMillis();
        }
        return unit.fromMm(cachedDistance);
    }

    public long getLastReadTimestamp() {
        return lastRead;
    }
}


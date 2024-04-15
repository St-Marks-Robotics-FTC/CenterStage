package org.firstinspires.ftc.teamcode.LebronBot.Subsystems;

import android.util.Log;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceDriver {
    private AnalogInput sensor;
    private double cachedDistance = -1;
    private long lastRead = 0;
    private int cycles = 0;

    public DistanceDriver (HardwareMap hardwareMap, String sensor){
        this.sensor= hardwareMap.get(AnalogInput.class, sensor);
    }
    public double getDistance(){
        if(System.currentTimeMillis() - lastRead > 100){
            double volt = sensor.getVoltage();
            if (true) cachedDistance = ((volt*6)/((2.74/1024)))-300; //increasing vcc makes less "extreme"
            lastRead = System.currentTimeMillis();
        }
        return (cachedDistance);
    }
    public double getVoltage() {
        return sensor.getVoltage();
    }

    public long getLastReadTimestamp() {
        return lastRead;
    }
}


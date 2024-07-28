package org.firstinspires.ftc.teamcode.LebronBot.PurePursuit;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MotorCache {

    private final double tolerance;
    private double lastPower = 0;
    private DcMotorEx motor;

    public MotorCache(DcMotorEx motor) {
        this(motor, 0.001);
    }

    public MotorCache(DcMotorEx motor, double threshold) {
        this.motor=motor;
        tolerance = threshold;
    }
    public void setPower(double power) {
        if (Math.abs(power - lastPower) > tolerance) {
            motor.setPower(power);
            lastPower = power;
        }
    }
}

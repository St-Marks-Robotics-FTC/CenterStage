package org.firstinspires.ftc.teamcode.LebronBot.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class SpecialTeams {


    Servo drone;


    // Drone
    public static double holdPos = 0.0;
    public static double releasePos = 0.5;

    public SpecialTeams (HardwareMap hardwareMap) {
//        drone = hardwareMap.get(Servo.class, "drone");

    }


    // Drone
    public void holdDrone() {
        drone.setPosition(holdPos);
    }

    public void shootDrone() {
        drone.setPosition(releasePos);
    }
}
package org.firstinspires.ftc.teamcode.jankbot.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class SpecialTeams {

    DcMotorEx hang;
    Servo drone;

    // Hang
    public static int hangPrepare = 700;
    public static int hangHold = 1200;
    public static double hangPower = 1.0;

    // Drone
    public static double holdPos = 0.0;
    public static double releasePos = 0.5;

    public SpecialTeams (HardwareMap hardwareMap) {
        hang = hardwareMap.get(DcMotorEx.class, "hang");
        drone = hardwareMap.get(Servo.class, "drone");

        hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Hang
    public void hangReady() {
        hang.setTargetPosition(hangPrepare);
        hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hang.setPower(hangPower);
    }

    public void hangHold() {
        hang.setTargetPosition(hangHold);
        hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hang.setPower(hangPower);
    }

    public int getHangPos() {
        return hang.getCurrentPosition();
    }


    // Drone
    public void holdDrone() {
        drone.setPosition(holdPos);
    }

    public void releaseDrone() {
        drone.setPosition(releasePos);
    }
}

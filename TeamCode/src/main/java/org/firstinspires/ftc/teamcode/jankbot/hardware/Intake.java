package org.firstinspires.ftc.teamcode.jankbot.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.teamcode.jankbot.util.ActionQueue;

@Config
public class Intake {

    public DcMotorEx intake;
    public Servo dropdown;
    public Servo tilt;
    public Servo lock;


    public static double dropDown = 0.5;
    public static double dropUp = 0.0;

    public static double transferLock = 0.5;
    public static double transferUnlock = 0.3;

    public static double transferUp = 0.4;
    public static double transferDown=0.2;


    public Intake(HardwareMap hardwareMap) {
        //intake
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        dropdown = hardwareMap.get(Servo.class, "dropdown");
        //transfer
        tilt = hardwareMap.get(Servo.class, "tilt");
        lock = hardwareMap.get(Servo.class, "lock");


        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }


    // Intake Motor
    public void setIntake(double power) {
        intake.setPower(power);
    }

    public double getPower() {
        return intake.getPower();
    }

    public double getVelocity() {
        return intake.getVelocity();
    }

    // Intake Dropdown
    public void dropdown() {
        dropdown.setPosition(dropDown);
    }
    public void raise() {
        dropdown.setPosition(dropUp);
    }


    // Transfer Lock
    public void lock() {
        lock.setPosition(transferLock);
    }

    public void unlock() {
        lock.setPosition(transferUnlock);
    }






    // Transfer Tilt
    public void tiltUp() {
        tilt.setPosition(transferUp);
    }

    public void tiltDown() {
        tilt.setPosition(transferDown);
    }


}

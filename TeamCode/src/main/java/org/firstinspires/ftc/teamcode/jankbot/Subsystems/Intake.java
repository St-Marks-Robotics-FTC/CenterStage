package org.firstinspires.ftc.teamcode.jankbot.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.teamcode.jankbot.util.ActionQueue;

@Config
public class Intake {

    public DcMotorEx intake;
    public Servo tilt1;
    public Servo tilt2;


    public static double dropDown = 0.5;
    public static double dropUp = 0.0;


    public static double transferUp = 0.4;
    public static double transferDown=0.2;
    public static double stackPos = 0.3;


    public Intake(HardwareMap hardwareMap) {
        //intake
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        //transfer
        tilt1 = hardwareMap.get(Servo.class, "tilt1");
        tilt2 = hardwareMap.get(Servo.class, "tilt2");


        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tilt2.setDirection(Servo.Direction.REVERSE);
    }


    // Intake Motor
    public void setIntake(double power) {
        //half power because vivek says the motors are too fast
        intake.setPower(power/2);
    }

    public double getPower() {
        return intake.getPower();
    }

    public double getVelocity() {
        return intake.getVelocity();
    }


    // Transfer Tilt
    public void tiltUp() {
        tilt1.setPosition(transferUp);
        tilt2.setPosition(transferUp);
    }

    public void tiltDown() {
        tilt1.setPosition(transferDown);
        tilt2.setPosition(transferDown);
    }

    public void tiltStack() {
        tilt1.setPosition(stackPos);
        tilt2.setPosition(stackPos);
    }


}

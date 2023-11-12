package org.firstinspires.ftc.teamcode.armbot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {

    public DcMotorEx arm;
    public Servo claw;
    public Servo autoClaw;

    private double target = 0.0;

    public Arm(HardwareMap hardwareMap) {
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");
        autoClaw = hardwareMap.get(Servo.class, "autoClaw");

        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean clawState() {
        return target==0.3 ? true : false;
    }
    public void closeClaw() {
        claw.setPosition(0.41);
        target = 0.3;
    }

    public void openAutoClaw() {
        autoClaw.setPosition(0.5);
    }
    public void openClaw() {
        claw.setPosition(0.35);
        target = 0.7;
    }

    public void setArm(int pos) {
        arm.setTargetPosition(pos);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.45);
    }

}

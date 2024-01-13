package org.firstinspires.ftc.teamcode.jankbot.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class JankOuttake {
    DcMotorEx arm;
    Servo clawLeft;
    Servo clawRight;

    private static double leftClose=0.65;
    private static double rightClose=0.34; // .34
    private static double leftOpen=0.8;
    private static double rightOpen =0.25;

    //starts up
    private static int armUp=-615;
    private static int armDown=-904;

    public JankOuttake(HardwareMap hardwareMap) {
        arm=hardwareMap.get(DcMotorEx.class, "arm");
        clawLeft=hardwareMap.get(Servo.class, "clawLeft");
        clawRight=hardwareMap.get(Servo.class, "clawRight");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        arm.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void closeBoth() {
        clawLeft.setPosition(leftClose);
        clawRight.setPosition(rightClose);
    }

    public void openBoth() {
        clawLeft.setPosition(leftOpen);
        clawRight.setPosition(rightOpen);
    }
    public void closeLeft() {
        clawLeft.setPosition(leftClose);
    }

    public void closeRight() {
        clawRight.setPosition(rightClose);
    }

    public void openLeft() {
        clawLeft.setPosition(leftOpen);
    }

    public void openRight() {
        clawRight.setPosition(rightOpen);
    }

    public void scoreArm() {
        arm.setTargetPosition(armUp);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.5);
    }
    public void armDown() {
        arm.setTargetPosition(armDown);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.5);
    }
}

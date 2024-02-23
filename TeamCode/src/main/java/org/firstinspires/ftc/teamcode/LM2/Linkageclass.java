package org.firstinspires.ftc.teamcode.LM2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Linkageclass {

    public DcMotorEx arm;
    public Servo clawLeft;
    public Servo clawRight;

    public Servo wrist; //0.35 for default 0 is flat
    public Servo linkageLeft;
    public Servo linkageRight;

    public Servo drone;

    public TouchSensor sensorLeft;
    public TouchSensor sensorRight;

    private double target = 0.0;
    private int prev;

    public Linkageclass(HardwareMap hardwareMap) {
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        //reversed on accident
        clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        clawRight = hardwareMap.get(Servo.class, "clawRight");

        wrist = hardwareMap.get(Servo.class, "wrist");
        linkageLeft = hardwareMap.get(Servo.class, "linkageLeft");
        linkageRight = hardwareMap.get(Servo.class, "linkageRight");

        linkageRight.setDirection(Servo.Direction.REVERSE);

        drone = hardwareMap.get(Servo.class, "drone");

        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //sensorLeft = hardwareMap.get(TouchSensor.class, "sensorLeft");
        //sensorRight = hardwareMap.get(TouchSensor.class, "sensorRight");
    }

    public boolean clawState() {
        return target==0.3 ? true : false;
    }
    public void closeClaw() {
        closeLeft();
        closeRight();
        //target = 0.3;
    }

    public void openLeft() {
        clawLeft.setPosition(0.45);
    } // .79
    public void scoreLeft() {
        clawLeft.setPosition(0.45);
    }
    public void closeLeft() {
        clawLeft.setPosition(0.76);
    }

    public void openRight() {
        clawRight.setPosition(0.3);
    } // .35
    public void scoreRight() {
        clawRight.setPosition(0.3);
    }
    public void closeRight() {
        clawRight.setPosition(0.6);
    }




    public void openClaw() {
        openLeft();
        openRight();
        //target = 0.7;
    }

    public void scoreClaw() {
        scoreLeft();
        scoreRight();
        //target = 0.7;
    }

    public void holdDrone() {
        drone.setPosition(0.1);
    }

    public void shootDrone() {
        drone.setPosition(0.5);
    }
    public void hang(int pos){
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setPower(1);
    }
    public void setArm(int pos) {
        arm.setTargetPosition(pos);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower((pos>prev) ? 0.75 : 0.5);
        prev = pos;
    }

    public void zeroArm() {
        arm.setPower(-0.1);
    }

    public boolean detectLeft() {
        return false;
        //return sensorLeft.isPressed();
    }
    public boolean detectRight() {
        return false;
        //return sensorRight.isPressed();
    }

    public void setWrist(double pos) {
        wrist.setPosition(pos);
    } //0.35 for default 0 is flat

    public void wristPickup(){
        setWrist(0.35);
    }

    public void setLinkage(double pos) {
        linkageLeft.setPosition(pos);
        linkageRight.setPosition(pos);
        //0.25 is retract 1.0 is extend
    }

    public void retractLinkage() {
        setLinkage(0.25);
    }

}
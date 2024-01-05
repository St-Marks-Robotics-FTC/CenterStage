package org.firstinspires.ftc.teamcode.jankbot.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Outtake {

    public DcMotorEx leftSlide;
    public DcMotorEx rightSlide;

    public Servo v4barLeft;
    public Servo v4barRight;

    public Servo clawLeft;
    public Servo clawRight;

    public Servo turret;
    public Servo v4bpivot;

    public static int slidesDown = 0;
    public static int level1 = 300;
    public static int levelIncrement = 60;
    public static double slideDownPower=0.2;
    public static double slideUpPower=0.5;

    public static double v4barTransfer = 0.2;
    public static double v4barStow = 0.4;
    public static double v4barScore = 0.7;

    public static double clawLeftClosed = 0.0;
    public static double clawRightClosed = 0.0;
    public static double clawLeftOpen = 0.3;
    public static double clawRightOpen = 0.3;

    public static double turretTransfer = 0.5;
    public static double turret60 = 0.2;





    public static int TOLERANCE =10;



    public Outtake (HardwareMap hardwareMap) {
        //outtake
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

        v4barLeft = hardwareMap.get(Servo.class, "v4barLeft");
        v4barRight = hardwareMap.get(Servo.class, "v4barRight");

        clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        clawRight = hardwareMap.get(Servo.class, "clawRight");

        turret = hardwareMap.get(Servo.class, "turret");
        v4bpivot = hardwareMap.get(Servo.class, "pivot");




        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        v4barRight.setDirection(Servo.Direction.REVERSE);

        clawLeft.setDirection(Servo.Direction.REVERSE);

    }

    // Slides
    public void retractSlides() {
        setSlides(slidesDown);
    }

    public void slidesTo(int level) {
        setSlides(level1 +  (level - 1) * levelIncrement);
    }

    public boolean isDone() {
        return (!leftSlide.isBusy() && !rightSlide.isBusy());
    }

    public int getSlidePos() {
        return (leftSlide.getCurrentPosition()+ rightSlide.getCurrentPosition())/2;
    }

    public void setSlides(int pos) {
        rightSlide.setTargetPosition(pos);
        leftSlide.setTargetPosition(pos);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(getSlidePos() > pos ? slideUpPower : slideDownPower);
        rightSlide.setPower(getSlidePos() > pos ? slideUpPower : slideDownPower);
    }

    // V4Bar
    public void v4barTransfer() {
        setV4Bar(v4barTransfer);
    }
    public void v4barStow() {
        setV4Bar(v4barStow);
    }
    public void v4barScore() {
        setV4Bar(v4barScore);
    }


    public void setV4Bar(double pos) {
        v4barRight.setPosition(pos);
        v4barLeft.setPosition(pos);
    }


    // Claw
    public void closeClaw() {
        closeLeft();
        closeRight();
    }
    public void openClaw() {
        openLeft();
        openRight();
    }

    public void closeLeft() {
        clawLeft.setPosition(clawLeftClosed);
    }
    public void closeRight() {
        clawRight.setPosition(clawRightClosed);
    }
    public void openLeft() {
        clawLeft.setPosition(clawLeftOpen);
    }
    public void openRight() {
        clawRight.setPosition(clawRightOpen);
    }




    // Turret
    public void turretTransfer() {
        setTurret(turretTransfer);
    }
    public void turretLeft() {
        if (getTurret() == turretTransfer) {
            setTurret(getTurret() - turret60/2);
        } else {
            setTurret(getTurret() - turret60);
        }
    }
    public void turretRight() {
        if (getTurret() == turretTransfer) {
            setTurret(getTurret() + turret60/2);
        } else {
            setTurret(getTurret() + turret60);
        }
    }

    public void setTurret(double pos) {
        turret.setPosition(pos);
    }
    public double getTurret() {
        return turret.getPosition();
    }


    // Pivot
    public void sideAdjust(double degrees) {
        // TODO
    }

    public void setPivot(double pos) {
        v4bpivot.setPosition(pos);
    }


}

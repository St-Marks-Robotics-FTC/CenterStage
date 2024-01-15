package org.firstinspires.ftc.teamcode.jankbot.Subsystems;

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
    public Servo v4barAngle;

    public Servo clawLeft;
    public Servo clawRight;

    public Servo turret;


    // Slides
    public static int slidesDown = 0;
    public static int level1 = 300;
    public static int levelIncrement = 60;

    public static double slideDownPower=0.2;
    public static double slideUpPower=0.5;

    // V4Bar
    public static double v4barTransfer = 0.2;
    public static double v4barStow = 0.4;
    public static double v4barScore = 0.7;

    public static double angleTransfer = 0.2;
    public static double angleStow = 0.5;
    public static double angleScore = 0.7;

    // Claw
    public static double clawLeftClosed = 0.0;
    public static double clawRightClosed = 0.0;
    public static double clawLeftOpen = 0.3;
    public static double clawRightOpen = 0.3;

    // Turret
    public static double turretTransfer = 0.5;
    public static double turret60 = 0.2;






    public Outtake (HardwareMap hardwareMap) {
        // Slides
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        // V4Bar
        v4barLeft = hardwareMap.get(Servo.class, "v4barLeft");
        v4barRight = hardwareMap.get(Servo.class, "v4barRight");
        v4barAngle = hardwareMap.get(Servo.class, "v4barPivot");

        v4barRight.setDirection(Servo.Direction.REVERSE);

        // Claw
        clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        clawRight = hardwareMap.get(Servo.class, "clawRight");

        clawLeft.setDirection(Servo.Direction.REVERSE);

        // Turret
        turret = hardwareMap.get(Servo.class, "turret");

    }

    // Slides
    public void retractSlides() {
        setSlides(slidesDown);
    }

    public void slidesToLevel(int level) {
        setSlides(level1 +  (level - 1) * levelIncrement);
    }

    public void manualSlides(double power) {
        setSlides(getSlidePos() + (int) (power * 50));
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
        v4barAngleTransfer();
    }
    public void v4barStow() {
        setV4Bar(v4barStow);
        v4barAngleStow();
    }
    public void v4barScore() {
        setV4Bar(v4barScore);
        v4barAngleScore();
    }


    public void setV4Bar(double pos) {
        v4barRight.setPosition(pos);
        v4barLeft.setPosition(pos);
    }

    // V4Bar Pivot
    public void v4barAngleTransfer() {
        v4barAngle.setPosition(angleTransfer);
    }

    public void v4barAngleStow() {
        v4barAngle.setPosition(angleStow);
    }

    public void v4barAngleScore() {
        v4barAngle.setPosition(angleScore);
    }


    // Claw
    public void closeBothClaws() {
        closeLeft();
        closeRight();
    }
    public void openBothClaws() {
        openLeft();
        openRight();
    }

    public boolean isClawOpen() {
        return (clawLeft.getPosition() == clawLeftOpen) && (clawRight.getPosition() == clawRightOpen);
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

    public void turretTo(int position) {
        if (position == 0) {
            setTurret(turretTransfer);
        } else if (position > 0) {
            setTurret(turretTransfer + (position * turret60 + turret60/2));
        } else {
            setTurret(turretTransfer - (position * turret60 - turret60/2));
        }
    }

    public void setTurret(double pos) {
        turret.setPosition(pos);
    }
    public double getTurret() {
        return turret.getPosition();
    }



}

package org.firstinspires.ftc.teamcode.LebronBot.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public class Outtake {

    public DcMotorEx leftSlide;
    public DcMotorEx midSlide;
    public DcMotorEx rightSlide;

    public ServoImplEx v4barLeft;
    public ServoImplEx v4barRight;
    public ServoImplEx v4barAngle;

    public ServoImplEx clawLeft;
    public ServoImplEx clawRight;

    public ServoImplEx turret;


    // Slides
    public static int slidesDown = 2;
    public static int level1 = 300;
    public static int levelIncrement = 60;

    public static double slideDownPower=0.15;
    public static double slideUpPower=0.4;

    // V4Bar
    public static double v4barTransfer = 0.87; // 0.9
    public static double v4barStow = 0.73;
    public static double v4barOut = 0.64;
    public static double v4barScore = 0.1;

    public static double angleTransfer = 0.95;
    public static double angleStow = 0.8;
    public static double angleScore = 0.15;

    // Claw
    public static double clawLeftOpen = 0.25;
    public static double clawLeftClosed = 0.45;
    public static double clawLeftMoreClosed = 0.55;

    public static double clawRightOpen = 0.66;
    public static double clawRightClosed = 0.48;
    public static double clawRightMoreClosed = 0.38;

    // Turret
    public static double turretTransfer = 0.53;
    public static double turret60 = 0.15;






    public Outtake (HardwareMap hardwareMap) {
        // Slides
        leftSlide = hardwareMap.get(DcMotorEx.class, "liftLeft");
        midSlide = hardwareMap.get(DcMotorEx.class, "liftMid");
        rightSlide = hardwareMap.get(DcMotorEx.class, "liftRight");

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        midSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        midSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftSlide.setTargetPositionTolerance(15);
        midSlide.setTargetPositionTolerance(15);
        rightSlide.setTargetPositionTolerance(15);


        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        // V4Bar
        v4barLeft = hardwareMap.get(ServoImplEx.class, "leftV4B");
        v4barRight = hardwareMap.get(ServoImplEx.class, "rightV4B");
        v4barAngle = hardwareMap.get(ServoImplEx.class, "wrist");

        v4barRight.setDirection(Servo.Direction.REVERSE);

        // Claw
        clawLeft = hardwareMap.get(ServoImplEx.class, "clawLeft");
        clawRight = hardwareMap.get(ServoImplEx.class, "clawRight");

        // Turret
        turret = hardwareMap.get(ServoImplEx.class, "turret");

        turret.setPwmRange(new PwmControl.PwmRange(500, 2500));

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
        leftSlide.setTargetPosition(pos);
        midSlide.setTargetPosition(pos);
        rightSlide.setTargetPosition(pos);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        midSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(getSlidePos() > pos ? slideUpPower + 0.1 : slideDownPower);
        midSlide.setPower(getSlidePos() > pos ? slideUpPower : slideDownPower);
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

    public void v4barOut() {
        setV4Bar(v4barOut);
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
    public void moreClose() {
        closeLeftMore();
        closeLeftMore();
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
    public void closeLeftMore() {
        clawLeft.setPosition(clawLeftMoreClosed);
    }
    public void closeRight() {
        clawRight.setPosition(clawRightClosed);
    }
    public void closeRightMore() {
        clawRight.setPosition(clawRightMoreClosed);
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
            setTurret(turretTransfer + (position * turret60) + turret60/2);
        } else {
            setTurret(turretTransfer - (Math.abs(position) * turret60) - turret60/2);
        }
    }

    public void setTurret(double pos) {
        turret.setPosition(pos);
    }
    public double getTurret() {
        return turret.getPosition();
    }



}
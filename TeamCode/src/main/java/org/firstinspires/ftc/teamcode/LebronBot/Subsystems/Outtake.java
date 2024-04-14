package org.firstinspires.ftc.teamcode.LebronBot.Subsystems;

import com.acmerobotics.dashboard.config.Config;
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
    public static int slidesDown = -20;
    public static int level1 = 160;
    public static int levelIncrement = 150;

    public static double slideDownPower = 0.4;
    public static double slideUpPower = 0.9;

    // V4Bar
    public static double v4barTransfer = 0.115; // 0.9
    public static double v4barStow = 0.19;
    public static double v4barView = 0.37;
    public static double v4barUp = 0.45;
    public static double v4barScore = 0.7;
    public static double v4barAutoScore = 0.75;
    public static double v4barPurple=1;

    public static double angleTransfer = 0.825;
    public static double angleStow = 0.7;
    public static double angleScore = 0.04;
    public static double anglePurple = 0;

    // Claw
    public static double clawLeftOpen = 0;
    public static double clawLeftClosed = 0.55;
    public static double clawLeftMoreClosed = 0.75;

    public static double clawRightOpen = 1;
    public static double clawRightClosed = 0.65;
    public static double clawRightMoreClosed = 0.55;

    // Turret
    public static double turretTransfer = 0.49;
    public static double turret60 = 0.16;






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

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        midSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide.setTargetPositionTolerance(10);
        midSlide.setTargetPositionTolerance(10);
        rightSlide.setTargetPositionTolerance(10);


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
        if (level == 1) {
            setSlides(0);
        } else if (level == 2) {
            setSlides(120);
        } else if (level == 3) {
            setSlides(270);
        } else if (level == 4) {
            setSlides(390);
        } else if (level == 5) {
            setSlides(510);
        } else if (level == 6) {
            setSlides(630);
        } else if (level == 7) {
            setSlides(880);
        } else if (level == 8) {
            setSlides(1000);
        } else if (level == 9) {
            setSlides(1120);
        } else if (level == 10) {
            setSlides(1240);
        }
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

    public void hang(int pos) {
        leftSlide.setTargetPosition(pos);
        midSlide.setTargetPosition(pos);
        rightSlide.setTargetPosition(pos);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        midSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(getSlidePos() > pos ? slideUpPower + 0.1 : 1);
        midSlide.setPower(getSlidePos() > pos ? slideUpPower : 1);
        rightSlide.setPower(getSlidePos() > pos ? slideUpPower : 1);
    }

    public void setSlidesPower(double power) {
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        midSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftSlide.setPower(power);
        midSlide.setPower(power);
        rightSlide.setPower(power);
    }

    public void zeroSlides() {
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        midSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        midSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
    public void v4barView() {
        setV4Bar(v4barView);
        v4barAngleStow();
    }

    public void v4barUp() {
        setV4Bar(v4barUp);
        v4barAngleStow();
    }
    public void v4barScore() {
        setV4Bar(v4barScore);
        v4barAngleScore();
    }

    public void v4BarAuto() {
        setV4Bar(v4barAutoScore);
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

    public void setV4BarAngle (double pos) {
        v4barAngle.setPosition(pos);
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


    public void v4barPurple() {
        setV4Bar(v4barPurple);
    }

    public void v4BarAnglePurple() {
        v4barAngle.setPosition(anglePurple);
    }

    // Turret
    public void turretTransfer() {
        setTurret(turretTransfer);
    }

    public void turretTo(int position) {
        if (position == -3) {
            setTurret(0);
        } else if (position == -2) {
            setTurret(0.13);
        } else if (position == -1) {
            setTurret(0.345);
        } else if (position == 0) {
            setTurret(0.49);
        } else if (position == 1) {
            setTurret(0.6);
        } else if (position == 2) {
            setTurret(0.78);
        } else if (position == 3) {
            setTurret(1);
        }



//        else if (position > 0) {
//            setTurret(turretTransfer + (position * turret60) + turret60/2);
//        } else {
//            setTurret(turretTransfer - (Math.abs(position) * turret60) - turret60/2);
//        }
    }

    public void setTurret(double pos) {
        turret.setPosition(pos);
    }
    public double getTurret() {
        return turret.getPosition();
    }



}
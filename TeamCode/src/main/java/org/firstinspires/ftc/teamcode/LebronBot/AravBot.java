package org.firstinspires.ftc.teamcode.LebronBot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.LebronBot.Roadrunner.MecanumDrive;

@Config
public class AravBot {

    public MecanumDrive drive;

    public DcMotorEx slideLeft;
    public DcMotorEx slideRight;

    public ServoImplEx pivot;
    public ServoImplEx turret;

    public ServoImplEx clawLeft;
    public ServoImplEx clawRight;


    public ServoImplEx drone;

    public TouchSensor left;
    public TouchSensor right;
    public TouchSensor slideLimit;
    public TouchSensor intakeTouch;


    // Slides
    public static int slidesDown = 0;
    public static int level1 = 0;
    public static int levelIncrement = 80;

    public static double slideDownPower = 0.15;
    public static double slideUpPower = 0.2;


    // V4B
    public static double v4bPickup = 0.5;
    public static double anglePickup = 0.1;


    public static double v4bStow = 0.1;
    public static double angleStow = 0.5;

    public static double v4bScore = 0.1;
    public static double angleScore = 0.5;


    // Claw
    public static double clawOpen = 0.25;
    public static double clawClose = 0.8;

    // Drone
    public static double droneHold = 0.8;
    public static double droneRelease = 0.1;



    public AravBot(HardwareMap hardwareMap) {

        drive = new MecanumDrive(hardwareMap);

        slideLeft = hardwareMap.get(DcMotorEx.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");

        pivot = hardwareMap.get(ServoImplEx.class, "pivot");
        turret = hardwareMap.get(ServoImplEx.class, "turret");

        clawLeft = hardwareMap.get(ServoImplEx.class, "clawLeft");
        clawRight = hardwareMap.get(ServoImplEx.class, "clawRight");

        drone = hardwareMap.get(ServoImplEx.class, "drone");

//        left = hardwareMap.get(TouchSensor.class, "leftSensor");
//        right = hardwareMap.get(TouchSensor.class, "rightSensor");

        //Reverse
        slideLeft.setDirection(DcMotorEx.Direction.REVERSE);
        clawLeft.setDirection(ServoImplEx.Direction.REVERSE);

        //RTP
        slideLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
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
        return (!slideLeft.isBusy() && !slideRight.isBusy());
    }

    public int getSlidePos() {
        return (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2;
    }


    public void setSlides(int pos) {
        slideLeft.setTargetPosition(pos);
        slideRight.setTargetPosition(pos);
        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideLeft.setPower(getSlidePos() > pos ? slideUpPower : slideDownPower);
        slideRight.setPower(getSlidePos() > pos ? slideUpPower : slideDownPower);
    }


    // V4Bar
    public void armPickup() {
        setArm(v4bPickup);
        turret.setPosition(anglePickup);
    }
    public void armStow() {
        setArm(v4bStow);
        turret.setPosition(angleStow);
    }
    public void armScore() {
        setArm(v4bScore);
        turret.setPosition(angleScore);
    }
    public double getArmPos() {
        return pivot.getPosition();
    }
    public void setArm(double pos) {
        pivot.setPosition(pos);
    }




    // Claw
    public void openClaw() {
        openLeftClaw();
        openRightClaw();
    }
    public void closeClaw() {
        closeLeftClaw();
        closeRightClaw();
    }

    public void openLeftClaw() {
        clawLeft.setPosition(clawOpen);
    }
    public void openRightClaw() {
        clawRight.setPosition(clawOpen);
    }

    public void closeLeftClaw() {
        clawLeft.setPosition(clawClose);
    }
    public void closeRightClaw() {
        clawRight.setPosition(clawClose);
    }


    // Drone
    public void droneHold() {
        drone.setPosition(droneHold);
    }
    public void droneRelease() {
        drone.setPosition(droneRelease);
    }

}
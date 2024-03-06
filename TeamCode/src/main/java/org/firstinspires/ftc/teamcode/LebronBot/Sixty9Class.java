package org.firstinspires.ftc.teamcode.LebronBot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.LebronBot.Roadrunner.MecanumDrive;

@Config
public class Sixty9Class {

    public MecanumDrive drive;

    public DcMotorEx intakeMotor;
    public ServoImplEx intakePivot;

    public ServoImplEx v4barLeft;
    public ServoImplEx v4barRight;
    public ServoImplEx v4barAngle;


    public ServoImplEx clawFront;
    public ServoImplEx clawBack;
    public ServoImplEx turret;

    public DcMotorEx slideLeft;
    public DcMotorEx slideRight;


    public ServoImplEx drone;


    // Intake
    public static double intakePower = 0.8;
    public static double intakePickup = 0.1;
    public static double intakeTransfer = 0.5;


    // Slides
    public static int slidesDown = 0;
    public static int level1 = 0;
    public static int levelIncrement = 80;

    public static double slideDownPower = 0.4;
    public static double slideUpPower = 0.6;


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

    public static double turretTransfer = 0.5;
    public static double turret60 = 0.2;

    // Drone
    public static double droneHold = 0.8;
    public static double droneRelease = 0.1;



    public Sixty9Class(HardwareMap hardwareMap) {

        drive = new MecanumDrive(hardwareMap);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakePivot = hardwareMap.get(ServoImplEx.class, "intakePivot");

        v4barLeft = hardwareMap.get(ServoImplEx.class, "v4barLeft");
        v4barRight = hardwareMap.get(ServoImplEx.class, "v4barRight");
        v4barAngle = hardwareMap.get(ServoImplEx.class, "v4barAngle");


        clawFront = hardwareMap.get(ServoImplEx.class, "clawFront");
        clawBack = hardwareMap.get(ServoImplEx.class, "clawBack");
        turret = hardwareMap.get(ServoImplEx.class, "turret");

        slideLeft = hardwareMap.get(DcMotorEx.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");



//        drone = hardwareMap.get(ServoImplEx.class, "drone");


        //Reverse
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);

        v4barLeft.setDirection(ServoImplEx.Direction.REVERSE);

        clawBack.setDirection(ServoImplEx.Direction.REVERSE);

        slideLeft.setDirection(DcMotorEx.Direction.REVERSE);

        //RTP
        slideLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }



    // Intake
    public void intakePower(double power) {
        intakeMotor.setPower(power);
    }
    public void intakePickup() {
        intakePivot.setPosition(intakePickup);
    }
    public void intakeTransfer() {
        intakePivot.setPosition(intakeTransfer);
    }
    //0.09 armPivot transfer
    //0.8 armPivot score
    // V4Bar
    public void v4bTransfer() {
        setV4bar(v4bPickup);
        v4barAngle.setPosition(anglePickup);
    }
    public void v4bStow() {
        setV4bar(v4bStow);
        v4barAngle.setPosition(angleStow);
    }
    public void v4bScore() {
        setV4bar(v4bScore);
        v4barAngle.setPosition(angleScore);
    }
    public void setV4bar(double pos) {
        v4barLeft.setPosition(pos);
        v4barRight.setPosition(pos);
    }

    // Claw
    public void openClaws() {
        openFrontClaw();
        openBackClaw();
    }
    public void closeClaws() {
        closeFrontClaw();
        closeBackClaw();
    }

    public void openFrontClaw() {
        clawFront.setPosition(clawOpen);
    }
    public void openBackClaw() {
        clawBack.setPosition(clawOpen);
    }

    public void closeFrontClaw() {
        clawFront.setPosition(clawClose);
    }
    public void closeBackClaw() {
        clawBack.setPosition(clawClose);
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


    // Drone
    public void droneHold() {
        drone.setPosition(droneHold);
    }
    public void droneRelease() {
        drone.setPosition(droneRelease);
    }

}
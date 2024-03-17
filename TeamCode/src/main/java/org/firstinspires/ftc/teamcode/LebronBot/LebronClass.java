package org.firstinspires.ftc.teamcode.LebronBot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.LebronBot.Roadrunner.MecanumDrive;

@Config
public class LebronClass {

    public MecanumDrive drive;

    public DcMotorEx slideLeft;
    public DcMotorEx slideRight;
    public DcMotorEx slideMiddle;
    public DcMotorEx intake;

    public ServoImplEx pivot;
    public ServoImplEx wrist;
    public ServoImplEx turret;

    public ServoImplEx clawLeft;
    public ServoImplEx clawRight;
    public ServoImplEx leftIntakePivot;
    public ServoImplEx rightIntakePivot;

    public ServoImplEx drone;

    public TouchSensor left;
    public TouchSensor right;


    // Slides
    public static int slidesDown = 0;
    public static int level1 = 0;
    public static int levelIncrement = 80;

    public static double slideDownPower = 0.15;
    public static double slideUpPower = 0.2;

    // arm
    public static double armPickup = 0.5;
    public static double anglePickup = 0.1;
    public static double wristPickup = 0.6;


    public static double armStow = 0.1;
    public static double angleStow = 0.5;
    public static double wristStow = 0.6;

    public static double armScore = 0.1;
    public static double angleScore = 0.5;
    public static double wristScore = 0.6;


    // Claw
    public static double clawOpen = 0.25;
    public static double clawClose = 0.8;

    // Drone
    public static double droneHold = 0.8;
    public static double droneRelease = 0.1;

    public static double intakeDown = 0.4;
    public static double intakeUp = 0.6;
    public static double intakeStack = 0.5;

    public LebronClass(HardwareMap hardwareMap) {

        drive = new MecanumDrive(hardwareMap);

        slideLeft = hardwareMap.get(DcMotorEx.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");
        slideMiddle = hardwareMap.get(DcMotorEx.class, "slideMid");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        leftIntakePivot = hardwareMap.get(ServoImplEx.class, "leftPivot");
        rightIntakePivot = hardwareMap.get(ServoImplEx.class, "rightPivot");

        pivot = hardwareMap.get(ServoImplEx.class, "arm");
        turret = hardwareMap.get(ServoImplEx.class, "turret");
        wrist = hardwareMap.get(ServoImplEx.class, "wrist");

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
        slideMiddle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slideMiddle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntakePivot.setDirection(Servo.Direction.REVERSE);
    }

    // Slides
    public void retractSlides() {
        setSlides(slidesDown);
    }

    public void slidesToLevel(int level) {
        setSlides(level1 +  (level - 1) * levelIncrement);
    }

    public void setIntake(double power) {
        intake.setPower(power);
    }

    public void setIntakePos(double pos) {
        leftIntakePivot.setPosition(pos);
        rightIntakePivot.setPosition(pos);
    }

    public void intakeDown() {
        setIntake(-1);
        setIntakePos(intakeDown);
    }

    public void intakeUp() {
        setIntake(0);
        setIntake(intakeUp);
    }

    public void manualSlides(double power) {
        setSlides(getSlidePos() + (int) (power * 50));
    }

    public boolean isDone() {
        return (!slideLeft.isBusy() && !slideRight.isBusy() && !slideMiddle.isBusy());
    }

    public int getSlidePos() {
        return (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition() + slideMiddle.getCurrentPosition()) / 3;
    }

    public void setSlides(int pos) {
        slideLeft.setTargetPosition(pos);
        slideRight.setTargetPosition(pos);
        slideMiddle.setTargetPosition(pos);
        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMiddle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideLeft.setPower(getSlidePos() > pos ? slideUpPower : slideDownPower);
        slideRight.setPower(getSlidePos() > pos ? slideUpPower : slideDownPower);
        slideMiddle.setPower(getSlidePos() > pos ? slideUpPower : slideDownPower);
    }


    // armar
    public void armPickup() {
        setArm(armPickup);
        turret.setPosition(anglePickup);
        wrist.setPosition(wristPickup);
    }
    public void armStow() {
        setArm(armStow);
        turret.setPosition(angleStow);
        wrist.setPosition(wristStow);
    }
    public void armScore() {
        setArm(armScore);
        turret.setPosition(angleScore);
        wrist.setPosition(wristScore);
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
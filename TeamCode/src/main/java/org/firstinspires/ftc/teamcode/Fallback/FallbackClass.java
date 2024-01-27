package org.firstinspires.ftc.teamcode.Fallback;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;

@Config
public class FallbackClass {

    public MecanumDrive drive;

    public DcMotorEx slideLeft;
    public DcMotorEx slideRight;

    public ServoImplEx v4bLeft;
    public ServoImplEx v4bRight;
    public ServoImplEx v4bAngle;

    public ServoImplEx clawLeft;
    public ServoImplEx clawRight;


    public ServoImplEx drone;


    // Slides
    public static int slidesDown = 0;
    public static int level1 = 300;
    public static int levelIncrement = 60;

    public static double slideDownPower = 0.2;
    public static double slideUpPower = 0.5;


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
    public static double droneHold = 0.5;
    public static double droneRelease = 0.1;





    public FallbackClass(HardwareMap hardwareMap) {

        drive = new MecanumDrive(hardwareMap);

        //slideLeft = hardwareMap.get(DcMotorEx.class, "slideLeft");
        //slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");

        //v4bLeft = hardwareMap.get(ServoImplEx.class, "v4bLeft");
        //v4bRight = hardwareMap.get(ServoImplEx.class, "v4bRight");
        //v4bAngle = hardwareMap.get(ServoImplEx.class, "v4bAngle");

        //clawLeft = hardwareMap.get(ServoImplEx.class, "clawLeft");
        //clawRight = hardwareMap.get(ServoImplEx.class, "clawRight");

        // Reverse
        //slideLeft.setDirection(DcMotorEx.Direction.REVERSE);
        //v4bLeft.setDirection(ServoImplEx.Direction.REVERSE);
        //clawLeft.setDirection(ServoImplEx.Direction.REVERSE);

        // RTP
        //slideLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //slideRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //slideLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //slideRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

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
    public void v4barPickup() {
        setV4Bar(v4bPickup);
        v4bAngle.setPosition(anglePickup);
    }
    public void v4barStow() {
        setV4Bar(v4bStow);
        v4bAngle.setPosition(angleStow);
    }
    public void v4barScore() {
        setV4Bar(v4bScore);
        v4bAngle.setPosition(angleScore);
    }
    public double getV4BarPos() {
        return (v4bLeft.getPosition()+v4bRight.getPosition())/2;
    }
    public void setV4Bar(double pos) {
        v4bLeft.setPosition(pos);
        v4bRight.setPosition(pos);
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
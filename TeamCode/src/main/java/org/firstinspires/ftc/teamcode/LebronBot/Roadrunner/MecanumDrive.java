package org.firstinspires.ftc.teamcode.LebronBot.Roadrunner;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.LebronBot.PurePursuit.ErrorCalculator;
import org.firstinspires.ftc.teamcode.LebronBot.PurePursuit.MotorCache;
import org.firstinspires.ftc.teamcode.LebronBot.PurePursuit.Testing.BackTracking_TUNING;
import org.firstinspires.ftc.teamcode.LebronBot.Roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.LebronBot.Roadrunner.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.LebronBot.Roadrunner.util.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.LebronBot.Roadrunner.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Vector;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class MecanumDrive extends com.acmerobotics.roadrunner.drive.MecanumDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(7.75, 0, 0.2);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(7.75, 0, 0.1);

    public static double LATERAL_MULTIPLIER = 1.153;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    protected TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(DriveConstants.MAX_ACCEL);

    private TrajectoryFollower follower;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    private IMU imu;
    private VoltageSensor batteryVoltageSensor;

    private List<Integer> lastEncPositions = new ArrayList<>();
    private List<Integer> lastEncVels = new ArrayList<>();

    private Vector2d prevPos = new Vector2d(0, 0);
    private Vector2d prevVel = new Vector2d(0, 0);
    private Vector2d vel = new Vector2d(0, 0);
    private Vector2d accel = new Vector2d(0, 0);

    Vector2d vP=new Vector2d(0,0); //covariance guess
    Vector2d vR=new Vector2d(1.5,1.5); //sensor covariance
    Vector2d vQ = new Vector2d(16, 16); //model covariance
    private long lastRead = 0;
    private ArrayList<Vector2d> accelAvg = new ArrayList<>();
    Pose2d poseNew;
    double[] lastErrors = new double[3];
    static double [] TotalChange = new double[2];
    double [] prevPose = new double[2];
    double[] newPose= new double []{0.0,0.0};
    static double [] Difference = new double[2];

    //This is the exact percentage of the effectiveness of this Localizer
    double [] percentage_Correction = new double[]{0.0,0.0};
    public static ElapsedTime timer = new ElapsedTime();
    private double heading=0;
    private double lastHeading=0;
    private double headingDelta=0;
    ArrayList<Pose2d> listOfPoses = new ArrayList<>();
    ErrorCalculator errorCalculator;
    Pose2d prevPose2d = new Pose2d();
    MotorCache LF;
    MotorCache LR;
    MotorCache RR;
    MotorCache RF;

    public MecanumDrive(HardwareMap hardwareMap) {
        super(DriveConstants.kV, DriveConstants.kA, DriveConstants.kStatic, DriveConstants.TRACK_WIDTH, DriveConstants.TRACK_WIDTH, LATERAL_MULTIPLIER);

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        errorCalculator = new ErrorCalculator();
        // TODO: adjust the names of the following hardware devices to match your configuration
//        imu = hardwareMap.get(IMU.class, "imu");
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
//        imu.initialize(parameters);

        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        leftRear = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightRear = hardwareMap.get(DcMotorEx.class, "backRight");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");
        LF = new MotorCache(leftFront);
        LR = new MotorCache(leftRear);
        RR = new MotorCache(rightRear);
        RF = new MotorCache(rightFront);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (DriveConstants.RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (DriveConstants.RUN_USING_ENCODER && DriveConstants.MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DriveConstants.MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        // TODO: if desired, use setLocalizer() to change the localization method
         setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels));
        //setLocalizer(new TwoWheelTrackingLocalizer(hardwareMap, this));

        trajectorySequenceRunner = new TrajectorySequenceRunner(
                follower, HEADING_PID, batteryVoltageSensor,
                lastEncPositions, lastEncVels, lastTrackingEncPositions, lastTrackingEncVels
        );
    }

    public Vector2d getVelocity() {
        return vel;
    }

    public Vector2d getAccel() {
        return accel;
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        updateMech();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void updateMech() {
        double timeGap = (double)(System.currentTimeMillis()-lastRead)/1000;
        Vector2d poseVec = getPoseEstimate().vec();
        Vector2d estimate = vel.plus(vel.minus(prevVel));
        prevVel=vel;
        vel= poseVec.minus(prevPos);
        vel = new Vector2d(vel.getX()/timeGap, vel.getY()/timeGap);
        vP=vP.plus(vQ);
        Vector2d K = new Vector2d(vP.getX()/(vP.getX()+vR.getX()),vP.getY()/(vP.getY()+vR.getY()));
        vel = new Vector2d(vel.getX() + K.getX()*(estimate.getX()-vel.getX()), vel.getY() + K.getY()*(estimate.getY()-vel.getY()));
        vP = new Vector2d((1-K.getX())*vP.getX(), (1-K.getY())*vP.getY());
        accel= vel.minus(prevVel);
        accel = new Vector2d(accel.getX(), accel.getY());
        prevPos = poseVec;
        lastRead = System.currentTimeMillis();
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        lastEncPositions.clear();

        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            int position = motor.getCurrentPosition();
            lastEncPositions.add(position);
            wheelPositions.add(DriveConstants.encoderTicksToInches(position));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        lastEncVels.clear();

        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            int vel = (int) motor.getVelocity();
            lastEncVels.add(vel);
            wheelVelocities.add(DriveConstants.encoderTicksToInches(vel));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        LF.setPower(v);
        LR.setPower(v1);
        RR.setPower(v2);
        RF.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//        return 0;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return (double) imu.getRobotAngularVelocity(AngleUnit.RADIANS).xRotationRate;
//        return (double)0;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }
}
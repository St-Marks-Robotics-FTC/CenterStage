package org.firstinspires.ftc.teamcode.Testing;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.LM2.LM2class;
import org.firstinspires.ftc.teamcode.LM2.Roadrunner.DriveConstants;
import org.firstinspires.ftc.teamcode.LM2.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.LM2.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Vision.Prop.BlueFarPropThreshold;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
@Autonomous
public class AprilCorrection extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static int DESIRED_TAG_ID = 5;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal aprilPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    double strafe = 0;











    FtcDashboard dashboard;

    public static String loc = "left";
    MecanumDrive drive;
    LM2class robot;
    ElapsedTime aprilTimer = new ElapsedTime();

    private VisionPortal portal;
    private BlueFarPropThreshold blueFarPropThreshold;


    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  forwardDist           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)


        blueFarPropThreshold = new BlueFarPropThreshold();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(blueFarPropThreshold)
                .build();



        drive = new MecanumDrive(hardwareMap);
//        drive.setPoseEstimate(new Pose2d(12, -60, Math.toRadians(90)));
        robot = new LM2class(hardwareMap);

        Pose2d startPose = new Pose2d(0,0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

//        TrajectorySequence traj11 = drive.trajectorySequenceBuilder(startPose) // left
//                .splineToSplineHeading(new Pose2d(-31, 34, Math.toRadians(-30)), Math.toRadians(0))
//                .build();
//        TrajectorySequence traj12 = drive.trajectorySequenceBuilder(startPose) // middle
//                .splineToSplineHeading(new Pose2d(-40, 25, Math.toRadians(-10)), Math.toRadians(-10))
//                .build();
//        TrajectorySequence traj13 = drive.trajectorySequenceBuilder(startPose) // right
//                .splineTo(new Vector2d(-36, 32), Math.toRadians(-150))
//                .build();
//        TrajectorySequence traj21 = drive.trajectorySequenceBuilder(traj11.end())
//                //.splineTo(new Vector2d(-41, -32), Math.toRadians(150))
//                .setTangent(Math.toRadians(180))
//                .splineToSplineHeading(new Pose2d(-38, 34, Math.toRadians(0)), Math.toRadians(-180))
//                .setTangent(Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(-31, 11), Math.toRadians(0))
//                .splineToSplineHeading(new Pose2d(22, 11, Math.toRadians(0)), Math.toRadians(0))
//                .splineToSplineHeading(new Pose2d(57, 37, Math.toRadians(0)), Math.toRadians(0))
//                .build();
//        TrajectorySequence traj22 = drive.trajectorySequenceBuilder(traj12.end())
//                .splineToConstantHeading(new Vector2d(-48, 22), Math.toRadians(-180))
//                .setTangent(Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(-31, 9), Math.toRadians(0))
//                .splineToSplineHeading(new Pose2d(22, 9, Math.toRadians(0)), Math.toRadians(0))
//                .splineToSplineHeading(new Pose2d(57, 28.5, Math.toRadians(0)), Math.toRadians(0)) // 59
//                .build();
//        TrajectorySequence traj23 = drive.trajectorySequenceBuilder(traj13.end())
//                .lineToLinearHeading(new Pose2d(-34, 32, Math.toRadians(-120)))
//                .lineToLinearHeading(new Pose2d(-34, 11, Math.toRadians(-90)))
//                .setTangent(0)
//                .splineTo(new Vector2d(22, 11), Math.toRadians(0))
//                .splineToSplineHeading(new Pose2d(57, 25, Math.toRadians(0)), Math.toRadians(0))
//                .build();
//
//        TrajectorySequence park1 = drive.trajectorySequenceBuilder(traj21.end())
//                .back(8)
//                .build();
//        TrajectorySequence park2 = drive.trajectorySequenceBuilder(traj22.end())
//                .back(8)
//                .build();
//        TrajectorySequence park3 = drive.trajectorySequenceBuilder(traj23.end())
//                .back(8)
//                .build();

        //robot.closeClaw();
        robot.closeClaw();
        while (opModeInInit()) {
            loc = blueFarPropThreshold.getPropPosition();
            telemetry.addData("Prop Position", blueFarPropThreshold.getPropPosition());
            telemetry.addData("Avg Left Value", blueFarPropThreshold.getAvergageLeft());
            telemetry.addData("Avg Right Value", blueFarPropThreshold.getAvergageRight());
            telemetry.update();
        }
        portal.close();

        waitForStart();
//        sleep(1000);
//        switch (loc) {
//            case "none":
//                drive.followTrajectorySequence(traj11);
//                break;
//            case "left":
//                drive.followTrajectorySequence(traj12);
//                break;
//            case "right":
//                drive.followTrajectorySequence(traj13);
//                break;
//        }
//
//        robot.openRight();
//        sleep(3000);
//        robot.setArm(420); // 390
//
//        //outtake
//        switch (loc) {
//            case "none":
//                drive.followTrajectorySequence(traj21);
//                break;
//            case "left":
//                drive.followTrajectorySequence(traj22);
//                break;
//            case "right":
//                drive.followTrajectorySequence(traj23);
//                break;
//        }

        robot.setArm(0); // 390
//        sleep(2000);

        // Approach the target with apriltag
        switch (loc) {
            case "none":
                DESIRED_TAG_ID = 4;
                break;
            case "left":
                DESIRED_TAG_ID = 5;
                break;
            case "right":
                DESIRED_TAG_ID = 6;
                break;
        }

        initAprilTag();
        aprilTimer.reset();
        while (aprilTimer.milliseconds() < 2000 && opModeIsActive()) {

            targetFound = false;
            desiredTag  = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if (detection.id == DESIRED_TAG_ID) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    }
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {

                forwardDist = desiredTag.ftcPose.range;
                strafe = desiredTag.ftcPose.x - 1; // negative when left

                telemetry.addData("Strafe value" , strafe);
                telemetry.addData("Strafe Correction" , 1 * strafe);

                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
                telemetry.update();
            } else {
                telemetry.addLine("Nothing found");
                telemetry.addData("Looking for", desiredTag);
                telemetry.update();
            }

        }



        if (strafe != 0) {
            TrajectorySequence strafeRight = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .strafeRight(1 * strafe)
                    .build();
            drive.followTrajectorySequence(strafeRight);
        }






        robot.setArm(420); // 390
        sleep(1000);

        TrajectorySequence forward = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(forwardDist, MecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        MecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        drive.followTrajectorySequence(forward);





        robot.openClaw();
        sleep(500);
//        switch (loc) {
//            case "none":
//                drive.followTrajectorySequence(park1);
//                break;
//            case "left":
//                drive.followTrajectorySequence(park2);
//                break;
//            case "right":
//                drive.followTrajectorySequence(park3);
//                break;
//        }

        TrajectorySequence backwards = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .back(10)
                .build();
        drive.followTrajectorySequence(backwards);
        robot.setArm(0); // 700

    }






    // Functions


    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.

        aprilPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);


    }   // end method initAprilTag()

}





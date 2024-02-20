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
import org.firstinspires.ftc.teamcode.Vision.Misc.YellowPreload;
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

    private VisionPortal portal;
    private YellowPreload yellowDetector;
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    double strafe = 0;













    FtcDashboard dashboard;

    public static String loc = "middle";
    MecanumDrive drive;
    LM2class robot;
    ElapsedTime aprilTimer = new ElapsedTime();




    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  forwardDist           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)


        yellowDetector = new YellowPreload();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(yellowDetector)
                .addProcessor(aprilTag)
                .build();



        drive = new MecanumDrive(hardwareMap);
        robot = new LM2class(hardwareMap);

        Pose2d startPose = new Pose2d(0,0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);


        robot.closeClaw();
        while (opModeInInit()) {
            telemetry.addData("Yellow X", yellowDetector.getCentroidX());
            telemetry.addData("Yellow Y", yellowDetector.getCentroidY());



            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }   // end for() loop

            // Add "key" information to telemetry
            telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
            telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
            telemetry.addLine("RBE = Range, Bearing & Elevation");

            telemetry.update();
        }
        portal.close();

        waitForStart();
//
//        // Approach the target with apriltag
//        switch (loc) {
//            case "none":
//                DESIRED_TAG_ID = 4;
//                break;
//            case "left":
//                DESIRED_TAG_ID = 5;
//                break;
//            case "right":
//                DESIRED_TAG_ID = 6;
//                break;
//        }
//
//        aprilTimer.reset();
//        while (aprilTimer.milliseconds() < 2000 && opModeIsActive()) {
//
//            targetFound = false;
//            desiredTag  = null;
//
//            // Step through the list of detected tags and look for a matching tag
//            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//            for (AprilTagDetection detection : currentDetections) {
//                // Look to see if we have size info on this tag.
//                if (detection.metadata != null) {
//                    //  Check to see if we want to track towards this tag.
//                    if (detection.id == DESIRED_TAG_ID) {
//                        // Yes, we want to use this tag.
//                        targetFound = true;
//                        desiredTag = detection;
//                        break;  // don't look any further.
//                    }
//                }
//            }
//
//            // Tell the driver what we see, and what to do.
//            if (targetFound) {
//
//                forwardDist = desiredTag.ftcPose.range;
//                strafe = desiredTag.ftcPose.x - 1; // negative when left
//
//                telemetry.addData("Strafe value" , strafe);
//                telemetry.addData("Strafe Correction" , 1 * strafe);
//
//                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
//                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
//                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
//                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
//                telemetry.update();
//            } else {
//                telemetry.addLine("Nothing found");
//                telemetry.addData("Looking for", desiredTag);
//                telemetry.update();
//            }
//
//        }
//
//
//
//        if (strafe != 0) {
//            TrajectorySequence strafeRight = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                    .strafeRight(1 * strafe)
//                    .build();
//            drive.followTrajectorySequence(strafeRight);
//        }
//
//
//
//
//
//
//        robot.setArm(420); // 390
//        sleep(1000);
//
//        TrajectorySequence forward = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                .forward(forwardDist, MecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        MecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();
//        drive.followTrajectorySequence(forward);
//
//
//
//
//
//        robot.openClaw();
//        sleep(500);
//        TrajectorySequence backwards = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                .back(10)
//                .build();
//        drive.followTrajectorySequence(backwards);
//        robot.setArm(0); // 700

    }

}





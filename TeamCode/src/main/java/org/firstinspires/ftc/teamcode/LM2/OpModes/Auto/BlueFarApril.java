package org.firstinspires.ftc.teamcode.LM2.OpModes.Auto;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.LM2.LM2class;
import org.firstinspires.ftc.teamcode.LM2.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.LM2.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Vision.Prop.BlueFarPropThreshold;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
@Autonomous
public class BlueFarApril extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static int DESIRED_TAG_ID = 2;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
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
        double  drivePower           = 0;        // Desired forward power/speed (-1 to +1)
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

        Pose2d startPose = new Pose2d(-38, 60, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence traj11 = drive.trajectorySequenceBuilder(startPose) // left
                .splineToSplineHeading(new Pose2d(-31, 34, Math.toRadians(-30)), Math.toRadians(0))
                .build();
        TrajectorySequence traj12 = drive.trajectorySequenceBuilder(startPose) // middle
                .splineToSplineHeading(new Pose2d(-40, 25, Math.toRadians(-10)), Math.toRadians(-10))
                .build();
        TrajectorySequence traj13 = drive.trajectorySequenceBuilder(startPose) // right
                .splineTo(new Vector2d(-36, 32), Math.toRadians(-150))
                .build();
        TrajectorySequence traj21 = drive.trajectorySequenceBuilder(traj11.end())
                //.splineTo(new Vector2d(-41, -32), Math.toRadians(150))
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-38, 34, Math.toRadians(0)), Math.toRadians(-180))
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-31, 11), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(22, 11, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(57, 37, Math.toRadians(0)), Math.toRadians(0))
                .build();
        TrajectorySequence traj22 = drive.trajectorySequenceBuilder(traj12.end())
                .splineToConstantHeading(new Vector2d(-48, 22), Math.toRadians(-180))
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-31, 9), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(22, 9, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(57, 28.5, Math.toRadians(0)), Math.toRadians(0)) // 59
                .build();
        TrajectorySequence traj23 = drive.trajectorySequenceBuilder(traj13.end())
                .lineToLinearHeading(new Pose2d(-34, 32, Math.toRadians(-120)))
                .lineToLinearHeading(new Pose2d(-34, 11, Math.toRadians(-90)))
                .setTangent(0)
                .splineTo(new Vector2d(22, 11), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(57, 25, Math.toRadians(0)), Math.toRadians(0))
                .build();

        TrajectorySequence park1 = drive.trajectorySequenceBuilder(traj21.end())
                .back(8)
                .build();
        TrajectorySequence park2 = drive.trajectorySequenceBuilder(traj22.end())
                .back(8)
                .build();
        TrajectorySequence park3 = drive.trajectorySequenceBuilder(traj23.end())
                .back(8)
                .build();

        //robot.closeClaw();
        robot.closeClaw();
        while (opModeInInit()) {
            loc = blueFarPropThreshold.getPropPosition();
            telemetry.addData("Prop Position", blueFarPropThreshold.getPropPosition());
            telemetry.addData("Avg Left Value", blueFarPropThreshold.getAvergageLeft());
            telemetry.addData("Avg Right Value", blueFarPropThreshold.getAvergageRight());
            telemetry.update();
        }

        waitForStart();
        sleep(10000);
        switch (loc) {
            case "none":
                drive.followTrajectorySequence(traj11);
                break;
            case "left":
                drive.followTrajectorySequence(traj12);
                break;
            case "right":
                drive.followTrajectorySequence(traj13);
                break;
        }

        robot.openRight();
        sleep(3000);
        robot.setArm(420); // 390

        //outtake
        switch (loc) {
            case "none":
                drive.followTrajectorySequence(traj21);
                break;
            case "left":
                drive.followTrajectorySequence(traj22);
                break;
            case "right":
                drive.followTrajectorySequence(traj23);
                break;
        }

        robot.setArm(0); // 390
        sleep(2000);

        // Approach the target with apriltag
        switch (loc) {
            case "none":
                DESIRED_TAG_ID = 1;
                break;
            case "left":
                DESIRED_TAG_ID = 2;
                break;
            case "right":
                DESIRED_TAG_ID = 3;
                break;
        }

        initAprilTag();
        aprilTimer.reset();
        while (aprilTimer.milliseconds() < 3000 && opModeIsActive()) {

            targetFound = false;
            desiredTag  = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    }
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {

                strafe = desiredTag.ftcPose.x + 1; // negative when left

                telemetry.addData("Strafe value" , strafe);
                telemetry.addData("Strafe Correction" , 1.5 * strafe);

                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
            }

            telemetry.update();
        }

        if (Math.signum(strafe) == -1) {
            TrajectorySequence strafeLeft = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .strafeLeft(1.5 * strafe)
                    .build();
            drive.followTrajectorySequence(strafeLeft);
        } else if (Math.signum(strafe) == 1){
            TrajectorySequence strafeRight = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .strafeRight(1.5 * strafe)
                    .build();
            drive.followTrajectorySequence(strafeRight);
        }




        robot.setArm(420); // 390
        sleep(1000);

        TrajectorySequence forward = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(5)
                .build();
        drive.followTrajectorySequence(forward);





        robot.openLeft();
        sleep(500);
        switch (loc) {
            case "none":
                drive.followTrajectorySequence(park1);
                break;
            case "left":
                drive.followTrajectorySequence(park2);
                break;
            case "right":
                drive.followTrajectorySequence(park3);
                break;
        }
        robot.setArm(0); // 700

    }






    // Functions


    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()

}





package org.firstinspires.ftc.teamcode.Vision.AprilTag;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class AprilTagRelocalize {

    AprilTagLibrary.Builder aprilTagLibraryBuilder;
    AprilTagLibrary aprilTagLibrary;
    AprilTagProcessor.Builder aprilTagProcessorBuilder;
    AprilTagProcessor aprilTagProcessor;
    AprilTagVisionProcessor visionProcessor;

    VisionPortal.Builder visionPortalBuilder;
    VisionPortal visionPortal;

    //how far away is the camera from the center of the robot
    private Pose2d cameraOffset = new Pose2d(-7, 0, Math.toRadians(180));
    //5 inches away from the apriltag

    public AprilTagRelocalize(HardwareMap hardwareMap) {
        aprilTagLibraryBuilder = new AprilTagLibrary.Builder();
        aprilTagLibraryBuilder.addTags(AprilTagGameDatabase.getCenterStageTagLibrary());
        //aprilTagLibraryBuilder.addTag(7, "TestTag" , 7.7, DistanceUnit.INCH);

        aprilTagLibrary = aprilTagLibraryBuilder.build();
        aprilTagProcessorBuilder = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES);
        aprilTagProcessorBuilder.setTagLibrary(aprilTagLibrary);
        aprilTagProcessor = aprilTagProcessorBuilder.build();
        visionProcessor = new AprilTagVisionProcessor();

        visionPortalBuilder = new VisionPortal.Builder();
        visionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        visionPortalBuilder.addProcessor(aprilTagProcessor);
        visionPortalBuilder.setCameraResolution(new Size(640, 480));
        visionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        visionPortalBuilder.enableLiveView(true);
        visionPortalBuilder.setAutoStopLiveView(true);
        visionPortalBuilder.addProcessor(visionProcessor); //to rotate the camera 180

        visionPortal = visionPortalBuilder.build();

    }

    public Pose2d getTagPos(int tag) {
        //AprilTagGameDatabase.getCenterStageTagLibrary();
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        AprilTagDetection target = null;
        for (AprilTagDetection detection : detections) {
            if (detection.metadata!=null && detection.id == tag) {
                target = detection;
            }
        }
        if (target == null) {
            return new Pose2d(999, 0, 0);
        } else {
            //double angle = Math.toRadians(90)-target.ftcPose.bearing;
            return new Pose2d(
                    target.ftcPose.range*Math.cos(Math.toRadians(target.ftcPose.bearing-target.ftcPose.yaw)),
                    target.ftcPose.range*Math.sin(Math.toRadians(target.ftcPose.bearing-target.ftcPose.yaw)),
                    angleWrap(Math.toRadians(90-target.ftcPose.yaw)));//.plus(cameraOffset);
        }
    }

    public double angleWrap(double radians) {

        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        // keep in mind that the result is in radians
        return radians;
    }

    public void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return;
        }
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
    }
}

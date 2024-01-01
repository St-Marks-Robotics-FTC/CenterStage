package org.firstinspires.ftc.teamcode.Vision.AprilTag;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagRelocalize {

    AprilTagLibrary.Builder aprilTagLibraryBuilder;
    AprilTagLibrary aprilTagLibrary;
    AprilTagProcessor.Builder aprilTagProcessorBuilder;
    AprilTagProcessor aprilTagProcessor;

    VisionPortal.Builder visionPortalBuilder;
    VisionPortal  visionPortal;

    //how far away is the camera from the center of the robot
    private int cameraOffsetY = 7;
    //5 inches away from the apriltag
    private double D = 6;

    public AprilTagRelocalize(HardwareMap hardwareMap) {
        aprilTagLibraryBuilder = new AprilTagLibrary.Builder();
        aprilTagLibraryBuilder.addTags(AprilTagGameDatabase.getCenterStageTagLibrary());
        //aprilTagLibraryBuilder.addTag(7, "TestTag" , 7.7, DistanceUnit.INCH);

        aprilTagLibrary = aprilTagLibraryBuilder.build();
        aprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        aprilTagProcessorBuilder.setTagLibrary(aprilTagLibrary);
        aprilTagProcessor = aprilTagProcessorBuilder.build();

        visionPortalBuilder = new VisionPortal.Builder();
        visionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        visionPortalBuilder.addProcessor(aprilTagProcessor);
        visionPortalBuilder.setCameraResolution(new Size(640, 480));
        visionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        visionPortalBuilder.enableLiveView(true);
        visionPortalBuilder.setAutoStopLiveView(true);

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
            return new Pose2d(target.ftcPose.x-D*Math.sin(target.ftcPose.yaw),
                    target.ftcPose.y+cameraOffsetY-D*Math.cos(target.ftcPose.yaw),
                    Math.toRadians(90)+target.ftcPose.yaw);
        }
    }
}

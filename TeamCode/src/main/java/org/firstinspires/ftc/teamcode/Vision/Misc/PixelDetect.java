package org.firstinspires.ftc.teamcode.Vision.Misc;

import android.util.Log;
import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Vision.AprilTag.AprilTagVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class PixelDetect {

    FullPixelDetection fullPixelDetection;

    VisionPortal.Builder visionPortalBuilder;
    public VisionPortal visionPortal;
    Vector2d cameraOffset = new Vector2d(-7.1,0.6); //measure from the lense

    public PixelDetect(HardwareMap hardwareMap) {
        fullPixelDetection = new FullPixelDetection();
        visionPortalBuilder = new VisionPortal.Builder();
        visionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        visionPortalBuilder.addProcessor(fullPixelDetection);
        visionPortalBuilder.setCameraResolution(new Size(1280, 720));
        visionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        visionPortalBuilder.enableLiveView(true);
        visionPortalBuilder.setAutoStopLiveView(true);

        visionPortal = visionPortalBuilder.build();
    }

    public Vector2d getCoord() {
        Vector2d cameraCoord = fullPixelDetection.getPixelcoord();
//        Log.d("raw camera: ", cameraCoord.toString());
        Vector2d robotCoord = cameraCoord.rotated(Math.PI).plus(cameraOffset);
        return robotCoord;
    }
}

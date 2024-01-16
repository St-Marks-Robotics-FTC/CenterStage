package org.firstinspires.ftc.teamcode.jankbot.prototype;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.Prop.BlueFarPropThreshold;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class CameraTest extends LinearOpMode {
    private VisionPortal portal;
    private BlueFarPropThreshold blueFarPropThreshold;
    @Override
    public void runOpMode() throws InterruptedException {
        blueFarPropThreshold = new BlueFarPropThreshold();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(blueFarPropThreshold)
                .build();
        while (opModeInInit()) {
            telemetry.addData("Prop Position", blueFarPropThreshold.getPropPosition());
            telemetry.addData("Avg Left Value", blueFarPropThreshold.getAvergageLeft());
            telemetry.addData("Avg Right Value", blueFarPropThreshold.getAvergageRight());
            telemetry.update();
        }
    }
}

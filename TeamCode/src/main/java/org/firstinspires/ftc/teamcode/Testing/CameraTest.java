package org.firstinspires.ftc.teamcode.Testing;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.Prop.RedPropThreshold;
import org.firstinspires.ftc.vision.VisionPortal;

@Disabled
@Autonomous(name="Vision Test")
public class CameraTest extends LinearOpMode {

    private VisionPortal portal;
    private RedPropThreshold redPropThreshold;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        redPropThreshold = new RedPropThreshold();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(redPropThreshold)
                .build();


        while (opModeInInit()) {
            telemetry.addData("Prop Position", redPropThreshold.getPropPosition());
            telemetry.update();
        }

        waitForStart();
        telemetry.addData("Prop Position", redPropThreshold.getPropPosition());
        telemetry.update();                        //Will output prop position on Driver Station Console




    }
}
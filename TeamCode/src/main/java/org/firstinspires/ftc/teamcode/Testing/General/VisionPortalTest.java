package org.firstinspires.ftc.teamcode.Testing.General;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.Misc.YellowPreload;
import org.firstinspires.ftc.vision.VisionPortal;

//@Disabled
@Autonomous(group = "General")
public class VisionPortalTest extends LinearOpMode {

    private VisionPortal portal;
    private YellowPreload yellowPreload;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        yellowPreload = new YellowPreload();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(yellowPreload)
                .build();


        while (opModeInInit()) {
            telemetry.addData("Prop Position", yellowPreload.getCentroidX());
            telemetry.addData("Prop Position", yellowPreload.getCentroidY());
            telemetry.update();
        }

        waitForStart();
        telemetry.addData("Prop Position", yellowPreload.getCentroidX());
        telemetry.addData("Prop Position", yellowPreload.getCentroidY());
        telemetry.update();                        //Will output prop position on Driver Station Console




    }
}
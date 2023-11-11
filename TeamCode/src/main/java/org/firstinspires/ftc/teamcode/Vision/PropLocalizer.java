package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;
//@Config
public class PropLocalizer{

    Telemetry telemetry;
    HardwareMap hardwareMap;
    Gamepad gamepad1;
    public int loc = 0;
    private OpenCvWebcam webcam;
    int maxExp;
    int minExp;
    boolean blue = true;
    FtcDashboard dashboard;
    public static int curExp = 1250;
    int maxGain;
    int minGain;
    public static int curGain = 255;
    ExposureControl exposureControl;
    GainControl gainControl;
    boolean cameraOpened = false;

    public PropPipeline pipeline;
    public PropLocalizer(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, boolean blue,FtcDashboard dashboard) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        this.blue = blue;
        this.dashboard = dashboard;
        this.pipeline = new PropPipeline(blue,dashboard);
    }

    public void initLocalizer() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                pipeline = new PropPipeline( blue,dashboard);
                webcam.setPipeline(pipeline);
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

                exposureControl = webcam.getExposureControl();
                gainControl = webcam.getGainControl();

                maxExp = (int)exposureControl.getMaxExposure(TimeUnit.MICROSECONDS);
                minExp = (int) exposureControl.getMinExposure(TimeUnit.MICROSECONDS);
                maxGain = gainControl.getMaxGain();
                minGain = gainControl.getMinGain();

                gainControl.setGain(curGain);

                exposureControl.setAePriority(false);
                exposureControl.setMode(ExposureControl.Mode.Manual);
                exposureControl.setExposure(curExp, TimeUnit.MICROSECONDS);
                cameraOpened = true;
            }

            @Override
            public void onError(int errorCode) {

            }
        });


    }

    public void initLoop() {
        if (!cameraOpened) return;
        webcam.getPtzControl().setZoom(webcam.getPtzControl().getMinZoom());

        float changeExp = -gamepad1.left_stick_y/100;
        float changeGain = -gamepad1.right_stick_y/100;

        int changeExpInt = (int) (changeExp * 500);
        int changeGainInt = (int) (changeGain * 5);

        curExp += changeExpInt;
        curGain += changeGainInt;

        curExp = Math.max(curExp, minExp);
        curExp = Math.min(curExp, maxExp);

        curGain = Math.max(curGain, minGain);
        curGain = Math.min(curGain, maxGain);

        gainControl.setGain(curGain);
        exposureControl.setExposure(curExp, TimeUnit.MICROSECONDS);


        telemetry.addData("Frame Count", webcam.getFrameCount());
        telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
        telemetry.addLine("\nExposure: left stick; Gain: right stick");
        telemetry.addData("Exposure", "Min:%d, Max:%d, Cur:%d", minExp, maxExp, curExp);
        telemetry.addData("Gain", "Min:%d, Max:%d, Cur:%d", minGain, maxGain, curGain);
        //telemetry.update();

    }

    public void terminator() {
        webcam.stopStreaming();
        cameraOpened=false;
    }

    public int getLoc() {
        if (cameraOpened) loc = pipeline.locationTSE;
        if(!cameraOpened) loc = 0;
        return loc;
    }

    public long getExp() {
        return curExp;
    }
}
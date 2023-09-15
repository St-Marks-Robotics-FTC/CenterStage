package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//@Config
@Disabled
@TeleOp(name = "PROPLOCALIZER", group = "Prototype")
public class PropLocalizerExample extends LinearOpMode {
    PropLocalizer propLocalizer;
    int loc;
    FtcDashboard dashboard;
    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        propLocalizer = new PropLocalizer(telemetry, hardwareMap, gamepad1, false, dashboard);
        propLocalizer.initLocalizer();

        waitForStart();
        while (opModeIsActive()) {
            propLocalizer.initLoop();
            loc = propLocalizer.getLoc();
            telemetry.addData("loc: ", loc);
            telemetry.addData("exposure: ", propLocalizer.curExp);
            // Don't burn CPU cycles busy-looping in this sample
            idle();
        }

        propLocalizer.terminator();
    }
}

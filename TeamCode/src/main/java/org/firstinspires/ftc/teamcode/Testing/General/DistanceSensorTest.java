package org.firstinspires.ftc.teamcode.Testing.General;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.LebronBot.Subsystems.DistanceRelocalize;

@TeleOp
public class DistanceSensorTest extends OpMode {
    DistanceRelocalize relocalize;
    @Override
    public void init() {
        relocalize=new DistanceRelocalize(hardwareMap, DistanceRelocalize.Side.BLUE);
    }

    @Override
    public void loop() {
        telemetry.addData("relocalize Pose: ", relocalize.relocalize());
        telemetry.addData("distL", relocalize.getDistL());
        telemetry.addData("distR", relocalize.getDistR());
        telemetry.addData("distF", relocalize.getDistF());
        telemetry.addData("distLV", relocalize.getDistLV());
        telemetry.addData("distRV", relocalize.getDistRV());
        telemetry.addData("distFV", relocalize.getDistFV());
        telemetry.update();
    }
}

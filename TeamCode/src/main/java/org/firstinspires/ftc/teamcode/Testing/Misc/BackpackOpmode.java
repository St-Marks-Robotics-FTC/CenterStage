package org.firstinspires.ftc.teamcode.Testing.Misc;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * Created by Dryw Wade
 *
 * OpMode for testing Adafruit's MCP9808 temperature sensor driver
 */
@TeleOp(name = "MCP9808", group = "Tests")
public class BackpackOpmode extends LinearOpMode
{
    private OpenI2C adafruitLEDBackpack;

    public void runOpMode() throws InterruptedException
    {
        adafruitLEDBackpack = hardwareMap.get(OpenI2C.class, "backpack");

        // Uncomment to use parameter version of driver class. This will require you to respecify
        // the sensor type from MCP9808 to MCP9808Params
//        MCP9808Params.Parameters parameters = new MCP9808Params.Parameters();
//        parameters.hysteresis = MCP9808Params.Hysteresis.HYST_1_5;
//        parameters.alertControl = MCP9808Params.AlertControl.ALERT_ENABLE;
//        tempSensor.initialize(parameters);

        adafruitLEDBackpack.initialize();
        adafruitLEDBackpack.allOn();
        waitForStart();

        while(opModeIsActive())
        {
//            telemetry.addData("Temperature", adafruitLEDBackpack.getTemperature());
//            telemetry.addData("", "");
//
//            telemetry.addData("Lower Limit", adafruitLEDBackpack.getTemperatureLimit(MCP9808.Register.T_LIMIT_LOWER));
//            telemetry.addData("Lower Limit Triggered", adafruitLEDBackpack.lowerLimitTriggered());
//            telemetry.addData("Upper Limit", adafruitLEDBackpack.getTemperatureLimit(MCP9808.Register.T_LIMIT_UPPER));
//            telemetry.addData("Upper Limit Triggered", adafruitLEDBackpack.upperLimitTriggered());
//            telemetry.addData("Critical Limit", adafruitLEDBackpack.getTemperatureLimit(MCP9808.Register.T_LIMIT_CRITICAL));
//            telemetry.addData("Critical Limit Triggered", adafruitLEDBackpack.criticalLimitTriggered());
//            telemetry.addData("", "");
//
//            telemetry.addData("Config", Integer.toHexString(adafruitLEDBackpack.readShort(MCP9808.Register.CONFIGURATION)));
//            telemetry.addData("Manufacturer ID", adafruitLEDBackpack.getManufacturerIDRaw());

            telemetry.update();
//            idle();
        }
    }
}
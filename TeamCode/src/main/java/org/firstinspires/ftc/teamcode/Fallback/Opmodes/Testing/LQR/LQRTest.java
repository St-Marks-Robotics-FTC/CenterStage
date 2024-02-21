package org.firstinspires.ftc.teamcode.Fallback.Opmodes.Testing.LQR;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.ejml.simple.SimpleMatrix;

@Disabled
@Config
@TeleOp
public class LQRTest extends OpMode {

    DcMotorEx motor;
    String motorName = "liftLeft";

    private double maxError;
    private double minVolt;

    @Override
    public void init() {
        motor=hardwareMap.get(DcMotorEx.class, motorName);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            SimpleMatrix a = new SimpleMatrix(1, 1);
            a.set(0, 0, 1);
            SimpleMatrix b = new SimpleMatrix(1, 1);
            a.set(0, 0, 1.5);
            SimpleMatrix q = new SimpleMatrix(1, 1);
            a.set(0, 0, 0.1);
            SimpleMatrix r = new SimpleMatrix(1, 1);
            a.set(0, 0, 12);
            LQR lqr = new LQR(a, b, q, r, 0.02);
            SimpleMatrix K = lqr.calculateK();
            double k = K.get(0, 0);
            motor.setPower(k*(1000-motor.getCurrentPosition()));
        }
    }
}

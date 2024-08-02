package org.firstinspires.ftc.teamcode.LebronBot.Opmodes.Testing.LQR;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.ejml.simple.SimpleMatrix;

//@Disabled
@Config
@TeleOp
public class LQRTest extends OpMode {

    private double maxError;
    private double minVolt;
    private double targetVelocity = 8;

    @Override
    public void init() {
        double Kv = 1;
        double Ka = 1.5;
        SimpleMatrix a = new SimpleMatrix(1, 1);
        a.set(0, 0, 1);
        SimpleMatrix b = new SimpleMatrix(1, 1);
        b.set(0, 0, 0.3);
        SimpleMatrix q = new SimpleMatrix(1, 1);
        q.set(0, 0, 0.2);
        SimpleMatrix r = new SimpleMatrix(1, 1);
        r.set(0, 0, 0.1);
        LQR lqr = new LQR(a, b, q, r, 0.02);
        SimpleMatrix K = lqr.calculateK();
        //should be 1.34
//        K.printDimensions();
        double k = K.get(0, 0);
        Log.d("k: ", Double.toString(k));
    }

    @Override
    public void loop() {
//        if (gamepad1.a) {
//            SimpleMatrix a = new SimpleMatrix(1, 1);
//            a.set(0, 0, 1);
//            SimpleMatrix b = new SimpleMatrix(1, 1);
//            a.set(0, 0, 1.5);
//            SimpleMatrix q = new SimpleMatrix(1, 1);
//            a.set(0, 0, 0.1);
//            SimpleMatrix r = new SimpleMatrix(1, 1);
//            a.set(0, 0, 12);
//            LQR lqr = new LQR(a, b, q, r, 0.02);
//            SimpleMatrix K = lqr.calculateK();
//            double k = K.get(0, 0);
//            motor.setPower(k*(targetVelocity-motor.getVelocity()));
//        } else {
//            motor.setPower(0);
//        }
//        double Kv = 1;
//        double Ka = 1.5;
//        SimpleMatrix a = new SimpleMatrix(1, 1);
//        a.set(0, 0, -Kv/Ka);
//        SimpleMatrix b = new SimpleMatrix(1, 1);
//        b.set(0, 0, 1/Ka);
//        SimpleMatrix q = new SimpleMatrix(1, 1);
//        q.set(0, 0, 0.1);
//        SimpleMatrix r = new SimpleMatrix(1, 1);
//        r.set(0, 0, 12);
//        LQR lqr = new LQR(a, b, q, r, 0.02);
//        SimpleMatrix K = lqr.calculateK();
//        //should be 1.34
////        K.printDimensions();
//        double k = K.get(0, 0);
//        Log.d("k: ", Double.toString(k));
//        Log.d("motor velocity: ", Double.toString(motor.getVelocity()));
//        motor.setPower(k*70);
//        telemetry.addData("velocity: ", motor.getVelocity());
    }
}

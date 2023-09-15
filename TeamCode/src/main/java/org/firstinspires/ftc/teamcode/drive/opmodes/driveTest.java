package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.drive.swerve.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.drive.drive.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.drive.geometry.Point;
import org.firstinspires.ftc.teamcode.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.drive.localizer.TwoWheelLocalizer;

@TeleOp
public class driveTest extends OpMode {

    SwerveDrivetrain drive;
    SlewRateLimiter fw;
    SlewRateLimiter str;
    TwoWheelLocalizer local;

    public static double fw_r = 4;
    public static double str_r = 4;

    @Override
    public void init() {
        drive = new SwerveDrivetrain(hardwareMap);
        fw = new SlewRateLimiter(fw_r);
        str = new SlewRateLimiter(str_r);
        local = new TwoWheelLocalizer(hardwareMap);
    }

    @Override
    public void loop() {
        drive.read();
        drive.getTelemetry();

        double rotationAmount = (true) ? local.getHeading() - SwerveDrivetrain.imuOffset : 0;
        double turn = gamepad1.right_trigger - gamepad1.left_trigger;
        Pose driving = new Pose(
                new Point(joystickScalar(gamepad1.left_stick_y, 0.001),
                        joystickScalar(gamepad1.left_stick_x, 0.001)).rotate(rotationAmount),
                        joystickScalar(turn, 0.01)
        );

        driving = new Pose(
                fw.calculate(driving.x),
                str.calculate(driving.y),
                driving.heading
        );
        drive.set(driving);
        drive.write();
    }

    private double joystickScalar(double num, double min) {
        return joystickScalar(num, min, 0.66, 4);
    }

    private double joystickScalar(double n, double m, double l, double a) {
        return Math.signum(n) * m
                + (1 - m) *
                (Math.abs(n) > l ?
                        Math.pow(Math.abs(n), Math.log(l / a) / Math.log(l)) * Math.signum(n) :
                        n / a);
    }
}

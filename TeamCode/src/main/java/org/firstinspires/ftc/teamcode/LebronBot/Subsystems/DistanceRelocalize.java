package org.firstinspires.ftc.teamcode.LebronBot.Subsystems;

import static org.firstinspires.ftc.teamcode.LebronBot.Subsystems.DistanceRelocalize.Side.BLUE;
import static org.firstinspires.ftc.teamcode.LebronBot.Subsystems.DistanceRelocalize.Side.RED;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.LebronBot.Roadrunner.DriveConstants;

//relocalizes the robot in the warehouse based on distance sensor angles and alliance color
public class DistanceRelocalize {

    //1 is front, 2 is right, 3 is left
    double angle1=0, angle2=Math.toRadians(-90), angle3=Math.toRadians(90);
    public double angle=0;
    Pose2d oldPose= new Pose2d(0,0,0);
    Pose2d[] sensors={new Pose2d(7.375,6.1875,Math.toRadians(0)), new Pose2d(3,-7.1875,Math.toRadians(-90)), new Pose2d(3.875,7.1875,Math.toRadians(90))};

    public enum Side {
        RED, BLUE
    }
    Side side;
//    public DistanceSensor distF;
//    public DistanceSensor distR;
//    public DistanceSensor distL;
    //public AsyncMB1242 distF;

    public DistanceDriver distF;
    public DistanceDriver distR;
    public DistanceDriver distL;


    private IMU imu;

    public DistanceRelocalize(HardwareMap hardwareMap, Side side) {
        //if (BaseAutonomous.autorunning)
        distF = new DistanceDriver(hardwareMap, "distF");
        distR = new DistanceDriver(hardwareMap, "distR");
        distL = new DistanceDriver(hardwareMap, "distL");

        imu = hardwareMap.get(IMU.class, "adafruit_imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
        imu.initialize(parameters);
        /*telemetry.addData("Distance", asyncSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Last Reading", asyncSensor.getLastMeasurementTimestamp());
            if(gamepad1.a){
                asyncSensor.setSensorAccuracyMode(AsyncRev2MSensor.AccuracyMode.MODE_HIGH_ACCURACY);*/
        this.side = side;
    }

    private double getAngle(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void setSide (Side side){
        this.side=side;
    }

    public void setSide (boolean side){
        if (side) {
            setSide(BLUE);
        } else {
            setSide(RED);
        }
    }

    public Pose2d relocalize(){

        //i was gonna make this more complicated but i want to keep it simple for now, so just assume the distance sensors are both facing the correct walls
        angle=getAngle()+((side==RED)?Math.toRadians(180):0);
        double dist1 = getDistF();///Math.cos(Math.toRadians(20));

        if (dist1>100) return new Pose2d(999,0,0);

        double dist2,dist3;
        double x=Math.cos(angle+angle1)*dist1;
        double xOff=sensors[0].vec().rotated(angle).getX();
        x+=xOff;
        double y;

        if(side == BLUE){
            dist2 = getDistR();
            if (dist2>100) return new Pose2d(999,0,0);

            y=-Math.cos(Math.toRadians(90)+(angle2+angle))*dist2;
            double yOff=sensors[1].vec().rotated(angle).getY();
            Log.d("y, ", Double.toString(y));
            y+=yOff;
            Log.d("yOff, ", Double.toString(yOff));
        }
        else{
            dist3 = getDistL();
            if (dist3>100) return new Pose2d(999,0,0);

            y=Math.cos(Math.toRadians(90)-(angle3+angle))*dist3;
            double yOff=sensors[2].vec().rotated(angle).getY();
            y+=yOff;
        }
        Pose2d pose=new Pose2d(x,y,angle);
        Log.d("pose: ", pose.toString());
        if(side == BLUE){
            pose=new Pose2d(-72+pose.getX(),72+pose.getY(), pose.getHeading());
        }else{
            pose=new Pose2d(-72+pose.getX(),-72-pose.getY(), pose.getHeading());
        }
        oldPose=pose;
        return pose;
    }

    //get distF
    public double getDistF(){
        return distF.getDistance()/25.4;
    }

    //get distR
    public double getDistR(){
        return distR.getDistance()/25.4;
    }

    //get distL
    public double getDistL(){
        return distL.getDistance()/25.4;
    }
    public double getDistFV(){
        return distF.getVoltage();
    }

    //get distR
    public double getDistRV(){
        return distR.getVoltage();
    }

    //get distL
    public double getDistLV(){
        return distL.getVoltage();
    }

    public Side getSide() {
        return side;
    }

}


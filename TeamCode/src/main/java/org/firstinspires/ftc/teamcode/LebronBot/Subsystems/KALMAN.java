package org.firstinspires.ftc.teamcode.LebronBot.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.ejml.simple.SimpleMatrix;

/**
 * Extended Kalman Filter (EKF) for sensor fusion between odometry and distance sensor
 * Odometry is used as prediction to generate estimate
 * Distance Sensor is used as measurement to generate correction
 */
@Config
public class KALMAN {

    public SimpleMatrix[] pose; // pose, cov

    // values
    private final SimpleMatrix STARTING_COVARIANCE = new SimpleMatrix(new double[][]{
            {1, 0, 0},
            {0, 1, 0},
            {0, 0, 1}
    });

    private SimpleMatrix P;

    public KALMAN(SimpleMatrix[] pose) {
        this.pose = pose;
    }

    public KALMAN(SimpleMatrix pose, SimpleMatrix cov) {
        this.pose = new SimpleMatrix[]{pose,cov};
    }

    public KALMAN(Pose2d pose) {
        SimpleMatrix poseMatrix = new SimpleMatrix(new double[][]{
                {pose.getX()},
                {pose.getY()},
                {pose.getHeading()}
        });
        this.pose = new SimpleMatrix[]{poseMatrix,STARTING_COVARIANCE};
    }

    public void update(Pose2d update, Pose2d obs){
        SimpleMatrix updateMatrix = new SimpleMatrix(new double[][]{
                {update.getX()},
                {update.getY()},
                {update.getHeading()}
        });
        SimpleMatrix obsMatrix = new SimpleMatrix(new double[][]{
                {obs.getX()},
                {obs.getY()},
                {obs.getHeading()}
        });
        if (update != null && obs == null) pose = prediction(pose,updateMatrix);
        if (update == null && obs != null) pose = correction(pose, obsMatrix);
        if (update != null && obs != null) pose = fuse(pose,updateMatrix,obsMatrix);
    }

    public SimpleMatrix[] prediction(SimpleMatrix[] prev, SimpleMatrix update) {

        // set up
        double prevX = prev[0].get(0,0);
        double prevY = prev[0].get(1,0);
        double prevHeading = prev[0].get(2,0);
        SimpleMatrix prevCov = prev[1];

        SimpleMatrix F = new SimpleMatrix(new double[][]{
                {1, 0, 0},
                {0, 1, 0},
                {0, 0, 1}
        });
        //odometry prediction
        SimpleMatrix Q = new SimpleMatrix(new double[][]{
                {0.1, 0, 0},
                {0, 0.1, 0},
                {0, 0, 0.9}
        });
        P = F.mult(prevCov).mult(F.transpose()).plus(Q);

        // calculate
        double predX = update.get(0, 0);
        double predY = update.get(1,0);
        double predHeading = angleWrap(update.get(2,0));

        return new SimpleMatrix[]{
                new SimpleMatrix(new double[][]{
                        {predX},
                        {predY},
                        {predHeading}
                }),
                P
        };
    }
    //obs = observation
    public SimpleMatrix[] correction(SimpleMatrix[] pred, SimpleMatrix obs) {
        // set up
        double tX = obs.get(0,0);
        double tY = obs.get(1,0);
        double tPhi = obs.get(2,0);
        SimpleMatrix predCov = pred[1];

        SimpleMatrix i;
        SimpleMatrix z = new SimpleMatrix(new double [][]{
                {tX},
                {tY},
                {tPhi}
        });
        SimpleMatrix H = new SimpleMatrix(new double[][]{
                {1, 0, 0},
                {0, 1, 0},
                {0, 0, 1}
        });
        //sensor noise
        SimpleMatrix w = new SimpleMatrix(new double[][]{
                {0.4},
                {0.4},
                {0}
        });
        i = z.minus(H.mult(pred[0]).plus(w));
        //covariance
        SimpleMatrix Q = new SimpleMatrix(new double[][]{
                {1, 0, 0},
                {0, 1, 0},
                {0, 0, 1}
        }); // random

        SimpleMatrix S = H.mult(predCov).mult(H.transpose()).plus(Q);
        SimpleMatrix K = predCov.mult(H.transpose().mult(S.invert()));

        SimpleMatrix correct = pred[0].plus(K.mult(i));
        correct.set(2,0, angleWrap(correct.get(2,0)));
        SimpleMatrix correctCov = predCov.minus(K.mult(H).mult(predCov));

        return new SimpleMatrix[]{correct,correctCov};
    }

    public SimpleMatrix[] fuse(SimpleMatrix[] prev, SimpleMatrix update, SimpleMatrix obs) {
        SimpleMatrix[] prediction = prediction(prev, update);
        return correction(prediction, obs);
    }

    private double angleWrap(double input) {
        input%=360;
        if (input>180) input-=360;
        return input;
    }

    public Pose2d getPose() {
        double x = pose[0].get(0, 0);
        double y = pose[0].get(1, 0);
        double z = pose[0].get(2, 0);
        return new Pose2d(x, y, z);
    }
}

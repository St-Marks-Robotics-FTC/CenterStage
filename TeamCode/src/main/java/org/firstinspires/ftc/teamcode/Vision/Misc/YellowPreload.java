package org.firstinspires.ftc.teamcode.Vision.Misc;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

public class YellowPreload implements VisionProcessor {

    double centroidX = 0;
    double centroidY = 0;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        // Convert input image to HSV color space
        Mat hsvImage = new Mat();
        Imgproc.cvtColor(frame, hsvImage, Imgproc.COLOR_RGB2HSV);

        // Define lower and upper bounds for yellow color in HSV
        Scalar lowerYellow = new Scalar(20, 100, 100);
        Scalar upperYellow = new Scalar(30, 255, 255);

        // Threshold the HSV image to get only yellow color
        Mat yellowMask = new Mat();
        Core.inRange(hsvImage, lowerYellow, upperYellow, yellowMask);

        // Find contours in the thresholded image
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find the biggest contour
        double maxArea = 0;
        MatOfPoint biggestContour = null;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                biggestContour = contour;
            }
        }

        // Get the centroid of the biggest contour
        Point centroid = new Point();
        if (biggestContour != null) {
            Moments moments = Imgproc.moments(biggestContour);
            centroid.x = moments.get_m10() / moments.get_m00();
            centroid.y = moments.get_m01() / moments.get_m00();

            centroidX = centroid.x;
            centroidY = centroid.y;
        }

        // Draw centroid on the input image
        if (biggestContour != null) {
            Imgproc.circle(frame, centroid, 5, new Scalar(0, 255, 0), -1);
        }

        // Release the Mats
        hsvImage.release();
        yellowMask.release();
        hierarchy.release();


        return null;            //You do not return the original mat anymore, instead return null




    }


    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public double getCentroidX() {
        return centroidX;
    }

    public double getCentroidY() {
        return centroidY;
    }
}
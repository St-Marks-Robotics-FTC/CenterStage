package org.firstinspires.ftc.teamcode.Vision.EOCVSIM;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class TutorialPipeline extends OpenCvPipeline {

    private Mat HSVMat = new Mat();
    private Mat binaryMat = new Mat();
    private Mat maskedInputMat = new Mat();
    private Mat hierarchy = new Mat();

    private boolean blue = false;

    public Scalar lowerRed = new Scalar(168.1, 107.7, 52.5);
    public Scalar upperRed = new Scalar(188, 255, 140.3);

    public Scalar lowerBlue = new Scalar(0,0,0);
    public Scalar upperBlue = new Scalar(0,0,0);
    private Telemetry telemetry;
    public TutorialPipeline (Telemetry telemetry, boolean blue){
        this.telemetry = telemetry;
        this.blue = blue;
    }
    @Override
    public void init(Mat mat) {
        super.init(mat);
    }

    @Override
    public Mat processFrame(Mat source) {
        //List of Contours
        ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        //Converts RGB to HSV
        Imgproc.cvtColor(source, HSVMat, Imgproc.COLOR_RGB2HSV);
        //Filter colors
        if (blue) {
            Core.inRange(HSVMat, lowerBlue, upperBlue, binaryMat);
        } else {
            Core.inRange(HSVMat, lowerRed, upperRed, binaryMat);
        }
        //Finds Contours
        Imgproc.findContours(binaryMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        maskedInputMat.release();
        //Imgproc.drawContours(source, contours, 0, new Scalar(0, 255, 0));
        //Core.bitwise_and(source, source, maskedInputMat, binaryMat);
        //rectangles from the contours
        ArrayList<Rect> rectangles = new ArrayList<>();
        //Finds rectangles around the contours
        rectangles = analyzeRect(contours);
        for (Rect rect : rectangles) {
            //Draws the rectangle
            Imgproc.rectangle(source, rect, new Scalar(0, 255, 0), 2);
            //Find the center of the rectangle
            Point retPoint = new Point(rect.x + (double) rect.width / 2, rect.y + (double) rect.height / 2);
            //Output where the rectangle is (0: left, 1:middle, 2: right)
            telemetry.addData("Location TSE: ", getLocation(retPoint));
        }
        telemetry.update();
        return source;
    }

    public ArrayList<Rect> analyzeRect(ArrayList<MatOfPoint> contours) {
        ArrayList<Rect> rectangles = new ArrayList<Rect>();
        for (MatOfPoint contour: contours) {
            //draws bounding rectangle
            Rect boundingRect = Imgproc.boundingRect(contour);
            //if the area is less than 100 exclude the rectangle
            if (boundingRect.area()>100) {
                rectangles.add(boundingRect);
            }
        }
        return rectangles;
    }

    public int getLocation(Point a) {
        //basic math with pixel location
        if (a.x<100) {
            return 0;
        } else if (a.x<200) {
            return 1;
        } else {
            return 2;
        }
    }
}

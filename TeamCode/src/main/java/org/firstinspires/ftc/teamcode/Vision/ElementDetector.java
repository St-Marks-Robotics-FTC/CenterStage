package org.firstinspires.ftc.teamcode.Vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


public class ElementDetector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location {
        MIDDLE,
        RIGHT,
        LEFT
    }
    private Location location;

    static final Rect MIDDLE_ROI = new Rect(
            //100,200
            //175,270
            new Point(80, 180),
            new Point(195, 290));
    static final Rect RIGHT_ROI = new Rect(
            //430,210
            //500,280
            new Point(410, 190),
            new Point(520, 300));
    static double PERCENT_COLOR_THRESHOLD = 0.2;

    public ElementDetector(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(23, 50, 70);
        Scalar highHSV = new Scalar(325, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat middle = mat.submat(MIDDLE_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        double middleValue = Core.sumElems(middle).val[0] / MIDDLE_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        middle.release();
        right.release();

        telemetry.addData("Middle raw value", (int) Core.sumElems(middle).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Middle percentage", Math.round(middleValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");

        boolean elementMiddle = middleValue > PERCENT_COLOR_THRESHOLD;
        boolean elementRight = rightValue > PERCENT_COLOR_THRESHOLD;

        if (elementMiddle) {
            location = Location.MIDDLE;
            telemetry.addData("Element Location", "middle");
        }
        else if (elementRight) {
            location = Location.RIGHT;
            telemetry.addData("Element Location", "right");
        }
        else {
            location = Location.LEFT;
            telemetry.addData("Element Location", "left");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorElement = new Scalar(255, 0, 0);
        Scalar colorBlank = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, MIDDLE_ROI, location == Location.LEFT? colorBlank:colorElement);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? colorBlank:colorElement);

        return mat;
    }

    public Location getLocation() {
        return location;
    }
}
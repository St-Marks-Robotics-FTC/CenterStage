package org.firstinspires.ftc.teamcode.Vision.EOCVSIM;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class pixelcoord extends OpenCvPipeline {

    private Telemetry telemetry = null;
    ArrayList<MatOfPoint2f> src = new ArrayList<>();

    public Scalar lower = new Scalar(225, 125, 125);
    public Scalar upper = new Scalar(255, 137, 137);
    //(253,395) (115, 39)

    public pixelcoord(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat output = new Mat();
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2YCrCb);
        Core.inRange(input, lower, upper, output);
//       answer:  : {528, 593, 193x47}
        double xo = 539;
        double yo = 593;
        double wo = 193;
        double ho = 57;
        List<Point> srcPointsList = new ArrayList<>();
        srcPointsList.add(new Point(xo, yo));
        srcPointsList.add(new Point(xo+wo, yo));
        srcPointsList.add(new Point(xo, yo+ho));
        srcPointsList.add(new Point(xo+wo, yo+ho));

        MatOfPoint2f srcPoints = new MatOfPoint2f();
        srcPoints.fromList(srcPointsList);

        // Define destination points
//        double x = 923;
//        double y = 589;
//        double w = 204;
//        double h = 64;
        double x = 539;
        double y = 593;
        double w = 193;
        double h = 57;
        List<Point> dstPointsList = new ArrayList<>();
        dstPointsList.add(new Point(x, y));
        dstPointsList.add(new Point(x+w, y));
        dstPointsList.add(new Point(x, y+h));
        dstPointsList.add(new Point(x+w, y+h));
        //answer:  : {979, 590, 204x64}

        MatOfPoint2f dstPoints = new MatOfPoint2f();
        dstPoints.fromList(dstPointsList);

        // Compute the homography matrix
        Mat homography = Calib3d.findHomography(srcPoints, dstPoints, Calib3d.RANSAC, 5);
        telemetry.addData("Homography", homography.dump());
        Mat vector = new Mat(3, 1, CvType.CV_64F);
        vector.put(0, 0, 23); // x-coordinate
        vector.put(1, 0, 0); // y-coordinate
        vector.put(2, 0, 1); // homogeneous coordinate (usually 1 for points)
        // Multiply the homography matrix by the vector
        Mat transformedVector = new Mat();
        Core.gemm(homography, vector, 1, new Mat(), 0, transformedVector);

        // Normalize the result by dividing by the third coordinate if it's not 1 (homogeneous coordinate)
        Core.divide(transformedVector, new Scalar(transformedVector.get(2, 0)[0]), transformedVector);
        // Print the vector
        telemetry.addData("Vector", transformedVector.dump());
        return input;
    }
}

package org.firstinspires.ftc.teamcode.Vision.EOCVSIM;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
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
//        answer:  : {248, 561, 588x70}
        double xo = 248;
        double yo = 561;
        double wo = 588;
        double ho = 70;
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
        double x = 24.5;
        double y = -3.5;
        double w = 8;
        double h = 10.5;
        List<Point> dstPointsList = new ArrayList<>();
        dstPointsList.add(new Point(x, y));
        dstPointsList.add(new Point(x+w, y));
        dstPointsList.add(new Point(x, y+h));
        dstPointsList.add(new Point(x+w, y+h));
        //answer:  : {979, 590, 204x64}

        MatOfPoint2f dstPoints = new MatOfPoint2f();
        dstPoints.fromList(dstPointsList);

        // Compute the homography matrix
        Mat homography = Calib3d.findHomography(srcPoints, dstPoints, Calib3d.RANSAC, 0.5);
        telemetry.addData("Homography", homography.dump());
        Mat vector = new Mat(3, 1, CvType.CV_64F);
        vector.put(0, 0, 23); // x-coordinate
        vector.put(1, 0, 0); // y-coordinate
        vector.put(2, 0, 1); // homogeneous coordinate (usually 1 for points)
        // Multiply the homography matrix by the vector
        Mat transformedVector = new Mat();
//        Core.gemm(homography, vector, 1, new Mat(), 0, transformedVector);
//
//        // Normalize the result by dividing by the third coordinate if it's not 1 (homogeneous coordinate)
//        Core.divide(transformedVector, new Scalar(transformedVector.get(2, 0)[0]), transformedVector);
        double a[][] = new double[3][3];
        for (int i = 0; i<3; i++) {
            for (int j = 0; j<3; j++) {
                a[i][j]=homography.get(i, j)[0];
            }
        }
        double b[] = {23, 0, 1};
        RealMatrix matrix = new Array2DRowRealMatrix(a);
        RealVector vect = new ArrayRealVector(b);
        DecompositionSolver solver = new LUDecomposition(matrix).getSolver();
        RealVector solution= solver.solve(vect);
        // Print the vector
//        Imgproc.warpPerspective(input, input, homography, srcPoints.size());
        // Example point to transform
        Point pt = new Point(933, 543); //542, 596
        //933 543
        // Transform point using homography
        Mat ptMat = new Mat(1, 1, CvType.CV_64FC2);
        ptMat.put(0, 0, new double[]{pt.x, pt.y});

        Mat transformedPtMat = new Mat();
        Core.perspectiveTransform(ptMat, transformedPtMat, homography);

        double[] transformedPt = transformedPtMat.get(0, 0);
        Point transformedPoint = new Point(transformedPt[0], transformedPt[1]);
        telemetry.addData("Vector", transformedPoint.toString());
        return input;
    }
}

package org.firstinspires.ftc.teamcode.Vision.Misc;

import static org.firstinspires.ftc.teamcode.Vision.EOCVSIM.VisionUtil.cvDilate;
import static org.firstinspires.ftc.teamcode.Vision.EOCVSIM.VisionUtil.cvErode;
import static org.firstinspires.ftc.teamcode.Vision.EOCVSIM.VisionUtil.findContours;

import android.graphics.Canvas;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class FullPixelDetection implements VisionProcessor {
    public static double location;
    public static boolean blue = false;
    public static double filterContoursMinArea = 00.0;
    public static double filterContoursMinPerimeter = 0.0;
    public static double filterContoursMinWidth = 0.0;
    public static double filterContoursMinHeight = 0.0;
    public static double filterContoursMaxHeight = 9000.0;
    public static double filterContoursMaxWidth = 9000.0;
    public static double filterContoursMinX = 0.0;
    public static double filterContoursMaxX = 9000.0;
    public static double filterContoursMinY = 0.0;
    public static double filterContoursMinYBlue = 0.0;
    public static double filterContoursMaxY = 9000.0;
    public static int leftSep = 109;
    public static int rightSep = 218;
    public static double cvThresholdThresh = 0;
    public static double cvThresholdThreshBlue = 120;
    public static int cutoffLine =400;

    public static double cvThresholdMaxval = 255.0;
    public static double cvDilateIterations = 1.0;
    public static double cvErodeIterations = 1;

    public static int targetLinePos = 170;
    public static int idealWidth = 32;

    public static double maxY = 320;
    public static boolean invertThreshold = false;
    public static double cvExtractchannelChannel = 2.0;
    public static double cvExtractchannelChannelRed = 1.0;
    public static int cvCvtcolorCode = Imgproc.COLOR_RGB2YCrCb;
    public boolean found = false;
    public static boolean blueDash = true;
    public static double nearTol = 30;
    //Outputs

    ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
    List<MatOfPoint> filterContoursOutput = new ArrayList<MatOfPoint>();
    public Mat[] pipelineImg = new Mat[20];

    public static boolean dashboardEnabled = false;
    public static int dashboardImg = 7;
    public static boolean blueSide = true;
    Mat cvErodeKernel,cvDilateKernel;
    Mat displayMat;
    Mat blueWorkingMat, redWorkingMat;
    Mat cvCvtcolorOutput = new Mat();
    Mat cvExtractchannelOutput = new Mat();
    Mat cvThresholdOutput = new Mat();
    Mat cvErodeOutput = new Mat();
    Mat cvDilateOutput = new Mat();
    Rect boundingRect = new Rect();
    public int locationTSE = 0;
    static boolean processing = true;
    static boolean work = false;
    boolean[] covered = {true, true, true};
    public Scalar lower = new Scalar(235, 125, 125);
    public Scalar upper = new Scalar(255, 133, 133);
    private Rect answer = new Rect();
    private Point output = new Point();
    private Vector2d pixelcoord = new Vector2d();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    public Point analyzeRect(Mat src, Rect rect) {

        if (rect.width < 5 || rect.height < 5|| rect.y<cutoffLine) {
            return new Point(-69, -69);
        }
        Point retPoint = new Point(rect.x + (double) rect.width / 2, rect.y + (double) rect.height / 2);
        return retPoint;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        if(frame.width() == 0) return new Mat();
        invertThreshold  = blue;
        pipelineImg[0] = frame;// Step CV_cvtColor0:
        blueWorkingMat = frame;
//            cvCvtcolor(source0, cvCvtcolorCode, cvCvtcolorOutput);
        Imgproc.cvtColor(frame, cvCvtcolorOutput, Imgproc.COLOR_RGB2YCrCb);
        pipelineImg[1] = cvCvtcolorOutput;
        blueWorkingMat = cvCvtcolorOutput;
        List<Mat> labChannels = new ArrayList<>();
        Core.split(blueWorkingMat, labChannels);
        Imgproc.equalizeHist(labChannels.get(2), labChannels.get(2));
        Core.merge(labChannels, blueWorkingMat);

//            cvExtractchannel(blueWorkingMat, blue?cvExtractchannelChannel:cvExtractchannelChannelRed, cvExtractchannelOutput);
//            pipelineImg[2] = cvExtractchannelOutput;
//            blueWorkingMat = cvExtractchannelOutput;

        Core.inRange(blueWorkingMat, lower, upper, cvThresholdOutput);

        blueWorkingMat = cvThresholdOutput;
        pipelineImg[3] = cvThresholdOutput;
        // Step CV_erode0:

        cvErodeKernel = new Mat();

        cvErode(blueWorkingMat, cvErodeKernel, new Point(-1, -1), cvErodeIterations, Core.BORDER_CONSTANT, new Scalar(-1), cvErodeOutput);
        blueWorkingMat = cvErodeOutput;

        pipelineImg[4] = cvErodeOutput;

        // Step CV_dilate0:

        cvDilateKernel = new Mat();

        cvDilate(blueWorkingMat, cvDilateKernel, new Point(-1, -1), cvDilateIterations, Core.BORDER_CONSTANT, new Scalar(-1), cvDilateOutput);
        blueWorkingMat = cvDilateOutput;
        pipelineImg[5] = cvDilateOutput;


        // Step Find_Contours0:
        Mat showContours = new Mat(frame.rows(), frame.cols(),frame.type());

        findContours(blueWorkingMat, true, findContoursOutput);

        pipelineImg[6] = showContours;
        displayMat = frame;
        Imgproc.drawContours(displayMat, findContoursOutput, -1, new Scalar (0,255,0));
        //telemetry.addData("Contour list size: ", findContoursOutput.size());

        ArrayList<Rect> filterOutput = (ArrayList<Rect>) filterContours(findContoursOutput);
//        telemetry.addData("rect list size: ", filterOutput.size());
        double highest = 0;
        answer = new Rect(-69, -69, -69, -69);
        for (Rect a : filterOutput) {
            Point found = analyzeRect(frame, a);
//                telemetry.addData("found: ", found);
            //telemetry.addData("highest", highest);
//                Imgproc.rectangle(displayMat,a,new Scalar (0,255,0),2);
            if (found.x != -69) {//hohahehhehehe
                Imgproc.rectangle(displayMat,a,new Scalar (0,255,0),2);
                if (a.area()>answer.area()) {
                    answer=a;
                }
            }
        }
        if (answer.x==-69) {
            pixelcoord = new Vector2d(-69, -69);
            return null;
        } else {
            output = new Point(answer.x + (double) answer.width / 2, answer.y + (double) answer.height / 2);
        }

        //homography time
        List<Point> srcPointsList = new ArrayList<>();
        srcPointsList.add(new Point(478,560));
        srcPointsList.add(new Point(478+463, 560));
        srcPointsList.add(new Point(429, 563+71));
        srcPointsList.add(new Point(429+590, 563+71));
        MatOfPoint2f srcPoints = new MatOfPoint2f();
        srcPoints.fromList(srcPointsList);
        double x = 24.5;
        double y = -3.5;
        double w = 8;
        double h = 10.5;
        List<Point> dstPointsList = new ArrayList<>();
        dstPointsList.add(new Point(x+w, y));
        dstPointsList.add(new Point(x+w, y+h));
        dstPointsList.add(new Point(x, y));
        dstPointsList.add(new Point(x, y+h));
        MatOfPoint2f dstPoints = new MatOfPoint2f();
        dstPoints.fromList(dstPointsList);
        Mat homography = Calib3d.findHomography(srcPoints, dstPoints, Calib3d.RANSAC, 0.5);
        Point pt = new Point(output.x, output.y);
        Mat ptMat = new Mat(1, 1, CvType.CV_64FC2);
        ptMat.put(0, 0, new double[]{pt.x, pt.y});
        Mat transformedPtMat = new Mat();
        Core.perspectiveTransform(ptMat, transformedPtMat, homography);
        double[] transformedPt = transformedPtMat.get(0, 0);
        Point transformedPoint = new Point(transformedPt[0], transformedPt[1]);
        pixelcoord=new Vector2d(transformedPoint.x, -transformedPoint.y);
        showContours.release();
        return null;
    }

    public Vector2d getPixelcoord() {
        return pixelcoord;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    private List<Rect> filterContours(List<MatOfPoint> inputContours) {
        List<Rect> outputContours = new ArrayList<>();
        for (MatOfPoint contour : inputContours) {
            final Rect bb = Imgproc.boundingRect(contour);
            if(bb.x < filterContoursMinX || bb.x > filterContoursMaxX) continue;
//            if(bb.y < (blue?filterContoursMinYBlue:filterContoursMinY) || bb.y > filterContoursMaxY) continue;
            if (bb.width < filterContoursMinWidth || bb.width > filterContoursMaxWidth) continue;
            if (bb.height < filterContoursMinHeight || bb.height > filterContoursMaxHeight) continue;
            final double area = Imgproc.contourArea(contour);
            if (area < filterContoursMinArea) continue;
            if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < filterContoursMinPerimeter) continue;
            outputContours.add(bb);
        }
        return outputContours;
    }
}

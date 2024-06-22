package org.firstinspires.ftc.teamcode.Vision.EOCVSIM;

import static org.firstinspires.ftc.teamcode.Vision.EOCVSIM.VisionUtil.cvCvtcolor;
import static org.firstinspires.ftc.teamcode.Vision.EOCVSIM.VisionUtil.cvDilate;
import static org.firstinspires.ftc.teamcode.Vision.EOCVSIM.VisionUtil.cvErode;
import static org.firstinspires.ftc.teamcode.Vision.EOCVSIM.VisionUtil.cvExtractchannel;
import static org.firstinspires.ftc.teamcode.Vision.EOCVSIM.VisionUtil.cvThreshold;
import static org.firstinspires.ftc.teamcode.Vision.EOCVSIM.VisionUtil.findContours;

import android.graphics.Bitmap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;

public class PropPipeline extends OpenCvPipeline {
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
    private Telemetry telemetry;
    public Scalar lower = new Scalar(200, 125, 125);
    public Scalar upper = new Scalar(255, 137, 137);
    private Rect answer = new Rect();
    public PropPipeline (Telemetry telemetry){
        this.telemetry = telemetry;
    }

    public void stopProcessing() {
        processing = false;
    }

    public void startProcessing() {
        processing = true;
    }

    public Point analyzeRect(Mat src, Rect rect) {

        if (rect.width < 5 || rect.height < 5|| rect.y<cutoffLine) {
            return new Point(-69, -69);
        }
        Point retPoint = new Point(rect.x + (double) rect.width / 2, rect.y + (double) rect.height / 2);
        return retPoint;
    }

    @Override
    public void init(Mat firstFrame) {

    }

    @Override
    public Mat processFrame(Mat source0) {
        if (processing) {
            if(source0.width() == 0) return new Mat();
            invertThreshold  = blue;
            pipelineImg[0] = source0;// Step CV_cvtColor0:
            blueWorkingMat = source0;
//            cvCvtcolor(source0, cvCvtcolorCode, cvCvtcolorOutput);
            Imgproc.cvtColor(source0, cvCvtcolorOutput, Imgproc.COLOR_RGB2YCrCb);
            pipelineImg[1] = cvCvtcolorOutput;
            blueWorkingMat = cvCvtcolorOutput;

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
            Mat showContours = new Mat(source0.rows(), source0.cols(),source0.type());

            findContours(blueWorkingMat, true, findContoursOutput);

            pipelineImg[6] = showContours;
            displayMat = source0;
            Imgproc.drawContours(displayMat, findContoursOutput, -1, new Scalar (0,255,0));
            //telemetry.addData("Contour list size: ", findContoursOutput.size());

            ArrayList<Rect> filterOutput = (ArrayList<Rect>) filterContours(findContoursOutput);
            telemetry.addData("rect list size: ", filterOutput.size());
            double highest = 0;
            for (Rect a : filterOutput) {
                Point found = analyzeRect(source0, a);
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

            showContours.release();
//            telemetry.addData("loationTSE: ", locationTSE);
//            double a,b,c,d;
            telemetry.addData("answer: ", answer.toString());
            Imgproc.drawMarker(displayMat, new Point(answer.x + (double) answer.width / 2, answer.y + (double) answer.height / 2), new Scalar(0, 0, 255),0,0,10);
            Imgproc.line(displayMat,new Point(0,cutoffLine),new Point(source0.width()-1,cutoffLine),new Scalar(255,0,0),2);

            return displayMat;
        }

        telemetry.update();
        return source0;
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

    public int location (Point a) {
        int loc = 0;
        if (a.x>leftSep && a.x<rightSep) {
            loc = 1;

        } else if (a.x<leftSep) {
            loc = 0;

        } else if (a.x>rightSep) {
            loc = 2;

        }

        return loc;
    }
}
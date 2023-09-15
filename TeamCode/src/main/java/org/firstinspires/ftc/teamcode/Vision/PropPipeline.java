package org.firstinspires.ftc.teamcode.Vision;

import static org.firstinspires.ftc.teamcode.Vision.VisionUtil.cvCvtcolor;
import static org.firstinspires.ftc.teamcode.Vision.VisionUtil.cvDilate;
import static org.firstinspires.ftc.teamcode.Vision.VisionUtil.cvErode;
import static org.firstinspires.ftc.teamcode.Vision.VisionUtil.cvExtractchannel;
import static org.firstinspires.ftc.teamcode.Vision.VisionUtil.cvThreshold;
import static org.firstinspires.ftc.teamcode.Vision.VisionUtil.findContours;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
@Config
public class  PropPipeline extends PipelineWrapper {
    public static double location;
    public static boolean DuckDetected;
    public static boolean blue = true;
    public static double filterContoursMinArea = 0.0;
    public static double filterContoursMinPerimeter = 0.0;
    public static double filterContoursMinWidth = 0.0;
    public static double filterContoursMinHeight = 0.0;
    public static double filterContoursMaxHeight = 20.0;
    public static double filterContoursMaxWidth = 30.0;
    public static double filterContoursMinX = 0.0;
    public static double filterContoursMaxX = 9000.0;
    public static double filterContoursMinY = 80.0;
    public static double filterContoursMinYBlue = 100.0;
    public static double filterContoursMaxY = 9000.0;
    public static int leftSep = 109;
    public static int rightSep = 218;
    public static double cvThresholdThresh = 150.0;
    public static double cvThresholdThreshBlue = 120.0;
    public static int cutoffLine =120;

    public static double cvThresholdMaxval = 255.0;
    public static double cvDilateIterations = 1.0;
    public static double cvErodeIterations = 1;

    public static int targetLinePos = 170;
    public static int idealWidth = 32;

    public static double maxY = 320;
    public static boolean invertThreshold = false;
    public static double cvExtractchannelChannel = 2.0;
    public static double cvExtractchannelChannelRed = 1.0;
    public static int cvCvtcolorCode = Imgproc.COLOR_RGB2Lab;
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
    FtcDashboard dashboard;
    boolean[] covered = {true, true, true};
    public PropPipeline (boolean blue, FtcDashboard dashboard){
        PropPipeline.blue = blue;
        this.dashboard = dashboard;
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
    public Mat processFrame(Mat source0, long captureTimeNanos) {
        if (processing) {
            if(source0.width() == 0) return new Mat();
            invertThreshold  = blue;
            pipelineImg[0] = source0;// Step CV_cvtColor0:
            blueWorkingMat = source0;
            cvCvtcolor(source0, cvCvtcolorCode, cvCvtcolorOutput);
            pipelineImg[1] = cvCvtcolorOutput;
            blueWorkingMat = cvCvtcolorOutput;

            cvExtractchannel(blueWorkingMat, blue?cvExtractchannelChannel:cvExtractchannelChannelRed, cvExtractchannelOutput);
            pipelineImg[2] = cvExtractchannelOutput;
            blueWorkingMat = cvExtractchannelOutput;

            // Step CV_Threshold0:
            if (invertThreshold) {
                cvThreshold(blueWorkingMat, cvThresholdThreshBlue, cvThresholdMaxval, Imgproc.THRESH_BINARY_INV, cvThresholdOutput);
            } else {
                cvThreshold(blueWorkingMat, cvThresholdThresh, cvThresholdMaxval, Imgproc.THRESH_BINARY, cvThresholdOutput);
            }

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

            ArrayList<Rect> filterOutput = (ArrayList<Rect>) filterContours(findContoursOutput);
            for (Rect a : filterOutput) {
                Point found = analyzeRect(source0, a);
                if (found.x != -69) {//hohahehhehehe
                    covered[location(found)] = false;
                }
            }
            double minWidth = 999;
            for(Rect rect : filterOutput){
                Imgproc.rectangle(displayMat,rect,new Scalar (0,255,0),2);
                boundingRect = rect.width < minWidth ? rect : boundingRect;
                minWidth = Math.min(rect.width,minWidth);
            }
            pipelineImg[7] = displayMat;

            if(dashboardEnabled){
                Mat ret =  pipelineImg[dashboardImg];
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("Location",locationTSE);
                dashboard.sendTelemetryPacket(packet);
                if(ret.width() != 0) {
                    Bitmap displayBitmap = Bitmap.createBitmap(ret.width(), ret.height(), Bitmap.Config.RGB_565);
                    Utils.matToBitmap(ret, displayBitmap);
                    dashboard.sendImage(displayBitmap);
                }
            }

            //showFilteredContours.release();
            showContours.release();
            for (int u = 0; u<3; u++) {
                if (covered[u]) {
                    locationTSE = u;
                    break;
                }
            }
            covered[0] = true;
            covered[1] = true;
            covered[2] = true;

            Imgproc.line(displayMat,new Point(0,cutoffLine),new Point(source0.width()-1,cutoffLine),new Scalar(255,0,0),2);

            return displayMat;
        }


        return source0;
    }
    private List<Rect> filterContours(List<MatOfPoint> inputContours) {
        List<Rect> outputContours = new ArrayList<>();
        for (MatOfPoint contour : inputContours) {
            final Rect bb = Imgproc.boundingRect(contour);
            if(bb.x < filterContoursMinX || bb.x > filterContoursMaxX) continue;
            if(bb.y < (blue?filterContoursMinYBlue:filterContoursMinY) || bb.y > filterContoursMaxY) continue;
            if (bb.width < filterContoursMinWidth || bb.width > filterContoursMaxWidth) continue;
            if (bb.height < filterContoursMinHeight || bb.height > filterContoursMaxHeight) continue;
            final double area = Imgproc.contourArea(contour);
            if (area < filterContoursMinArea) continue;
            if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < filterContoursMinPerimeter) continue;
            outputContours.add(Imgproc.boundingRect(contour));
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

    @Override
    public Rect getResult() {
        return new Rect(locationTSE,locationTSE,locationTSE,locationTSE);
    }

    @Override
    public Mat[] getPipeline() {
        return pipelineImg;
    }
}
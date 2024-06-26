package org.firstinspires.ftc.teamcode.Vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class VisionUtil {
    private Mat workingMat,displayMat;
    List<MatOfPoint> findContoursOutput;
    /**
     * Converts an image from one color space to another.
     * @param src Image to convert.
     * @param code conversion code.
     * @param dst converted Image.
     */
    public static void cvCvtcolor(Mat src, int code, Mat dst) {
        Imgproc.cvtColor(src, dst, code);
    }
    public static void cvGaussianBlur(Mat src, Mat dst, int size){
        size = size%2==0?size+1:size;
        Imgproc.GaussianBlur(src,dst,new Size(size,size),0);
    }
    /**
     * Extracts given channel from an image.
     * @param src the image to extract.
     * @param channel zero indexed channel number to extract.
     * @param dst output image.
     */
    public static void cvExtractchannel(Mat src, double channel, Mat dst) {
        Core.extractChannel(src, dst, (int)channel);
    }

    /**
     * Apply a fixed-level threshold to each array element in an image.
     * @param src Image to threshold.
     * @param threshold threshold value.
     * @param maxVal Maximum value for THRES_BINARY and THRES_BINARY_INV
     * @param type Type of threshold to appy.
     * @param dst output Image.
     */
    public static void cvThreshold(Mat src, double threshold, double maxVal, int type,
                                   Mat dst) {
        Imgproc.threshold(src, dst, threshold, maxVal, type);
    }

    /**
     * Expands area of lower value in an image.
     * @param src the Image to erode.
     * @param kernel the kernel for erosion.
     * @param anchor the center of the kernel.
     * @param iterations the number of times to perform the erosion.
     * @param borderType pixel extrapolation method.
     * @param borderValue value to be used for a constant border.
     * @param dst Output Image.
     */
    public static void cvErode(Mat src, Mat kernel, Point anchor, double iterations,
                               int borderType, Scalar borderValue, Mat dst) {
        if (kernel == null) {
            kernel = new Mat();
        }
        if (anchor == null) {
            anchor = new Point(-1,-1);
        }
        if (borderValue == null) {
            borderValue = new Scalar(-1);
        }
        Imgproc.erode(src, dst, kernel, anchor, (int)iterations, borderType, borderValue);
    }

    /**
     * Expands area of higher value in an image.
     * @param src the Image to dilate.
     * @param kernel the kernel for dilation.
     * @param anchor the center of the kernel.
     * @param iterations the number of times to perform the dilation.
     * @param borderType pixel extrapolation method.
     * @param borderValue value to be used for a constant border.
     * @param dst Output Image.
     */
    public static void cvDilate(Mat src, Mat kernel, Point anchor, double iterations,
                                int borderType, Scalar borderValue, Mat dst) {
        if (kernel == null) {
            kernel = new Mat();
        }
        if (anchor == null) {
            anchor = new Point(-1,-1);
        }
        if (borderValue == null){
            borderValue = new Scalar(-1);
        }
        Imgproc.dilate(src, dst, kernel, anchor, (int)iterations, borderType, borderValue);
    }

    /**
     * Sets the values of pixels in a binary image to their distance to the nearest black pixel.
     * @param input The image on which to perform the Distance Transform
     */
    public static void findContours(Mat input, boolean externalOnly,
                                    List<MatOfPoint> contours) {
        Mat hierarchy = new Mat();
        contours.clear();
        int mode;
        if (externalOnly) {
            mode = Imgproc.RETR_EXTERNAL;
        }
        else {
            mode = Imgproc.RETR_LIST;
        }
        int method = Imgproc.CHAIN_APPROX_SIMPLE;
        Imgproc.findContours(input, contours, hierarchy, mode, method);

    }

    public Mat[] processFrame(Mat source0, VisionParameters v) {

        if(source0.width() == 0) return new Mat[0];
        Mat[] pipelineImg = new Mat[20];
        pipelineImg[0] = source0;// Step CV_cvtColor0:`
        workingMat = source0;
        cvCvtcolor(source0, v.cvCvtcolorCode, workingMat);
        pipelineImg[1] = workingMat;

        cvExtractchannel(workingMat, v.cvExtractchannelChannel, workingMat);
        pipelineImg[2] = workingMat;

        // Step CV_Threshold0:
        if (v.invertThreshold) {
            cvThreshold(workingMat, v.threshold, 255, Imgproc.THRESH_BINARY_INV, workingMat);
        } else {
            cvThreshold(workingMat, v.threshold, 255, Imgproc.THRESH_BINARY, workingMat);
        }

        pipelineImg[3] = workingMat;
        // Step CV_erode0:


        cvErode(workingMat,  new Mat(), new Point(-1, -1), v.cvErodeIterations, Core.BORDER_CONSTANT, new Scalar(-1), workingMat);

        pipelineImg[4] = workingMat;

        // Step CV_dilate0:


        cvDilate(workingMat, new Mat(), new Point(-1, -1), v.cvDilateIterations, Core.BORDER_CONSTANT, new Scalar(-1), workingMat);
        pipelineImg[5] = workingMat;


        // Step Find_Contours0:
        Mat showContours = new Mat(source0.rows(), source0.cols(),source0.type());

        findContours(workingMat, true, findContoursOutput);

        pipelineImg[6] = showContours;
        displayMat = source0;

        ArrayList<Rect> filterOutput = (ArrayList<Rect>) filterContours(findContoursOutput,v.minX,v.maxX,v.minY,v.maxY, v.minWidth,v.maxWidth,v.minHeight,v.maxHeight,v.minArea);

        for(Rect rect : filterOutput){
            Imgproc.rectangle(displayMat,rect,new Scalar (0,255,0),2);
        }

        pipelineImg[7] = displayMat;


        showContours.release();
        return pipelineImg;

    }



    private List<Rect> filterContours(List<MatOfPoint> inputContours,double minX,double maxX,double minY,double maxY, double minWidth,double maxWidth,double minHeight,double maxHeight,double minArea) {
        List<Rect> outputContours = new ArrayList<>();
        for (MatOfPoint contour : inputContours) {
            final Rect bb = Imgproc.boundingRect(contour);
            if(bb.x < minX || bb.x > maxX) continue;
            if(bb.y < minY || bb.y > maxY) continue;
            if (bb.width < minWidth || bb.width > maxWidth) continue;
            if (bb.height < minHeight || bb.height > maxHeight) continue;
            final double area = Imgproc.contourArea(contour);
            if (area < minArea) continue;
            outputContours.add(Imgproc.boundingRect(contour));
        }
        return outputContours;
    }

}
abstract class VisionParameters {
    public double minArea = 0.0;
    public double minPerimeter = 0.0;
    public double minWidth = 0.0;
    public double minHeight = 10.0;
    public double maxHeight = 1000.0;
    public double maxWidth = 1000.0;
    public double minX = 0.0;
    public double maxX = 9000.0;
    public double minY = 0.0;
    public double maxY = 9000.0;
    public double threshold = 140.0;
    public double cvDilateIterations = 1.0;
    public double cvErodeIterations = 3;
    public boolean invertThreshold = false;
    public double cvExtractchannelChannel = 1.0;
    public int cvCvtcolorCode = Imgproc.COLOR_RGB2Lab;
}
class shippingParameters extends VisionParameters{
    double minArea = 0.0;
    double minPerimeter = 0.0;
    double minWidth = 0.0;
    double minHeight = 10.0;
    double maxHeight = 1000.0;
    double maxWidth = 1000.0;
    double minX = 0.0;
    double maxX = 9000.0;
    double minY = 0.0;
    double maxY = 9000.0;
    double threshold = 140.0;
    double cvDilateIterations = 1.0;
    double cvErodeIterations = 3;
    boolean invertThreshold = false;
    double cvExtractchannelChannel = 1.0;
    int cvCvtcolorCode = Imgproc.COLOR_RGB2Lab;
}
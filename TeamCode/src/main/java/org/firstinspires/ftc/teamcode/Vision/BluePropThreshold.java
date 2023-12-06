package org.firstinspires.ftc.teamcode.Vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class BluePropThreshold implements VisionProcessor {
    Mat testMat = new Mat();
    Mat highMat = new Mat();
    Mat lowMat = new Mat();
    Mat finalMat = new Mat();
    double blueThreshold = 0.5;

    String outStr = "left"; //Set a default value in case vision does not work
    double avgLeft = 0;
    double avgRight = 0;

    static final Rect LEFT_RECTANGLE = new Rect(
            new Point(50, 150),
            new Point(150, 200)
    );

    static final Rect RIGHT_RECTANGLE = new Rect(
            new Point(440, 50),
            new Point(540, 125)
    );

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, testMat, Imgproc.COLOR_BGR2HSV);


        Scalar lowHSVRedLower = new Scalar(0, 100, 20);  //Beginning of Color Wheel
        Scalar lowHSVRedUpper = new Scalar(10, 255, 255);

        Scalar redHSVRedLower = new Scalar(160, 100, 20); //Wraps around Color Wheel
        Scalar highHSVRedUpper = new Scalar(180, 255, 255);

        Core.inRange(testMat, lowHSVRedLower, lowHSVRedUpper, lowMat);
        Core.inRange(testMat, redHSVRedLower, highHSVRedUpper, highMat);

        testMat.release();

        Core.bitwise_or(lowMat, highMat, finalMat);

        lowMat.release();
        highMat.release();

        double leftBox = Core.sumElems(finalMat.submat(LEFT_RECTANGLE)).val[0];
        double rightBox = Core.sumElems(finalMat.submat(RIGHT_RECTANGLE)).val[0];

        double averagedLeftBox = leftBox / LEFT_RECTANGLE.area() / 255;
        double averagedRightBox = rightBox / RIGHT_RECTANGLE.area() / 255; //Makes value [0,1]


        Imgproc.rectangle(finalMat,LEFT_RECTANGLE, new Scalar(255,255,255));
        Imgproc.rectangle(finalMat,RIGHT_RECTANGLE, new Scalar(255,255,255));


        if(averagedRightBox > blueThreshold){        //Must Tune Red Threshold
            outStr = "center";
        }else if(averagedLeftBox> blueThreshold){
            outStr = "left";
        }else{
            outStr = "right";
        }


        avgLeft = averagedLeftBox;
        avgRight = averagedRightBox;

        finalMat.copyTo(frame); /*This line should only be added in when you want to see your custom pipeline
                                  on the driver station stream, do not use this permanently in your code as
                                  you use the "frame" mat for all of your pipelines, such as April Tags*/
        return null;            //You do not return the original mat anymore, instead return null




    }


    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public String getPropPosition(){  //Returns postion of the prop in a String
        return outStr;
    }

    public Double getAvergageLeft(){  //Returns postion of the prop in a String
        return avgLeft;
    }

    public Double getAvergageRight(){  //Returns postion of the prop in a String
        return avgRight;
    }
}
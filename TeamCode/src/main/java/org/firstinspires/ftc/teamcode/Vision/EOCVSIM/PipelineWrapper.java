package org.firstinspires.ftc.teamcode.Vision.EOCVSIM;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.openftc.easyopencv.TimestampedOpenCvPipeline;

//standardized interface for VisionPipelines
public abstract class PipelineWrapper extends TimestampedOpenCvPipeline {
    public static double camMidPos = 160;
    public abstract Rect getResult();
    public abstract Mat[] getPipeline();
}
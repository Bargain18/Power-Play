package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ConeDetectionPipeline extends OpenCvPipeline {
//    public Scalar RED1 = new Scalar(0, 0, 0);
//    public Scalar RED2 = new Scalar(255, 255, 255);
    private static final Scalar[] RED1 = new Scalar[]{new Scalar(0, 80, 100), new Scalar(10, 255, 255)};
    private static final Scalar[] RED2 = new Scalar[]{new Scalar(160, 50, 50), new Scalar(180, 255, 255)};

    private Mat cvt = new Mat();
    private Mat binary = new Mat();
    private Mat masked = new Mat();

    private final Mat mask = new Mat();
    private final Mat mask2 = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, cvt, Imgproc.COLOR_RGB2HSV);
        //Core.inRange(cvt, RED1, RED2, binary);
        Core.inRange(cvt, RED1[0], RED1[1], mask);
        Core.inRange(cvt, RED2[0], RED2[1], mask2);
        Core.bitwise_or(mask, mask2, mask);
        masked.release();
        Core.bitwise_and(input, input, masked, mask);
        //Imgproc.cont
        return masked;
    }
}

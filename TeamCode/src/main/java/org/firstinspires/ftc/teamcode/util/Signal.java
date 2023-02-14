package org.firstinspires.ftc.teamcode.util;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class Signal {
    public Scalar lower;
    public Scalar upper;

    public Mat image;

    /*
        thresholds: [lowerBound, upperBound]
     */
    public Signal(int[] hueThreshold,
                  int[] saturationThreshold,
                  int[] valueThreshold) {
        lower = new Scalar(
                hueThreshold[0],
                saturationThreshold[0],
                valueThreshold[0]);

        upper = new Scalar(
                hueThreshold[1],
                saturationThreshold[1],
                valueThreshold[1]);
    }

    public boolean detect(Mat input) {
        Mat mask = new Mat();

        Core.inRange(input, lower, upper, mask);


    }
}

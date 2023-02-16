package org.firstinspires.ftc.teamcode.util;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Signal {
    public Scalar lower;
    public Scalar upper;

    public double minArea;
    public double maxArea;

    public Mat image;

    /*
        thresholds: [lowerBound, upperBound]
     */
    public Signal(Scalar lower,
                  Scalar upper,
                  int minArea,
                  int maxArea) {
        this.lower = lower;
        this.upper = upper;

        this.minArea = minArea;
        this.maxArea = maxArea;
    }

    public double getArea(Mat input) {
        Mat mask = new Mat();
        Mat temp = new Mat();
        double area = 0;

        Core.inRange(input, lower, upper, mask);
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask,
                contours,
                temp,
                Imgproc.RETR_TREE,
                Imgproc.CHAIN_APPROX_SIMPLE);

        Imgproc.drawContours(input, contours, -1, new Scalar(250,0,250),20);

        for (MatOfPoint contour : contours) {
            area += Imgproc.contourArea(contour);
        }

        System.out.println(area);

        mask.release();
        temp.release();

        return area;
    }

    public boolean detect(double area) {
        return area > minArea && area < maxArea;
    }
}

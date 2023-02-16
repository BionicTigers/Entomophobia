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

    //Create a new signal with the upper and lower bounds and a minimum area with no maximum
    //The Scalar for lower and upper goes (H, S, V)
    public Signal(Scalar lower, Scalar upper, int minArea) {
        //Set the lower and upper bounds for HSV Values
        this.lower = lower;
        this.upper = upper;

        //Set the minimum area
        //Sets the maximum area to infinity
        this.minArea = minArea;
        this.maxArea = Integer.MAX_VALUE;
    }

    //Create a new signal with the upper and lower bounds and a minimum and maximum area
    //The Scalar for lower and upper goes (H, S, V)
    public Signal(Scalar lower, Scalar upper, int minArea, int maxArea) {
        //Set the lower and upper bounds for HSV Values
        this.lower = lower;
        this.upper = upper;

        //Set the minimum area and maximum area
        this.minArea = minArea;
        this.maxArea = maxArea;
    }

    public double getArea(Mat input) {
        //Create starting variables
        Mat mask = new Mat();
        Mat temp = new Mat();
        double area = 0;

        //Create a color mask
        Core.inRange(input, lower, upper, mask);

        //Create a new ArrayList of contours and then find said contours to map
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask,
                contours,
                temp,
                Imgproc.RETR_TREE,
                Imgproc.CHAIN_APPROX_SIMPLE);

        //Draw contours on top of the image for easier debugging
        //This is a work in progress.
        Imgproc.drawContours(input, contours, -1, new Scalar(250,0,250),20);

        //Sum the area of every contour
        for (MatOfPoint contour : contours) {
            area += Imgproc.contourArea(contour);
        }

        //Free the mat's from memory to prevent a memory leak
        mask.release();
        temp.release();

        return area;
    }

    //Used to check if the area falls inside of maxArea
    public boolean detect(double area) {
        return area > minArea && area < maxArea;
    }
}

package org.firstinspires.ftc.teamcode.util;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
//If you need help call me: 513-808-0241
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Signal {
    public Scalar lower;
    public Scalar upper;

    public ArrayList<MatOfPoint> contours;

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

        //Create a kernel for morphological operations
        Mat kernel = new Mat(new Size(5, 5), CvType.CV_8UC1, new Scalar(255));

        //Create a color mask
        Core.inRange(input, lower, upper, mask);

        //Preform morphological operation to remove noise
        Imgproc.morphologyEx(mask, temp, Imgproc.MORPH_OPEN, kernel);

        //Release mask to be about to use at the output then reassign it
        mask.release();
        mask = new Mat();
        Imgproc.morphologyEx(temp, mask, Imgproc.MORPH_CLOSE, kernel);

        //Create a new ArrayList of contours and then find said contours to map
        contours = new ArrayList<>();
        Imgproc.findContours(mask,
                contours,
                temp,
                Imgproc.RETR_TREE,
                Imgproc.CHAIN_APPROX_SIMPLE);

        //Sum the area of every contour
        for (MatOfPoint contour : contours) {
            area += Imgproc.contourArea(contour);
        }

        //Free the mat's from memory to prevent a memory leak
        kernel.release();
        mask.release();
        temp.release();

        return area;
    }

    //Used to check if the area falls inside of maxArea
    public boolean detect(double area) {
        return area > minArea && area < maxArea;
    }
}
//Purple Signal
//
//
//
//Green Signal
//
//
//
//Orange Signal
package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Scalar;

@Config
public class VisionConstants {
    public static Scalar ORANGE_LOWER = new Scalar(6, 150, 130);
    public static Scalar ORANGE_UPPER = new Scalar(20, 235, 185);

    public static Scalar GREEN_LOWER = new Scalar(40, 100, 100);
    public static Scalar GREEN_UPPER = new Scalar(74, 210, 170);

    public static Scalar PURPLE_LOWER = new Scalar(130, 130, 21);
    public static Scalar PURPLE_UPPER = new Scalar(180, 200, 150);

    public static Signal ORANGE = new Signal(ORANGE_LOWER, ORANGE_UPPER, 1000);
    public static Signal GREEN = new Signal(GREEN_LOWER, GREEN_UPPER, 1000);
    public static Signal PURPLE = new Signal(PURPLE_LOWER, PURPLE_UPPER, 1000);

    public static int EXPOSURE = 20;
    public static int GAIN = 0;
    public static int WHITE_BALANCE = 0;
}

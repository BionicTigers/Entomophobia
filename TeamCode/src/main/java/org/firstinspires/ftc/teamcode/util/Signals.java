package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Scalar;

@Config
public class Signals {
    public static Signal ORANGE = new Signal(new Scalar(6, 165, 145), new Scalar(26, 250, 200), 1000);
    public static Signal GREEN = new Signal(new Scalar(40, 100, 100), new Scalar(74, 210, 170), 1000);
    public static Signal PURPLE = new Signal(new Scalar(130, 130, 21), new Scalar(180, 220, 160), 1000);
}

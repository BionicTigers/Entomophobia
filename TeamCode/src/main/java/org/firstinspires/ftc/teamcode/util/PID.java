package org.firstinspires.ftc.teamcode.util;

import java.util.concurrent.TimeUnit;

public class PID {
    //Proportion: scales output with error
    private double kP;
    //Integral: corrects error over time
    private double kI;
    //Derivative: corrects based on future trend from current rate of change
    private double kD;

    //Integral: Sum of errors
    private double errorSum;

    private long previousTime;

    public PID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        previousTime = System.currentTimeMillis();
    }

    public double calculate(double setPoint, double processValue) {
        //Calculate the error
        double error = setPoint-processValue;

        //Calculate how long it has been since the last call
        long currentTime = System.currentTimeMillis();
        long deltaTime = currentTime - previousTime;



        previousTime = currentTime;

        return setPoint;
    }

    public double calculate(double setPoint, double processValue, double min, double max) {
        return setPoint;
    }
}

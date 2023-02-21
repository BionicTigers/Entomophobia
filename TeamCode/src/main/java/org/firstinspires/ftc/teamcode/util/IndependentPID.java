package org.firstinspires.ftc.teamcode.util;

public class IndependentPID {
    //Proportion: scales output with error
    private double kP;
    //Integral: corrects error over time
    private double kI;
    //Derivative: corrects based on future trend from current rate of change
    private double kD;

    private double min;
    private double max;

    //Time between samples in MS
    //Allows us to simplify the math and give a more accurate reading
    private final int sampleRate = 100;

    //Store the output to return if we sample faster than our sample rate.
    private double output;

    //Sum of errors over time
    private double errorSum;

    private long previousTime;
    private double previousProcessValue;

    private boolean doReset;

    public IndependentPID(double kP, double kI, double kD, double min, double max) {
        this.kP = kP;
        this.kI = kI * ((float) sampleRate/1000);
        this.kD = kD / ((float) sampleRate/1000);

        this.min = min;
        this.max = max;

        previousTime = 0;
    }

    //SetPoint is the goal, processValue is the current value/input
    public double calculate(double setPoint, double processValue) {
        //Reset the stored variables to default
        if (doReset) {
            errorSum = 0;
            previousProcessValue = processValue;
            previousTime = 0;

            doReset = false;
        }

        //Calculate how long it has been since the last call
        long currentTime = System.currentTimeMillis();
        double deltaTime = (double) currentTime - previousTime;

        //Calculate only if the time between samples is less than the change in time
        if (deltaTime >= sampleRate) {
            //Calculate the error
            double error = setPoint - processValue;
            errorSum = Math.max(min, Math.min(max, errorSum + error * kI)); //Allows for real-time tuning

            //We calculate the change in processValue rather than in error
            //This removes the spike in output when changing setPoint
            double deltaProcessValue = processValue - previousProcessValue;

            //Calculate the output
            output = Math.max(min, Math.min(max, kP * error + errorSum - kD * deltaProcessValue));

            previousProcessValue = processValue;
            previousTime = currentTime;
        }

        return output;
    }

    public void tune(double kP, double kI, double kD, double min, double max) {
        this.kP = kP;
        this.kI = kI * ((float) sampleRate/1000);
        this.kD = kD / ((float) sampleRate/1000);

        this.min = min;
        this.max = max;
    }

    public void reset() {
        doReset = true;
    }
}

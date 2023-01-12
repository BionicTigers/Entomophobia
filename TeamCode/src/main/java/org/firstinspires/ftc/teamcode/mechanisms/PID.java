
package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.util.ElapsedTime;

import static java.util.concurrent.TimeUnit.SECONDS;

public class PID {
    private double kP;
    private double tI;
    private double tD;
    private double kF;

    private double error;
    private double lastError = 0;
    private double lastLastError = 0;

    public double deltaE;

    private double cv;
    private double lastCv=0;

    public ElapsedTime systemTime;
    public double startTime;
    private double lastTime;

    private double minimum;
    private double maximum;

    public PID (double p, double i, double d, double f, ElapsedTime sisTime, double min, double max) {
        kP = p;
        tI = i;
        tD = d;
        kF = f;
        systemTime = sisTime;
        startTime = systemTime.now(SECONDS);
        lastTime = startTime;

        minimum = min;
        maximum = max;
    }

    public PID (PID other) {
        kP = other.kP;
        tI = other.tI;
        tD = other.tD;
        kF = other.kF;
        systemTime = other.systemTime;
        startTime = systemTime.now(SECONDS);
        lastTime = startTime;

        minimum = other.minimum;
        maximum = other.maximum;
    }

    public void pidStart() {
        startTime = systemTime.now(SECONDS);
        cv = 0;
        lastCv = 0;
        error = 0;
        lastError = 0;
        lastLastError = 0;
    }

    public double findOutput(double e) {

        error = e;

        deltaE = error - lastError;

        double currentTime = systemTime.now(SECONDS);

        double deltaT = currentTime - lastTime;

        double integral = 0;
        if (tI != 0) {
            integral = (error * deltaT)/(60 * tI);
        }
        double derivative = 0;

        if(deltaT != 0) {
            derivative = 60*tD*(error - 2*lastError + lastLastError)/deltaT;
        }

        cv = lastCv + kP*(deltaE + integral + derivative);

        if (cv < minimum) {
            cv = minimum;
            //return minimum;
        } else if (cv > maximum) {
            cv = maximum;
            //return maximum;
        }

        lastTime = systemTime.now(SECONDS);

        lastCv = cv;

        lastLastError = lastError;
        lastError = error;

        return cv;
    }

    public void setkP(double p) {
        kP = p;
    }

    public void settI(double i) {
        tI = i;
    }

    public void settD(double d) {
        tD = d;
    }

    public double getCv() {
        return cv;
    }

    public double getError() {
        return error;
    }

    public double getLastError() {
        return lastError;
    }

    public double getLastLastError() {
        return lastLastError;
    }
}

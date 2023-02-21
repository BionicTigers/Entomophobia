package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    double Kp;
    double Ti; //In Minutes
    double Td; //In Minutes

    double min;
    double max;

    double sampleTime = 100; //ms

    ElapsedTime previousCall;

    double E1; //1st Previous Error
    double E2; //2nd Previous Error

    double CV; //Current Value

    public PID(double Kp, double Ti, double Td, double min, double max) {
        //Set Terms
        this.Kp = Kp;
        this.Ti = Ti;
        this.Td = Td;

        //Set Limits
        this.min = min;
        this.max = max;

        //Set Defaults
        E1 = 0;
        E2 = 0;

        CV = 0;

        previousCall = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    public double calculate(double SP, double PV) {
        //Calculate the time between calls
        double dt = previousCall.milliseconds() / 1000;

        if (dt > sampleTime / 1000) {
            //Turn Error into a percentage
            double E = (SP - PV) / (max - min);

            double P = E;

            double I;
            if (Ti == 0) //Cannot divide by 0 so we set it to 0
                I = 0;
            else
                I = Kp * (1 / (60 * Ti) * E * dt);

            double D = 60 * Td * ((E2 * E1 + E) / dt);

            //Clamp CV to the min and the max
            CV = Math.max(min, Math.min(max, CV + Kp * (P + I + D)));

            //Set Futures
            E2 = E1;
            E1 = E;

            //Reset Elapsed Time
            previousCall.reset();
        }

        return CV;
    }
}

package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PID {
    double Kp;
    double Ti; //In Minutes
    double Td; //In Minutes

    double minPV;
    double maxPV;

    double minCV;
    double maxCV;

    double sampleTime = 20; //ms

    ElapsedTime previousCall;

    Telemetry telem = FtcDashboard.getInstance().getTelemetry();

    double PV1; //1st Previous Error
    double PV2; //2nd Previous Error

    double E1;

    double CV; //Current Value

    public PID(double Kp, double Ti, double Td, double minPV, double maxPV, double minCV, double maxCV) {
        //Set Terms
        this.Kp = Kp;
        this.Ti = Ti;
        this.Td = Td;

        //Set Limits
        this.minPV = minPV;
        this.maxPV = maxPV;

        this.minCV = minCV;
        this.maxCV = maxCV;

        //Set Defaults
        PV1 = 0;
        PV2 = 0;

        E1 = 0;

        CV = 0;

        previousCall = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    public double calculate(double SP, double PV) {
        //Calculate the time between calls
        double dt = previousCall.milliseconds() / 1000;

        if (dt > sampleTime / 1000) {
            //Turn to percentage
            double scaledPV = (maxPV - minPV);
            double E = (SP - PV) / (maxPV - minPV);

            double P = E - E1;

            double I;
            if (Ti == 0) //Cannot divide by 0 so we set it to 0
                I = 0;
            else
                I = (E * dt) / (60 * Ti);

            double D = 60 * Td * ((scaledPV - 2 * PV1 + PV2) / dt);

            telem.addData("E", E * (maxPV - minPV));
            telem.addData("P", Kp * P);
            telem.addData("I", Kp * I);
            telem.addData("D", Kp * D);

            //Clamp CV to the min and the max
            CV = Math.max(minCV, Math.min(maxCV, CV + Kp * (P + I + D)));

            //Set Futures
            E1 = E;

            PV2 = PV1;
            PV1 = scaledPV;

            //Reset Elapsed Time
            previousCall.reset();
        }

        return CV;
    }
}

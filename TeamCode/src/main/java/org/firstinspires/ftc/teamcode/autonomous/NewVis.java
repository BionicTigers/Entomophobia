package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.OpenCv;
import org.firstinspires.ftc.teamcode.util.Signal;
import org.opencv.core.Scalar;

import java.util.HashMap;
import java.util.Map;

@Autonomous(name = "newVision")
public class NewVis extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard.getInstance().getTelemetry().addData("", hardwareMap.get(WebcamName.class, "Webcam 1"));
        FtcDashboard.getInstance().getTelemetry().update();

        HashMap<String, Signal> signals = new HashMap<>();
        signals.put("Orange", new Signal(new Scalar(0, 0, 0), new Scalar(255, 50, 50), 0, Integer.MAX_VALUE));

        OpenCv detector = new OpenCv(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                signals
        );
        while (opModeInInit()) {
            telemetry.addData("Reading", detector.getDetection());
            continue;
        }
    }
}

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
        signals.put("Orange", new Signal(new Scalar(6, 103, 147), new Scalar(89, 182, 255), 1000, Integer.MAX_VALUE));
        signals.put("Purple", new Signal(new Scalar(129, 53, 73), new Scalar(180, 255, 255), 1000, Integer.MAX_VALUE));
        signals.put("Green", new Signal(new Scalar(39, 18, 108), new Scalar(91, 180, 227), 1000, Integer.MAX_VALUE));

        OpenCv detector = new OpenCv(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                signals,
                hardwareMap.appContext.getResources().getIdentifier(
                        "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName())
        );
        while (opModeInInit()) {
            telemetry.addData("Reading", detector.getDetection());
            telemetry.update();
            continue;
        }
    }
}

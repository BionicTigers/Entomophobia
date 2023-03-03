package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.OpenCv;
import org.firstinspires.ftc.teamcode.util.Signal;
import org.firstinspires.ftc.teamcode.util.VisionConstants;
import org.opencv.core.Rect;

import java.util.HashMap;

@Autonomous(name = "newVision")
public class NewVis extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard.getInstance().getTelemetry().addData("", hardwareMap.get(WebcamName.class, "Webcam 1"));
        FtcDashboard.getInstance().getTelemetry().update();

        HashMap<String, Signal> signals = new HashMap<>();
        signals.put("Orange", VisionConstants.ORANGE);
        signals.put("Purple", VisionConstants.PURPLE);
        signals.put("Green", VisionConstants.GREEN);

        OpenCv detector = new OpenCv(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                signals,
                hardwareMap.appContext.getResources().getIdentifier(
                        "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName())
        );

        detector.setCrop(new Rect(200, 200, 400, 500));
        while (opModeInInit()) {
            telemetry.addData("Reading", detector.getDetection());
            telemetry.update();
        }
    }
}

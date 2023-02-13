package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.OpenCv;

@Autonomous(name = "newVision")
public class NewVis extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard.getInstance().getTelemetry().addData("", hardwareMap.get(WebcamName.class, "Webcam 1"));
        FtcDashboard.getInstance().getTelemetry().update();
        OpenCv detector = new OpenCv(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                null
        );
        while (opModeInInit()) {
            continue;
        }
    }
}

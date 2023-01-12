package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Vision;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@Autonomous(name="VisAuto", group="autonomous")
public class VisAuto extends LinearOpMode {
    //    public Deposit deposit;
    private Vision vision;

    public void runOpMode() {
        vision = new Vision();
        waitForStart();
        OpenCvCamera webcam;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.setPipeline(new Vision());
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        vision = new Vision(webcam);
        waitForStart();
        while (true) {
            telemetry.addData("Area", vision.getArea());
            telemetry.update();
        }
    }
}
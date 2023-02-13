package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.HashMap;
import java.util.Map;

class Pipeline extends OpenCvPipeline {
    public HashMap<String, Signal> signals;

    private String detection = "None";

    public Pipeline(HashMap<String, Signal> signals) {
        this.signals = signals;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat hsvMat = new Mat();
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        for (Map.Entry<String, Signal> entry : signals.entrySet()) {
            String name = entry.getKey();
            Signal signal = entry.getValue();

            signal.setMat(new Mat());
        }

        hsvMat.release();
        return input;
    }

    public String getDetection() {
        return detection;
    }
}

public class OpenCv {
    OpenCvCamera camera;

    HashMap<String, Signal> signals;

    public OpenCv(WebcamName webcamName, HashMap<String, Signal> signals, int monitorId) {
        this.camera = OpenCvCameraFactory.getInstance()
                .createWebcam(webcamName, monitorId);
        this.signals = signals;

        startCameraStream();
    }

    public OpenCv(WebcamName webcamName, HashMap<String, Signal> signals) {
        this.camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        this.signals = signals;

        startCameraStream();
    }

    private void startCameraStream() {
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                System.out.println("Camera Opened");
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                camera.setPipeline(new Pipeline(signals));
                FtcDashboard.getInstance().startCameraStream(camera, 0);
                System.out.println("Camera Started");
            }

            @Override
            public void onError(int errorCode) {
                throw new RuntimeException(String.format("Camera Initialization Failed: %d", errorCode));
            }
        });
    }
}

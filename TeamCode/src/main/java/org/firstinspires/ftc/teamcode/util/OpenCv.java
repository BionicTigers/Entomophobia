package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
//If you need help call me: 513-808-0241
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.TimeUnit;

//This is the pipeline we use for Vision
//It is ran on every new frame
class Pipeline extends OpenCvPipeline {
    public HashMap<String, Signal> signals;

    private String detection = "None";

    public Pipeline(HashMap<String, Signal> signals) {
        this.signals = signals;
    }

    //This is essentially the Main method for the pipeline
    //On every new frame from the camera, this is ran
    @Override
    public Mat processFrame(Mat input) {
        Mat hsvMat = new Mat();
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        //Convert the input image into HSV
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        //Highest Area found during every signal process
        double high = 0;

        for (Map.Entry<String, Signal> entry : signals.entrySet()) {
            String name = entry.getKey();
            Signal signal = entry.getValue();

            double area = signal.getArea(hsvMat);

            System.out.println(name + ": " + area);

            contours.addAll(signal.contours);

            //Check if the detection is in the minimum radius
            //Also check if its higher than the previous highest area
            if (signal.detect(area) && high < area) {
                high = area;
                detection = name;
            }
        }

        //Draw contours on top of the image for easier debugging
        //This is a work in progress.
        Imgproc.drawContours(input, contours, -1, new Scalar(250,0,250),2);

        //Release so we don't get any memory leaks
        hsvMat.release();
        return input;
    }

    public String getDetection() {
        return detection;
    }
}

public class OpenCv {
    private OpenCvWebcam camera;
    private HashMap<String, Signal> signals;
    private Pipeline pipeline;

    //Create a new OpenCV Wrapper WITH a live monitor view
    //Useful for debugging but slows down cpu cycles
    public OpenCv(WebcamName webcamName, HashMap<String, Signal> signals, int monitorId) {
        this.camera = OpenCvCameraFactory.getInstance()
                .createWebcam(webcamName, monitorId);

        pipeline = new Pipeline(signals);

        startCameraStream();
    }

    //Create a new OpenCV Wrapper WITHOUT a live monitor view
    public OpenCv(WebcamName webcamName, HashMap<String, Signal> signals) {
        this.camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        this.signals = signals;

        pipeline = new Pipeline(signals);

        startCameraStream();
    }

    //Return the current detection from the pipeline, NONE if there isn't a detection
    public String getDetection() {
        return pipeline.getDetection();
    }

    private void startCameraStream() {
        //Start the camera asynchronously to prevent yielding
        //This creates a new thread but it won't cause any issues with hardware ownership
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.getExposureControl().setMode(ExposureControl.Mode.Manual);
                camera.getExposureControl().setExposure(50, TimeUnit.MILLISECONDS);
                //Start streaming at 1280x720, in the upright orientation
                //Start the detection pipeline
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                camera.setPipeline(pipeline);

                //Used to give realtime camera feed to FTC Dashboard
                FtcDashboard.getInstance().startCameraStream(camera, 0);
            }

            @Override
            public void onError(int errorCode) {
                throw new RuntimeException(String.format("Camera Initialization Failed: %d", errorCode));
            }
        });
    }
}

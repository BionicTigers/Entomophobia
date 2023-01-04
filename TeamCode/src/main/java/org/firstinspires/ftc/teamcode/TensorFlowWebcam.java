package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**+
 * <h1>TensorFlow Object Detection</h1>
 * In FTC, TensorFlow is used to detect what is inside an image, (The PowerPlay season it would be a signal sleeve.)<br><br>
 * In it's essence, TensorFlow is not very good at differentiating geometric shapes,
 * so it's better to use patterns, or images that are different from each other like a Apple and Banana.<br><br>
 *
 * <h3>Implementing The Model</h3>
 * Using the model you trained on the <a href="https://ftc-ml.firstinspires.org/">FTC-ML Pipeline</a>
 * (If you haven't trained one yet reference this <a href="https://ftc-docs.firstinspires.org/ftc_ml/managing_tool/overview/overview.html">guide</a>)<br><br>
 * Insert it into the FtcRobotController > assets folder.<br>
 * Put the name of the file into
 */
@TeleOp(name="TensorFlowWebcam")
public class TensorFlowWebcam extends LinearOpMode {
    //Path for the file you made
    private static final String TFOD_MODEL_ASSET = "Let_There_Be_Light.tflite";

    private static final String[] LABELS = {
            "Bear",
            "Chair",
            "Zerba"
    };

    private static final String VUFORIA_KEY = "ATwrk3v/////AAABmaeTGhT4Ek/vlsivypnpHgozTllYm5abCieT9lemzCyirZ+6wd4GT5iiP3MQe34vBvCSqW3f7hjtIRAXO3jCr1/Tbw3IMT3T6QnefgPHOt/UbwmxexRX67I1M4PP/EitLzL5uZhrpnMPgOLcThIXA17F5HQDclLglJ/C8ZVRkVuer3L3HGjl/1jXTx6CisGaNARfEq3c4GMMEdKnFpY+v7MCwycT6Z4ihX3tgFM4/2gYIpID0Fe8teqcOv0CjWwX2kDaPzk6i0HCetvt+WEpRA17UeZ7nQqYglZSG4ZFkDfZhpZfEzJL7XbVvTG9vC+ZdbRNU+bgGKz56WsDWI0bh6BMUxXKm+F8ClfqXPsq2Vrk";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;


    @Override
    public void runOpMode() throws InterruptedException {
        initVuforia();
        initTfod();

        /**
         * Activate Object Detection while waiting for start.
         */
        if (tfod != null) {
            tfod.activate();

            /*
            Tensorflow scales images to a lower resolution to process faster
            At longer distances it can result is less accuracy
            You can increase the magnification value (parameter 1/v)
            Make sure to use the right aspect ratio so you don't have any distortion (parameter 2/v1)
             */
            tfod.setZoom(1.25, 16.0/9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                            double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                            double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                            double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                            telemetry.addData(""," ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                            telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
                        }
                        telemetry.update();
                    }
                }
            }
        }

    }

    /**
     * Initialize Vuforia's Localization Engine
     */
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}

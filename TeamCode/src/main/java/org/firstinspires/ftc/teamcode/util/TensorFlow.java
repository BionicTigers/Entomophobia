package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
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
public class TensorFlow {
    //Path for the file you made
    private static final String TFOD_MODEL_ASSET = "tflitemodels/wookiewookie.tflite";

    private static final String[] LABELS = {
            "Apple",
            "Orange",
            "Lime"
    };

    private static final String VUFORIA_KEY = "ATwrk3v/////AAABmaeTGhT4Ek/vlsivypnpHgozTllYm5abCieT9lemzCyirZ+6wd4GT5iiP3MQe34vBvCSqW3f7hjtIRAXO3jCr1/Tbw3IMT3T6QnefgPHOt/UbwmxexRX67I1M4PP/EitLzL5uZhrpnMPgOLcThIXA17F5HQDclLglJ/C8ZVRkVuer3L3HGjl/1jXTx6CisGaNARfEq3c4GMMEdKnFpY+v7MCwycT6Z4ihX3tgFM4/2gYIpID0Fe8teqcOv0CjWwX2kDaPzk6i0HCetvt+WEpRA17UeZ7nQqYglZSG4ZFkDfZhpZfEzJL7XbVvTG9vC+ZdbRNU+bgGKz56WsDWI0bh6BMUxXKm+F8ClfqXPsq2Vrk";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private WebcamName webcam;
    private int tfodMonitorViewId;

    public TensorFlow(WebcamName webcam, int tfodMonitorViewId) {
        this.webcam = webcam;
        this.tfodMonitorViewId = tfodMonitorViewId;

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.25, 16.0/9.0);
        }
    }

    public List<Recognition> getDetected() {
        return tfod.getUpdatedRecognitions();
    }

    /**
     * Initialize Vuforia's Localization Engine
     */
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcam;
        //hardwareMap.get(WebcamName.class, "Webcam 1")

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.70f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}

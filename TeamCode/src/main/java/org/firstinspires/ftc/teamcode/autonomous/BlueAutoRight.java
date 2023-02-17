package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.OpenCv;
import org.firstinspires.ftc.teamcode.util.Signal;
import org.opencv.core.Scalar;
import java.util.HashMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.teleop.Robot;
import org.firstinspires.ftc.teamcode.util.Location;
import org.firstinspires.ftc.teamcode.util.Signal;
import org.firstinspires.ftc.teamcode.util.TensorFlow;
import org.opencv.core.Scalar;

import java.util.HashMap;
import java.util.List;

@Autonomous (name="Blue Auto Right", group="autonomous")
public class BlueAutoRight extends LinearOpMode {

    public int[] motorNumbers = {0, 1, 2, 3}; //creates motor numbers array

    public Robot robot;
    public Drivetrain drivetrain;
    public Claw claw;
    public Lift lift;
    public OpenCv detector;
    HashMap<String, Signal> signals = new HashMap<>();


    public boolean a = false;


    public Location reset = new Location(-15,-25,0);
    public Location middleZone = new Location(0, 650, 0);
    public Location leftZone = new Location(-480, 675, 0);
    public Location rightZone = new Location(480, 675,0);

    public Location origin = new Location(0, 0, 0);

    public Location terminal = new Location(300, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        drivetrain = new Drivetrain(robot, motorNumbers, telemetry, hardwareMap.get(Servo.class, "LeftOdo"), hardwareMap.get(Servo.class, "BackOdo"), hardwareMap.get(Servo.class, "RightOdo"));
        detector = new OpenCv(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                signals,
                hardwareMap.appContext.getResources().getIdentifier(
                        "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName())
        );
        claw = new Claw(hardwareMap.get(Servo.class, "claw"));
        signals.put("Orange", new Signal(new Scalar(6, 103, 147), new Scalar(89, 182, 255), 1000, Integer.MAX_VALUE));
        signals.put("Purple", new Signal(new Scalar(129, 80, 73), new Scalar(163, 165, 255), 1000, Integer.MAX_VALUE));
        signals.put("Green", new Signal(new Scalar(39, 34, 108), new Scalar(83, 180, 219), 1000, Integer.MAX_VALUE));

//        lift = new Lift(hardwareMap.get(DcMotorEx.class, "liftT"),
//                hardwareMap.get(DcMotorEx.class, "liftM"),
//                hardwareMap.get(DcMotorEx.class, "liftB"),
//                hardwareMap.get(DigitalChannel.class, "Lift"), telemetry);

        drivetrain.odoDown();
        robot.odometry.reset();
        claw.open();




        waitForStart();
        telemetry.update();

        sleep(3000);

        drivetrain.moveToPositionMod(terminal, 5, 5, 0, 0.2, 2000);
        sleep(250);
        drivetrain.moveToPositionMod(origin, 5, 5, 0, 0.3, 2000);
        sleep(250);
        drivetrain.moveToPositionMod(middleZone, 5, 5, 0, 0.3, 2000);
        sleep(250);

            switch (detector.getDetection()) {
                case "Orange":
                    drivetrain.moveToPositionMod(leftZone, 5, 5,1, .3, 2000);
                    break;
                case "Purple":
                    break;
                case "Green":
                    drivetrain.moveToPositionMod(rightZone, 5, 5, 1, .3, 2000);
                    break;
                default:
                    break;
            }

        drivetrain.odoUp();
        sleep(5000);
        //These lines are for testing
        //drivetrain.moveToPositionMod(middleZone, 5, 5, 1, 0.3, 2000);
        //drivetrain.moveToPositionMod(rightZone, 5, 5, 1, 0.3, 4000);




//I have not tested the switch statement, zones work but the switch statement might not, it won't work 100% of the time bc of no rotation but a good 97%
//        if (detection != null && detection.size() > 0) {
//            switch (detection.get(0).getLabel()) {
//                case "Apple":
//                    drivetrain.moveToPositionMod(leftZone, 5, 5,1, .3, 3000);
//                    break;
//                case "Lime":
//                    break;
//                case "Orange":
//                    drivetrain.moveToPositionMod(rightZone, 5, 5, 1, .3, 4000);
//                    break;
//                default:
//                    break;
//            }
//        }
        drivetrain.odoUp();
        sleep(1000);
    }
}
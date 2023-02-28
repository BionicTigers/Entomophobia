package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.teleop.Robot;
import org.firstinspires.ftc.teamcode.util.Location;
import org.firstinspires.ftc.teamcode.util.OpenCv;
import org.firstinspires.ftc.teamcode.util.Signal;
import org.firstinspires.ftc.teamcode.util.Signals;

import java.util.HashMap;

@Autonomous (name="Red Auto Left", group="autonomous")
public class RedAutoLeft extends LinearOpMode {

    public int[] motorNumbers = {0, 1, 2, 3}; //creates motor numbers array

    public Robot robot;
    public Drivetrain drivetrain;
    public Claw claw;
    public Lift lift;

    public OpenCv detector;
    public HashMap<String, Signal> signals = new HashMap<>();

    public Location reset = new Location(-15, -25, 0);
    public Location middleZone = new Location(0, 625, 0);
    public Location leftZone = new Location(-585, 675, 0);
    public Location rightZone = new Location(600, 650, 0);

    public Location origin = new Location(0, 0, 0);

    public Location terminal = new Location(-300, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        drivetrain = new Drivetrain(robot, motorNumbers, telemetry, hardwareMap.get(Servo.class, "LeftOdo"), hardwareMap.get(Servo.class, "BackOdo"), hardwareMap.get(Servo.class, "RightOdo"));
        detector = new OpenCv(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                signals,
                hardwareMap.appContext.getResources().getIdentifier(
                        "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
        claw = new Claw(hardwareMap.get(Servo.class, "claw"), hardwareMap.get(DistanceSensor.class, "distance"), hardwareMap.get(RevBlinkinLedDriver.class, "blinkin"));
        //signals.put("Orange", new Signal(new Scalar(6, 103, 147), new Scalar(89, 182, 255), 1000));
        signals.put("Orange", Signals.ORANGE);
        signals.put("Purple", Signals.PURPLE);
        signals.put("Green", Signals.GREEN);


        drivetrain.odoDown();
        robot.odometry.reset();
        claw.open();

        while (opModeInInit()) {
            telemetry.addData("Reading", detector.getDetection());
            telemetry.update();
        }

        waitForStart();
        String detection = detector.getDetection();

        drivetrain.moveToPositionMod(terminal, 5, 5, 0, 0.2, 2000);
        sleep(250);
        drivetrain.moveToPositionMod(origin, 5, 5, 0, 0.3, 2000);
        sleep(250);
        drivetrain.moveToPositionMod(middleZone, 5, 5, 0, 0.3, 2000);
        sleep(250);

        switch (detection) {
            case "Orange":
                drivetrain.moveToPositionMod(leftZone, 5, 5, 1, .3, 2000);
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
    }
}
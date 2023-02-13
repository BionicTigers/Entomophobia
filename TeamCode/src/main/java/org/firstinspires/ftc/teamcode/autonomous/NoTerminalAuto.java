package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;
import org.firstinspires.ftc.teamcode.mechanisms.NOdoDrivetrain;
import org.firstinspires.ftc.teamcode.teleop.NOdoRobot;
import org.firstinspires.ftc.teamcode.util.NOdoLocation;
import org.firstinspires.ftc.teamcode.util.TensorFlow;

import java.util.List;

@Autonomous (name="Non-Terminal Auto", group="autonomous")
public class NoTerminalAuto extends LinearOpMode {

    public int[] motorNumbers = {0, 1, 2, 3}; //creates motor numbers array

    public NOdoRobot robot;
    public NOdoDrivetrain drivetrain;
    public Claw claw;
    public Lift lift;
    public TensorFlow detector;


    public NOdoLocation reset = new NOdoLocation(-15,-25,0);
    public NOdoLocation middleZone = new NOdoLocation(0, 625, 0);
    public NOdoLocation leftZone = new NOdoLocation(-585, 675, 0);
    public NOdoLocation rightZone = new NOdoLocation(600, 675,0);

    public NOdoLocation coneZone = new NOdoLocation(100, -175, 0);
    public NOdoLocation coneDrop = new NOdoLocation(200, -175, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new NOdoRobot(this);
        drivetrain = new NOdoDrivetrain(robot, motorNumbers, telemetry, hardwareMap.get(Servo.class, "LeftOdo"), hardwareMap.get(Servo.class, "BackOdo"), hardwareMap.get(Servo.class, "RightOdo"));
      detector = new TensorFlow(hardwareMap.get(WebcamName.class, "Webcam 1"), hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
        claw = new Claw(hardwareMap.get(Servo.class, "claw"));
//        lift = new Lift(hardwareMap.get(DcMotorEx.class, "liftT"),
//                hardwareMap.get(DcMotorEx.class, "liftM"),
//                hardwareMap.get(DcMotorEx.class, "liftB"),
//                hardwareMap.get(DigitalChannel.class, "Lift"), telemetry);

        drivetrain.odoDown();
        robot.odometry.reset();
        claw.open();

        waitForStart();
//        List<Recognition> detection = detector.getDetected();
        List<Recognition> detection = detector.getDetected();

        if (detection != null && detection.size() > 0)
            telemetry.addData("", detection.get(0).getLabel());

        drivetrain.moveToPositionMod(middleZone, 5, 5, 2, 0.3, 8000);
        if (detection != null && detection.size() > 0) {
            switch (detection.get(0).getLabel()) {
                case "Apple":
                    drivetrain.moveToPositionMod(leftZone, 5, 5,1, .3, 3000);
                    break;
                case "Lime":
                    if (detection.get(0).getConfidence() < .85){
                        drivetrain.moveToPositionMod(rightZone, 5, 5, 1, .3, 2000);
                    }
                    break;
                case "Orange":
                    drivetrain.moveToPositionMod(rightZone, 5, 5, 1, .3, 3000);
                    break;
                default:
                    break;
            }
        }
        sleep(5000);
        drivetrain.odoUp();
        sleep(1000);
    }
}
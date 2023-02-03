package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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

@Autonomous (name="JustPark", group="autonomous")
public class NOdoAutopark extends LinearOpMode {

    public int[] motorNumbers = {0, 1, 2, 3}; //creates motor numbers array

    public NOdoRobot robot;
    public NOdoDrivetrain drivetrain;
    public Claw claw;
    public Lift lift;
    public TensorFlow detector;

    public NOdoLocation middleZone = new NOdoLocation(800, 0, 0);
    public NOdoLocation leftZone = new NOdoLocation(800, -600, 0);
    public NOdoLocation rightZone = new NOdoLocation(800, 600,0);


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new NOdoRobot(this);
        drivetrain = new NOdoDrivetrain(robot, motorNumbers, telemetry, hardwareMap.get(Servo.class, "LeftOdo"), hardwareMap.get(Servo.class, "BackOdo"), hardwareMap.get(Servo.class, "RightOdo"));
        detector = new TensorFlow(hardwareMap.get(WebcamName.class, "Webcam 1"), hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
        claw = new Claw(hardwareMap.get(Servo.class, "claw"));
        lift = new Lift(hardwareMap.get(DcMotorEx.class, "liftT"),
                hardwareMap.get(DcMotorEx.class, "liftM"),
                hardwareMap.get(DcMotorEx.class, "liftB"),
                hardwareMap.get(DigitalChannel.class, "Lift"), telemetry);

        drivetrain.odoDown();
        claw.close();

        waitForStart();
        List<Recognition> detection = detector.getDetected();

        drivetrain.moveToPositionMod(middleZone, 5, 5, 1, 0.2, 4000);


        if (detection != null && detection.size() > 0) {
            switch (detection.get(0).getLabel()) {
                case "Apple":
                    drivetrain.moveToPositionMod(leftZone, 5, 5,1, .2, 4000);
                    break;
               case "Lime":
                    break;
                case "Orange":
                    drivetrain.moveToPositionMod(rightZone, 5, 5, 1, .2, 4000);
                    break;
                default:
                    break;
            }
        }
        drivetrain.odoUp();
        sleep(10000);
    }
}

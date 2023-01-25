package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;
import org.firstinspires.ftc.teamcode.mechanisms.NOdoDrivetrain;
import org.firstinspires.ftc.teamcode.teleop.NOdoRobot;
import org.firstinspires.ftc.teamcode.util.AutoPositions;
import org.firstinspires.ftc.teamcode.util.NOdoLocation;
import org.firstinspires.ftc.teamcode.util.TensorFlow;

import java.util.List;

@Autonomous (name="NOdoAuto", group="autonomous")
public class NOdoAuto extends LinearOpMode {

    public int[] motorNumbers = {0, 1, 2, 3}; //creates motor numbers array

    public NOdoRobot robot;
    public NOdoDrivetrain drivetrain;
    public Claw claw;
    public Lift lift;
    public TensorFlow detector;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new NOdoRobot(this);
        drivetrain = new NOdoDrivetrain(robot, motorNumbers, telemetry, hardwareMap.get(Servo.class, "LeftOdo"), hardwareMap.get(Servo.class, "BackOdo"), hardwareMap.get(Servo.class, "RightOdo"));
        detector = new TensorFlow(hardwareMap.get(WebcamName.class, "Webcam 1"), hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
        claw = new Claw(hardwareMap.get(Servo.class, "claw"));
        lift = new Lift(hardwareMap.get(DcMotorEx.class, "liftL"), hardwareMap.get(DcMotorEx.class, "liftR"), telemetry);

        drivetrain.odoDown();
        claw.close();

        waitForStart();
        List<Recognition> detection = detector.getDetected();

        drivetrain.moveToPositionMod(AutoPositions.smallForward, 5, 5, 1, .3, 2000);
        drivetrain.moveToPositionMod(AutoPositions.coneZone, 5, 5, 1, .6, 2000);
        lift.height = -2750;
        lift.write();
        sleep(5000);
        drivetrain.moveToPositionMod(AutoPositions.coneDrop, 5, 5, 1, 0.3, 2000);
        claw.open();
        drivetrain.moveToPositionMod(AutoPositions.coneZone, 5, 5, 1, .5, 2000);
        lift.height = 0;
        lift.write();
        sleep(1000);
        drivetrain.moveToPositionMod(AutoPositions.reset, 5, 5, 1, .5, 2000);
        drivetrain.moveToPositionMod(AutoPositions.middleZone, 5, 5, 1, 0.3, 2000);
        drivetrain.moveToPositionMod(AutoPositions.leftZone, 5, 5, 1, 0.3, 4000);
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

package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;
import org.firstinspires.ftc.teamcode.mechanisms.NOdoDrivetrain;
import org.firstinspires.ftc.teamcode.teleop.NOdoRobot;
import org.firstinspires.ftc.teamcode.util.NOdoLocation;
import org.firstinspires.ftc.teamcode.util.TensorFlow;

@Autonomous (name="Non-Terminal Auto", group="autonomous")
public class NoTerminalAuto extends LinearOpMode {

    public int[] motorNumbers = {0, 1, 2, 3}; //creates motor numbers array

    public NOdoRobot robot;
    public NOdoDrivetrain drivetrain;
    public Claw claw;
    public Lift lift;
    public TensorFlow detector;


    public NOdoLocation reset = new NOdoLocation(-15,-25,0);
    public NOdoLocation middleZone = new NOdoLocation(0, 650, 0);
    public NOdoLocation leftZone = new NOdoLocation(-565, 650, 0);
    public NOdoLocation rightZone = new NOdoLocation(600, 650,0);

    public NOdoLocation coneZone = new NOdoLocation(100, -175, 0);
    public NOdoLocation coneDrop = new NOdoLocation(200, -175, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new NOdoRobot(this);
        drivetrain = new NOdoDrivetrain(robot, motorNumbers, telemetry, hardwareMap.get(Servo.class, "LeftOdo"), hardwareMap.get(Servo.class, "BackOdo"), hardwareMap.get(Servo.class, "RightOdo"));
//        detector = new TensorFlow(hardwareMap.get(WebcamName.class, "Webcam 1"), hardwareMap.appContext.getResources().getIdentifier(
//                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
//        claw = new Claw(hardwareMap.get(Servo.class, "claw"));
//        lift = new Lift(hardwareMap.get(DcMotorEx.class, "liftT"),
//                hardwareMap.get(DcMotorEx.class, "liftM"),
//                hardwareMap.get(DcMotorEx.class, "liftB"),
//                hardwareMap.get(DigitalChannel.class, "Lift"), telemetry);

        drivetrain.odoDown();
        robot.odometry.reset();
        //claw.close();
        waitForStart();
//        List<Recognition> detection = detector.getDetected();

        drivetrain.moveToPositionMod(middleZone, 5, 5, 2, 0.3, 8000);
        sleep(1000);
        drivetrain.moveToPositionMod(leftZone, 5, 5, 2, 0.3, 8000);
        drivetrain.odoUp();
        sleep(500);
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
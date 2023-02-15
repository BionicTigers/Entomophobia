package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.teleop.Robot;
import org.firstinspires.ftc.teamcode.util.Location;
import org.firstinspires.ftc.teamcode.util.TensorFlow;

import java.util.List;

@Autonomous (name="Blue Auto Right", group="autonomous")
public class BlueAutoRight extends LinearOpMode {

    public int[] motorNumbers = {0, 1, 2, 3}; //creates motor numbers array

    public Robot robot;
    public Drivetrain drivetrain;
    public Claw claw;
    public Lift lift;
    public TensorFlow detector;

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

        while (opModeInInit()) {
            List<Recognition> detection = detector.getDetected();

            if (detection != null && detection.size() > 0) {
                telemetry.addData("Detection", detection.get(0).getLabel());
                telemetry.update();
            }
        }

        waitForStart();
        List<Recognition> detection = detector.getDetected();
        telemetry.update();

        sleep(3000);

        drivetrain.moveToPositionMod(terminal, 5, 5, 0, 0.2, 2000);
        sleep(250);
        drivetrain.moveToPositionMod(origin, 5, 5, 0, 0.3, 2000);
        sleep(250);
        drivetrain.moveToPositionMod(middleZone, 5, 5, 0, 0.3, 2000);
        sleep(250);

        if (detection != null && detection.size() > 0) {
            switch (detection.get(0).getLabel()) {
                case "Apple":
                    drivetrain.moveToPositionMod(leftZone, 5, 5,1, .3, 2000);
                    break;
                case "Lime":
                    if (detection.get(0).getConfidence() < .85){
                        drivetrain.moveToPositionMod(rightZone, 5, 5, 1, .3, 2000);
                    }
                    break;
                case "Orange":
                    drivetrain.moveToPositionMod(rightZone, 5, 5, 1, .3, 2000);
                    break;
                default:
                    break;
            }
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
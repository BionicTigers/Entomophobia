package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.util.OpenCv;
import org.firstinspires.ftc.teamcode.util.Signal;

import java.util.HashMap;

import org.firstinspires.ftc.teamcode.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.teleop.Robot;
import org.firstinspires.ftc.teamcode.util.Location;
import org.firstinspires.ftc.teamcode.util.VisionConstants;

@Autonomous (name="Blue Auto Right", group="autonomous")
public class ScoringAutoRight extends LinearOpMode {

    public int[] motorNumbers = {0, 1, 2, 3}; //creates motor numbers array

    public Robot robot;
    public Drivetrain drivetrain;
    public Claw claw;
//    public Lift lift;

    public Arm arm;

    public OpenCv detector;
    public HashMap<String, Signal> signals = new HashMap<>();

    public boolean a = false;

    public Location reset = new Location(-15,-25,0);
    public Location middleZone = new Location(0, 650, 0);
    public Location leftZone = new Location(-585, 675, 0);
    public Location rightZone = new Location(600, 675,0);

    public Location origin = new Location(0, 0, 0);

    public Location avoidPenalty = new Location(0, 150, 0); //Moves far enough from the wall that the arm won't hang over

    public Location prescore = new Location(-270, 150, 0); //Aligns with the junction while still avoiding the penalty

    public Location scoring = new Location(-270, 25, 0); //Moves back to be able to score

    public Location postscore = new Location(0, 50, 0); //Moves to be able to park accurately after the score

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        drivetrain = new Drivetrain(robot, motorNumbers, telemetry, hardwareMap.get(Servo.class, "LeftOdo"), hardwareMap.get(Servo.class, "BackOdo"), hardwareMap.get(Servo.class, "RightOdo"));
        detector = new OpenCv(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                signals,
                hardwareMap.appContext.getResources().getIdentifier(
                        "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
        claw = new Claw(hardwareMap.get(Servo.class, "claw"), hardwareMap.get(DistanceSensor.class, "distance"), hardwareMap.get(RevBlinkinLedDriver.class, "blinkin"), telemetry);
//        lift = new Lift(hardwareMap.get(DcMotorEx.class, "liftT"), hardwareMap.get(DcMotorEx.class, "liftM"), hardwareMap.get(DcMotorEx.class, "liftB"), hardwareMap.get(DigitalChannel.class, "Lift"), telemetry);
        arm = new Arm(hardwareMap.get(CRServo.class, "armL"), hardwareMap.get(CRServo.class, "armR"), hardwareMap, telemetry);
        signals.put("Orange", VisionConstants.ORANGE);
        signals.put("Purple", VisionConstants.PURPLE);
        signals.put("Green", VisionConstants.GREEN);

//comments are original, new ones are bc of messed up lighting
        drivetrain.odoDown();
        robot.odometry.reset();
        claw.open();

        while (opModeInInit()) {
            telemetry.addData("Reading", detector.getDetection());
            telemetry.update();
        }

        waitForStart();
        String detection = detector.getDetection();
// arm broken so comment that stuff out baby, also this program is illegal as you are not within 18 by 18 on init.  skill issue.
// Grabs cone
        claw.close();
        sleep(250);
        //Moves to the initial location to avoid breaking the plane of the field
        drivetrain.moveToPositionMod(avoidPenalty, 5, 5, 0, 0.2, 2000);
        while(drivetrain.currentState == Drivetrain.State.MOVE_TO_POSITION && opModeIsActive()){
            drivetrain.write();
        }
        sleep(250);
        //Moves the arm to scoring position
        arm.move(150);
        while(arm.currentState == Arm.State.MOVING && opModeIsActive()) {
            arm.write();
        }
        sleep(250);
        drivetrain.moveToPositionMod(prescore, 5, 5, 0, 0.2, 2000);
        while(drivetrain.currentState == Drivetrain.State.MOVE_TO_POSITION && opModeIsActive()){
            drivetrain.write();
        }
        sleep(250);
        arm.move(111);
        while(arm.currentState == Arm.State.MOVING && opModeIsActive()){
            arm.write();
        }
        sleep(250);
        //Moves robot to scoring position
        drivetrain.moveToPositionMod(scoring, 5, 5, 0, 0.2, 1000);
        while(drivetrain.currentState == Drivetrain.State.MOVE_TO_POSITION && opModeIsActive()){
            drivetrain.write();
        }
        sleep(2000);
        //Opens claw
        claw.open();
        sleep(250);
        //Returns the robot back to the prescore position
        drivetrain.moveToPositionMod(postscore, 5, 5, 0, 0.2, 1000);
        while(drivetrain.currentState == Drivetrain.State.MOVE_TO_POSITION && opModeIsActive()){
            drivetrain.write();
        }
        sleep(250);
        //Moves arm down
        arm.move(0);
        while(arm.currentState == Arm.State.MOVING && opModeIsActive()) {
            arm.write();
        }
        sleep(250);
        //Returns the robot to the origin
        drivetrain.moveToPositionMod(origin, 5, 5, 0, 0.3, 2000);
        while(drivetrain.currentState == Drivetrain.State.MOVE_TO_POSITION && opModeIsActive()){
            drivetrain.write();
        }
        sleep(250);
        //Moves the robot to the middle zone
        drivetrain.moveToPositionMod(middleZone, 5, 5, 0, 0.3, 2000);
        while(drivetrain.currentState == Drivetrain.State.MOVE_TO_POSITION && opModeIsActive()){
            drivetrain.write();
        }
        sleep(250);
        //Moves the robot to the detected position
            switch (detection) {
                case "Orange":

                    drivetrain.moveToPositionMod(leftZone, 5, 5,1, .3, 2000);
                    while(drivetrain.currentState == Drivetrain.State.MOVE_TO_POSITION && opModeIsActive()){
                        drivetrain.write();
                    }
                    break;
                case "Purple":
                    break;
                case "Green":
                    drivetrain.moveToPositionMod(rightZone, 5, 5, 1, .3, 2000);
                    while(drivetrain.currentState == Drivetrain.State.MOVE_TO_POSITION && opModeIsActive()){
                        drivetrain.write();
                    }
                    break;
                default:
                    break;
            }
        //Retracts the odometry pods
        drivetrain.odoUp();
        sleep(5000);
    }
}
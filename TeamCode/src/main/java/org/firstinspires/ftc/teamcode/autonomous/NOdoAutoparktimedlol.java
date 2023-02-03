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

@Autonomous (name="JustParktimed", group="autonomous")
public class NOdoAutoparktimedlol extends LinearOpMode {

    public int[] motorNumbers = {0, 1, 2, 3}; //creates motor numbers array

    public NOdoRobot robot;
    public NOdoDrivetrain drivetrain;
    public Claw claw;
    public Lift lift;
    public TensorFlow detector;

    public NOdoLocation middleZone = new NOdoLocation(800, 0, 0);
    public NOdoLocation leftZone = new NOdoLocation(825, -600, 0);
    public NOdoLocation rightZone = new NOdoLocation(825, 600,0);


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


        waitForStart();
        List<Recognition> detection = detector.getDetected();




        if (detection != null && detection.size() > 0) {
            switch (detection.get(0).getLabel()) {
                case "Apple":
                    drivetrain.robot.motors.get(0).setPower(0.8);
                    drivetrain.robot.motors.get(1).setPower(0.8);
                    drivetrain.robot.motors.get(2).setPower(0.8);
                    drivetrain.robot.motors.get(3).setPower(0.8);
                    sleep(600);
                    drivetrain.robot.motors.get(0).setPower(0);
                    drivetrain.robot.motors.get(1).setPower(0);
                    drivetrain.robot.motors.get(2).setPower(0);
                    drivetrain.robot.motors.get(3).setPower(0);
                    sleep(1000);
                    //drivetrain.timedMove(3000, 0.75, 0.75);
                    drivetrain.robot.motors.get(0).setPower(0.8);
                    drivetrain.robot.motors.get(1).setPower(-0.55);
                    drivetrain.robot.motors.get(2).setPower(0.8);
                    drivetrain.robot.motors.get(3).setPower(-0.55);
                    sleep(1200);
                    drivetrain.robot.motors.get(0).setPower(0);
                    drivetrain.robot.motors.get(1).setPower(0);
                    drivetrain.robot.motors.get(2).setPower(0);
                    drivetrain.robot.motors.get(3).setPower(0);
                    break;
               case "Lime":
                   drivetrain.robot.motors.get(0).setPower(0.85);
                   drivetrain.robot.motors.get(1).setPower(0.8);
                   drivetrain.robot.motors.get(2).setPower(0.8);
                   drivetrain.robot.motors.get(3).setPower(0.85);
                   sleep(700);
                   drivetrain.robot.motors.get(0).setPower(0);
                   drivetrain.robot.motors.get(1).setPower(0);
                   drivetrain.robot.motors.get(2).setPower(0);
                   drivetrain.robot.motors.get(3).setPower(0);
                   break;
                case "Orange":
                    drivetrain.robot.motors.get(0).setPower(0.8);
                    drivetrain.robot.motors.get(1).setPower(0.7);
                    drivetrain.robot.motors.get(2).setPower(0.7);
                    drivetrain.robot.motors.get(3).setPower(0.8);
                    sleep(650);
                    drivetrain.robot.motors.get(0).setPower(0);
                    drivetrain.robot.motors.get(1).setPower(0);
                    drivetrain.robot.motors.get(2).setPower(0);
                    drivetrain.robot.motors.get(3).setPower(0);
                    sleep(1000);
                    drivetrain.robot.motors.get(0).setPower(-0.8); //front right
                    drivetrain.robot.motors.get(1).setPower(0.8); //front left
                    drivetrain.robot.motors.get(2).setPower(-0.7); //back left
                    drivetrain.robot.motors.get(3).setPower(0.7); // back right
                    sleep(1250);
                    drivetrain.robot.motors.get(0).setPower(0);
                    drivetrain.robot.motors.get(1).setPower(0);
                    drivetrain.robot.motors.get(2).setPower(0);
                    drivetrain.robot.motors.get(3).setPower(0);
                    break;
                default:
                    break;
            }
        }
        sleep(1000);
    }
}

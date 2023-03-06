package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;
import org.firstinspires.ftc.teamcode.teleop.Robot;
import org.firstinspires.ftc.teamcode.util.AutoLocations;
import org.firstinspires.ftc.teamcode.util.OpenCv;
import org.firstinspires.ftc.teamcode.util.Signal;
import org.firstinspires.ftc.teamcode.util.VisionConstants;

import java.util.HashMap;

@Autonomous(name = "*New* Blue Auto Right", group = "Yes")
public class NewAuto extends LinearOpMode {
    private Robot robot;
    private Drivetrain drivetrain;
    private Claw claw;
    private Lift lift;
    private Arm arm;

    private OpenCv detector;
    private HashMap<String, Signal> signals;

    @Override
    public void runOpMode() throws InterruptedException {
        signals = new HashMap<>();
        signals.put("Orange", VisionConstants.ORANGE);
        signals.put("Purple", VisionConstants.PURPLE);
        signals.put("Green", VisionConstants.GREEN);

        robot = new Robot(this);

        drivetrain = new Drivetrain(robot, telemetry,
                hardwareMap.get(Servo.class, "LeftOdo"),
                hardwareMap.get(Servo.class, "BackOdo"),
                hardwareMap.get(Servo.class, "RightOdo"));

        claw = new Claw(hardwareMap.get(Servo.class, "claw"),
                hardwareMap.get(DistanceSensor.class, "distance"),
                hardwareMap.get(RevBlinkinLedDriver.class, "blinkin"), telemetry);

        lift = new Lift(hardwareMap.get(DcMotorEx.class, "liftT"),

                hardwareMap.get(DcMotorEx.class, "liftM"),
                hardwareMap.get(DcMotorEx.class, "liftB"),
                hardwareMap.get(DigitalChannel.class, "Lift"),
                telemetry);

        arm = new Arm(hardwareMap.get(CRServo.class, "armL"),
                hardwareMap.get(CRServo.class, "armR"),
                hardwareMap,
                telemetry);

        detector = new OpenCv(hardwareMap.get(WebcamName.class, "Webcam 1"), signals);

        claw.open();

        drivetrain.odoDown();
        while (opModeInInit()) {
            telemetry.addData("Reading", detector.getDetection());
            telemetry.update();

        }
        String detection = detector.getDetection();

                waitForStart();
//        detection = detector.getDetection();
//        detector.stopDetection();

        //Score Preload
        claw.close();

        drivetrain.moveToPositionMod(AutoLocations.closeLow, 5,5, 2, .3, 400);
        arm.move(140);
        while ((drivetrain.currentState == Drivetrain.State.MOVE_TO_POSITION
                || arm.currentState == Arm.State.MOVING)
                && opModeIsActive()) {
            telemetry.addData("arm: ", arm.currentState);
            telemetry.addData("dt: ", drivetrain.currentState);

            drivetrain.write();
            arm.write();
        }
        arm.move(107);
        double heldStart = robot.getTimeMS();

        //keep arm up while claw opens
        while ((robot.getTimeMS()-heldStart < 1000
                || arm.currentState == Arm.State.MOVING)
                && opModeIsActive()) {
            arm.write();
        }

        claw.open();

        arm.move(140);
        heldStart = robot.getTimeMS();

        //keep arm up while claw opens
        while ((robot.getTimeMS()-heldStart < 200
                || arm.currentState == Arm.State.MOVING)
                && opModeIsActive()) {
            arm.write();
        }

        //Move to center of origin tile
        arm.move(115);
        drivetrain.moveToPositionMod(AutoLocations.origin, 5,5, 2, .3, 2000);
        while ((drivetrain.currentState == Drivetrain.State.MOVE_TO_POSITION
                || arm.currentState == Arm.State.MOVING)
                && opModeIsActive()) {
            telemetry.addData("arm: ",arm.currentState);
            telemetry.addData("dt: ", drivetrain.currentState);

            drivetrain.write();
            arm.write();
        }
//        //Pick up a cone from the stack
//        arm.move(0);
//        drivetrain.moveToPositionMod(AutoLocations.stackLineUp, 5,5, 2, .3, 4000);
//        while ((drivetrain.currentState == Drivetrain.State.MOVE_TO_POSITION
//                || arm.currentState == Arm.State.MOVING)
//                && opModeIsActive()) {
//            telemetry.addData("arm: ",arm.currentState);
//            telemetry.addData("dt: ", drivetrain.currentState);
//
//            drivetrain.write();
//            arm.write();
//        }
//
//        drivetrain.moveToPositionMod(AutoLocations.stackClose, 5,5, 2, .3, 2000);
//        while (drivetrain.currentState == Drivetrain.State.MOVE_TO_POSITION
//                && opModeIsActive()) {
//            telemetry.addData("dt", drivetrain.currentState);
//
//            drivetrain.write();
//        }
//
//        drivetrain.moveToPositionMod(AutoLocations.stack, 2,2, 0, .2, 2000);
//        while (drivetrain.currentState == Drivetrain.State.MOVE_TO_POSITION
//                && opModeIsActive()) {
//            telemetry.addData("dt", drivetrain.currentState);
//            drivetrain.write();
//        }
//
//        arm.move(20); //TODO: update the angle to be correct on all arm
//        while (arm.currentState == Arm.State.MOVING
//                && opModeIsActive()) {
//            telemetry.addData("arm",arm.currentState);
//            arm.write();
//        }
//
//        claw.close();
//
//        arm.move(30);
//        while (arm.currentState == Arm.State.MOVING
//                && opModeIsActive()) {
//            telemetry.addData("arm",arm.currentState);
//            arm.write();
//        }
    }
}

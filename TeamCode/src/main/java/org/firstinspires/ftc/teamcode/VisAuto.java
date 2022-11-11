package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@Autonomous(name="VisAuto", group="autonomous")
public class VisAuto extends LinearOpMode {
    public Robot robot;
    public Drivetrain drivetrain;
    public String[] motorNames = {"frontRight","frontLeft","backLeft","backRight"};
    public int[] motorNumbers = {0, 1, 2, 3}; //creates motor numbers array
//    public Deposit deposit;
private Vision vision;
    public void runOpMode(){
        robot = new Robot(this);
        drivetrain = new Drivetrain(robot, motorNumbers, telemetry);
//        deposit = new Deposit(hardwareMap.get(Servo.class, "outputServo"));
        vision = new Vision();
        Deadline stop = new Deadline(500, TimeUnit.MILLISECONDS);
        waitForStart();
        stop.reset();
        OpenCvCamera webcam;

       int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.setPipeline(new Vision());
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        vision = new Vision(webcam);
      telemetry.addData("Area", vision.getArea());

        drivetrain.motors.get(0).setPower(0.5);
        drivetrain.motors.get(1).setPower(0.5);
        drivetrain.motors.get(2).setPower(0.5);
        drivetrain.motors.get(3).setPower(0.5);
        if(stop.hasExpired()){
            drivetrain.motors.get(0).setPower(0);
            drivetrain.motors.get(1).setPower(0);
            drivetrain.motors.get(2).setPower(0);
            drivetrain.motors.get(3).setPower(0);
        }
    }
}
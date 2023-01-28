package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.util.TensorFlow;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;

import java.util.List;

@Autonomous(name="TimedAuto", group="autonomous")
public class TimedAuto extends LinearOpMode {
    //Declares motor arrays
    public int[] motorNumbers = {0, 1, 2, 3}; //creates motor numbers array

    //Declares mechanisms
    public Robot robot;
    public Drivetrain drivetrain;
    public TensorFlow tensorflow;

    private List<Recognition> detected;
    private byte randomization = 2;
    //public TensorFlowWebcam tensorFlow = ne
    //
    // w TensorFlowWebcam();

    @Override
    public void runOpMode() throws InterruptedException {
        //On initialization
        robot = new Robot(this);
        drivetrain = new Drivetrain(robot, motorNumbers, telemetry);
        tensorflow = new TensorFlow(hardwareMap.get(WebcamName.class, "Webcam 1"), hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
        System.out.println("Init Tensorflow");

        do {
            detected = tensorflow.getDetected();
        } while (!isStarted());

        if (detected != null) {
            if (detected.size() != 0) {
                if (detected.get(0).getLabel() == "Apple") {
                    randomization = 1;
                } else if (detected.get(0).getLabel() == "Orange") {
                    randomization = 3;
                }
                telemetry.addData("Start", detected.get(0).getLabel());
                telemetry.update();
            }
        }

        telemetry.addData("Start", "Default");
        telemetry.update();

        //On start

        //Scans signal sleeve
        //randomization =
        drivetrain.robot.motors.get(0).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drivetrain.robot.motors.get(1).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drivetrain.robot.motors.get(2).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drivetrain.robot.motors.get(3).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //drivetrain.timedMove(3000, 0, 0.75);

        switch (randomization) {
            //Strafes to the left 1 tile
            case 1:
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

            //Moves forward 1 tile
            case 2:
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
          //  Strafes to the right 1 tile
            case 3://drivetrain.timedMove(3000, 0.25, 0.75);
                //Moves forward 1 tile
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

        }
    }
}

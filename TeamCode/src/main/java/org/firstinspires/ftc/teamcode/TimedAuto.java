package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@Autonomous(name="TimedAuto", group="autonomous")
public class TimedAuto extends LinearOpMode {
    //Declares motor arrays
    public int[] motorNumbers = {0, 1, 2, 3}; //creates motor numbers array

    //Declares mechanisms
    public Robot robot;
    public Drivetrain drivetrain;
    //public TensorFlowWebcam tensorFlow = new TensorFlowWebcam();

    @Override
    public void runOpMode() throws InterruptedException {
        //On initialization
        robot = new Robot(this);
        drivetrain = new Drivetrain(robot, motorNumbers, telemetry);

        byte randomization = 3;

        waitForStart();
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
                sleep(1050);
                drivetrain.robot.motors.get(0).setPower(0);
                drivetrain.robot.motors.get(1).setPower(0);
                drivetrain.robot.motors.get(2).setPower(0);
                drivetrain.robot.motors.get(3).setPower(0);
                break;
            //Moves forward 1 tile
            case 2:
                drivetrain.robot.motors.get(0).setPower(0.8);
                drivetrain.robot.motors.get(1).setPower(0.8);
                drivetrain.robot.motors.get(2).setPower(0.8);
                drivetrain.robot.motors.get(3).setPower(0.8);
                sleep(600);
                drivetrain.robot.motors.get(0).setPower(0);
                drivetrain.robot.motors.get(1).setPower(0);
                drivetrain.robot.motors.get(2).setPower(0);
                drivetrain.robot.motors.get(3).setPower(0);
                break;
            //Strafes to the right 1 tile
            case 3://drivetrain.timedMove(3000, 0.25, 0.75);
                //Moves forward 1 tile
                drivetrain.robot.motors.get(0).setPower(0.8);
                drivetrain.robot.motors.get(1).setPower(0.7);
                drivetrain.robot.motors.get(2).setPower(0.7);
                drivetrain.robot.motors.get(3).setPower(0.8);
                sleep(600);
                drivetrain.robot.motors.get(0).setPower(0);
                drivetrain.robot.motors.get(1).setPower(0);
                drivetrain.robot.motors.get(2).setPower(0);
                drivetrain.robot.motors.get(3).setPower(0);
                sleep(1000);
                drivetrain.robot.motors.get(0).setPower(-0.8); //front right
                drivetrain.robot.motors.get(1).setPower(0.8); //front left
                drivetrain.robot.motors.get(2).setPower(-0.7); //back left
                drivetrain.robot.motors.get(3).setPower(0.7); // back right
                sleep(1100);
                drivetrain.robot.motors.get(0).setPower(0);
                drivetrain.robot.motors.get(1).setPower(0);
                drivetrain.robot.motors.get(2).setPower(0);
                drivetrain.robot.motors.get(3).setPower(0);
                break;
        }
    }
}

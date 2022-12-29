package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

public class TimedAuto extends LinearOpMode {
    //Declares motor arrays
    public String[] motorNames = {"frontRight", "frontLeft", "backLeft", "backRight"};
    public int[] motorNumbers = {0, 1, 2, 3}; //creates motor numbers array

    //Declares mechanisms
    public Robot robot = new Robot(this);
    public Drivetrain drivetrain = new Drivetrain(robot, motorNumbers, telemetry);
    //public TensorFlowWebcam tensorFlow = new TensorFlowWebcam();

    @Override
    public void runOpMode() throws InterruptedException {
        //On initialization
        byte randomization = 2;

        waitForStart();
        //On start
        //Scan signal sleeve
        //randomization =

        //Moves forward 1 tile
        //Deadline tile = new Deadline(1000, TimeUnit.MILLISECONDS);
        drivetrain.moveToPosition(new Location(0, 0, 0, 0), 0.0, 0.0, 0.0, 0);
        switch (randomization) {
            //Strafes to the left 1 tile
            case 1:
                drivetrain.moveToPosition(new Location(0, 0, 0, 0), 0.0, 0.0, 0.0, 0);
                break;
            //Strafes to the right 1 tile
            case 3:
                drivetrain.moveToPosition(new Location(0, 0, 0, 0), 0.0, 0.0, 0.0, 0);
                break;
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.Vuforia;

@Autonomous (name="VisAuto", group="autonomous")
public class VisAuto extends LinearOpMode {
    public Robot robot;
    public Drivetrain drivetrain;
    public String[] motorNames = {"frontRight","frontLeft","backLeft","backRight"};
    public int[] motorNumbers = {0, 1, 2, 3}; //creates motor numbers array
    public Claw claw;

    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        drivetrain = new Drivetrain(robot, motorNumbers, telemetry);

        waitForStart();

        while(opModeIsActive()) {

        }
    }
}

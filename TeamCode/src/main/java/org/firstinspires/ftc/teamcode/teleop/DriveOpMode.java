package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;

@TeleOp(name = "DriveOpMode")
public class DriveOpMode extends LinearOpMode {
    public Robot robot;
    public Drivetrain drivetrain;
    public String[] motorNames = {"frontRight","frontLeft","backLeft","backRight"};
    public int[] motorNumbers = {0, 1, 2, 3}; //creates motor numbers array
    public Telemetry telemetry;

    public void runOpMode(){
        robot = new Robot(this);
        drivetrain = new Drivetrain(robot, motorNumbers, telemetry);
        waitForStart();
        while (opModeIsActive()){
            drivetrain.update(gamepad1, gamepad2);
            drivetrain.write();
        }
    }
}
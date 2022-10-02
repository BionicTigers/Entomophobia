package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "DriveOpMode")
public class DriveOpMode extends LinearOpMode {
    public Robot robot;
    public Drivetrain drivetrain;
    public String[] motorNames = {"frontRight","frontLeft","backLeft","backRight"};
    public int[] motorNumbers = {0, 1, 2, 3}; //creates motor numbers array
    public Telemetry telemetry;

    public void runOpMode(){
        robot = new Robot(this);
        drivetrain = new Drivetrain(robot, motorNumbers);
        waitForStart();
        while (opModeIsActive()){
            drivetrain.update(gamepad1, gamepad2);
            drivetrain.write();
        }
    }
}
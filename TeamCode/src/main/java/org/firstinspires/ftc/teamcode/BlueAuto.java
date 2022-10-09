package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@Autonomous(name="BlueAuto", group="autonomous")
public class BlueAuto extends LinearOpMode {
    public Robot robot;
    public Drivetrain drivetrain;
    public String[] motorNames = {"frontRight","frontLeft","backLeft","backRight"};
    public int[] motorNumbers = {0, 1, 2, 3}; //creates motor numbers array
    public Deposit deposit;

    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        drivetrain = new Drivetrain(robot, motorNumbers);
        deposit = new Deposit(hardwareMap.get(Servo.class, "outputServo"));
        Deadline stop = new Deadline(500, TimeUnit.MILLISECONDS);
        waitForStart();
        stop.reset();
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

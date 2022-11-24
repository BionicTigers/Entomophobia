package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
/*
@TeleOp(name = "LiftOut")
public class LiftOutputOp extends LinearOpMode {

    public Lift lift;
    public Deposit deposit;
    public Robot robot;
    public Drivetrain drivetrain;

    public int[] motorNumbers = {0, 1, 2, 3}; //creates motor numbers array

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(this);
        drivetrain = new Drivetrain(robot, motorNumbers, telemetry);
        lift = new Lift (hardwareMap.get(DcMotorEx.class, "liftMotor"), telemetry);
        deposit = new Deposit(hardwareMap.get(Servo.class, "deposit"));

        waitForStart();

        while (opModeIsActive()) {
            lift.update(gamepad1, gamepad2);
            lift.write();
            deposit.update(gamepad1, gamepad2);
            deposit.write();
            telemetry.update();
        }
    }
}*/

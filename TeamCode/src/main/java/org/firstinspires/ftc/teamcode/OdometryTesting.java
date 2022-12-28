package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "OdoTesting")
public class OdometryTesting extends LinearOpMode {
    public NOdoRobot robot;
    public NOdoDrivetrain drive;
    public int[] motorNumbers = {0, 1, 2, 3};
    public Intake intake;
    public Lift lift;
    public Claw claw;

    public void runOpMode() {

        robot = new NOdoRobot(this);
        drive = new NOdoDrivetrain(robot, motorNumbers, telemetry, hardwareMap.get(Servo.class, "LeftOdo"), hardwareMap.get(Servo.class, "RightOdo"), hardwareMap.get(Servo.class, "BackOdo"));//        intake = new Intake(hardwareMap.get(DcMotorEx.class, "intakeMotor"));
//        lift = new Lift(hardwareMap.get(DcMotorEx.class, "liftMotor"), telemetry);
//        deposit = new Deposit(hardwareMap.get(Servo.class, "deposit"));
//        claw = new Claw(hardwareMap.get(Servo.class, "clawl"), hardwareMap.get(Servo.class, "clawr"));
        Mechanism[] mechanisms = {drive/*, lift/*, claw*/};

        waitForStart();
//        claw.initopen();
        while(opModeIsActive()) {
            for (Mechanism mech : mechanisms) { //For each mechanism in the mechanism array
                mech.update(gamepad1, gamepad2); //Run their respective update methods
            }

            for (Mechanism mech : mechanisms) { //For each mechanism in the mechanism array
                mech.write(); //Run their respective write methods
            }
            telemetry.update();
        }
    }
}
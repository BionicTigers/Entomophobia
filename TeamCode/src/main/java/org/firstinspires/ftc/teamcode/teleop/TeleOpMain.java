package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.Lift;
import org.firstinspires.ftc.teamcode.util.Mechanism;
import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.openftc.revextensions2.ExpansionHubEx;

@TeleOp(name = "TeleOpMain")
public class TeleOpMain extends LinearOpMode {
    public Robot robot;
    public Drivetrain drive;
    public int[] motorNumbers = {0, 1, 2, 3};
    public Lift lift;
    public Claw claw;
    public Arm arm;

    public void runOpMode() {

        robot = new Robot(this);
        drive = new Drivetrain(robot, motorNumbers, telemetry, hardwareMap.get(Servo.class, "LeftOdo"), hardwareMap.get(Servo.class, "BackOdo"), hardwareMap.get(Servo.class, "RightOdo"));
        lift = new Lift(hardwareMap.get(DcMotorEx.class, "liftT"),
                hardwareMap.get(DcMotorEx.class, "liftM"),
                hardwareMap.get(DcMotorEx.class, "liftB"),
                hardwareMap.get(DigitalChannel.class, "Lift"), telemetry);
        claw = new Claw(hardwareMap.get(Servo.class, "claw"), hardwareMap.get(DistanceSensor.class, "distance"), hardwareMap.get(RevBlinkinLedDriver.class, "blinkin"), telemetry);
        arm = new Arm(hardwareMap.get(CRServo.class, "armL"), hardwareMap.get(CRServo.class, "armR"), hardwareMap, telemetry/*, hardwareMap.get(DigitalChannel.class, "CBFront"), hardwareMap.get(DigitalChannel.class, "CBBack")*/);
        Mechanism[] mechanisms = {drive, lift, claw, arm};

        claw.init();
        while(!isStarted() && !isStopRequested()){
            telemetry.addData("Arm ticks: ", arm.controlHub.getEncoderTicks(3));
            telemetry.addData("Arm degrees: ", arm.controlHub.getEncoderTicks(3)/(8192/360));
            telemetry.update();
            arm.controlHub.refreshBulkData();
        }

        waitForStart();

        drive.odoUp();
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
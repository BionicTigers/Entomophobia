package org.firstinspires.ftc.teamcode.teleop;

import android.icu.text.LocaleDisplayNames;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.util.Mechanism;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.NOdoDrivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;

@TeleOp(name = "TeleOpMain")
public class TeleOpMain extends LinearOpMode {
    public NOdoRobot robot;
    public NOdoDrivetrain drive;
    public int[] motorNumbers = {0, 1, 2, 3};
    public Lift lift;
    public Claw claw;
    public Arm arm;

    public void runOpMode() {
        robot = new NOdoRobot(this);
        drive = new NOdoDrivetrain(robot, motorNumbers, telemetry, hardwareMap.get(Servo.class, "LeftOdo"), hardwareMap.get(Servo.class, "BackOdo"), hardwareMap.get(Servo.class, "RightOdo"));
        lift = new Lift(hardwareMap.get(DcMotorEx.class, "liftT"),
                hardwareMap.get(DcMotorEx.class, "liftM"),
                hardwareMap.get(DcMotorEx.class, "liftB"),
                hardwareMap.get(DigitalChannel.class, "Lift"), telemetry);
        claw = new Claw(hardwareMap.get(Servo.class, "claw"));
        arm = new Arm(hardwareMap.get(CRServo.class, "armL"), hardwareMap.get(CRServo.class, "armR"), telemetry/*, hardwareMap.get(DigitalChannel.class, "CBFront"), hardwareMap.get(DigitalChannel.class, "CBBack")*/);
        Mechanism[] mechanisms = {drive, lift, claw, arm};

        claw.init();

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
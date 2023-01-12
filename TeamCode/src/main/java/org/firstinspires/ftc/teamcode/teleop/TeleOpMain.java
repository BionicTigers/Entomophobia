package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Mechanism;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;

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
        drive = new Drivetrain(robot, motorNumbers, telemetry);
        lift = new Lift(hardwareMap.get(DcMotorEx.class, "liftL"), hardwareMap.get(DcMotorEx.class, "liftR"), telemetry);
        claw = new Claw(hardwareMap.get(Servo.class, "claw"));
        arm = new Arm(hardwareMap.get(CRServo.class, "armL"), hardwareMap.get(CRServo.class, "armR"), telemetry);
        Mechanism[] mechanisms = {drive, lift, claw, arm};

        waitForStart();
        claw.init();
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
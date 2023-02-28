package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;
import org.firstinspires.ftc.teamcode.teleop.Robot;
import org.firstinspires.ftc.teamcode.util.OpenCv;
import org.firstinspires.ftc.teamcode.util.Signal;
import org.firstinspires.ftc.teamcode.util.VisionConstants;

import java.util.HashMap;

@Autonomous(name = "Touch Grass", group = "Yes")
public class NewAuto extends LinearOpMode {
    private Robot robot;
    private Drivetrain drivetrain;
    private Claw claw;
    private Lift lift;
    private Arm arm;

    private OpenCv detector;
    private HashMap<String, Signal> signals;

    @Override
    public void runOpMode() throws InterruptedException {
        signals = new HashMap<>();
        signals.put("Orange", VisionConstants.ORANGE);
        signals.put("Purple", VisionConstants.PURPLE);
        signals.put("Green", VisionConstants.GREEN);

        robot = new Robot(this);

        drivetrain = new Drivetrain(robot, telemetry,
                hardwareMap.get(Servo.class, "LeftOdo"),
                hardwareMap.get(Servo.class, "BackOdo"),
                hardwareMap.get(Servo.class, "RightOdo"));

        claw = new Claw(hardwareMap.get(Servo.class, "claw"),
                hardwareMap.get(DistanceSensor.class, "distance"),
                hardwareMap.get(RevBlinkinLedDriver.class, "blinkin"));

        lift = new Lift(hardwareMap.get(DcMotorEx.class, "liftT"),
                hardwareMap.get(DcMotorEx.class, "liftM"),
                hardwareMap.get(DcMotorEx.class, "liftB"),
                hardwareMap.get(DigitalChannel.class, "Lift"),
                telemetry);

        arm = new Arm(hardwareMap.get(CRServo.class, "armL"),
                hardwareMap.get(CRServo.class, "armR"),
                hardwareMap,
                telemetry);

        detector = new OpenCv(hardwareMap.get(WebcamName.class, "Webcam 1"), signals);


    }
}

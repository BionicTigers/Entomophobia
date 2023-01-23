package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.NOdoDrivetrain;
import org.firstinspires.ftc.teamcode.teleop.NOdoRobot;
import org.firstinspires.ftc.teamcode.util.NOdoLocation;

@Autonomous (name="NOdoAuto", group="autonomous")
public class NOdoAuto extends LinearOpMode {

    public int[] motorNumbers = {0, 1, 2, 3}; //creates motor numbers array

    public NOdoRobot robot;
    public NOdoDrivetrain drivetrain;

    public NOdoLocation middleZone = new NOdoLocation(700, 0, 0);
    public NOdoLocation leftZone = new NOdoLocation(700, -600,0);
    public NOdoLocation rightZone = new NOdoLocation(700, 600,0);

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new NOdoRobot(this);
        drivetrain = new NOdoDrivetrain(robot, motorNumbers, telemetry, hardwareMap.get(Servo.class, "LeftOdo"), hardwareMap.get(Servo.class, "BackOdo"), hardwareMap.get(Servo.class, "RightOdo"));

        drivetrain.odoDown();

        waitForStart(); {
            drivetrain.moveToPositionMod(middleZone, 5, 5, 1, 0.2, 5000);
//            drivetrain.moveToPositionMod(leftZone, 5, 5,1, 0.2, 5000);
            drivetrain.moveToPositionMod(rightZone, 5, 5, 1, 0.2, 5000);
        }
    }
}

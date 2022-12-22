package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

@Autonomous(name = "OdoTest", group = "Autonomous")
public class OdoTestAuto extends LinearOpMode {
    public NOdoRobot robot;
    public NewDrivetrain drivetrain;
    public String[] odoServoNames = {"LeftOdo", "RightOdo", "BackOdo"};
    public HashMap<String, Servo> odoServos = new HashMap<>();
    public NOdoLocation Test;
    public void runOpMode(){
        for (String name : odoServoNames) {
            odoServos.put(name, hardwareMap.get(Servo.class, name));
        }
        robot = new NOdoRobot(this);
        drivetrain = new NewDrivetrain(robot, telemetry, odoServos);
        Test = new NOdoLocation(100, 0, 0);
        waitForStart();
        drivetrain.moveToPosition(Test, 5, 5, 2);
    }
}

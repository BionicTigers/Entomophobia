package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.ArrayList;

@TeleOp(name = "PortTest")
public class PortTestOp extends LinearOpMode {
    public Robot robot;
    public PortTest test;

    private ArrayList<DcMotorEx> motors;
    private ArrayList<Object> servos;

    public void runOpMode() {
        motors = new ArrayList<DcMotorEx>();
        servos = new ArrayList<Object>();

        /*motors.add(hardwareMap.get(DcMotorEx.class, "Motor0"));
        motors.add(hardwareMap.get(DcMotorEx.class, "Motor1"));
        motors.add(hardwareMap.get(DcMotorEx.class, "Motor2"));
        motors.add(hardwareMap.get(DcMotorEx.class, "Motor3"));*/

        servos.add(hardwareMap.get("Servo0"));
        servos.add(hardwareMap.get("Servo1"));
        /*servos.add(hardwareMap.get("Servo2"));
        servos.add(hardwareMap.get("Servo3"));
        servos.add(hardwareMap.get("Servo4"));
        servos.add(hardwareMap.get("Servo5"));*/
        test = new PortTest(motors, servos);

        waitForStart();

        while (opModeIsActive()) {
            test.update(gamepad1, gamepad2);
            test.write();
        }
    }
}
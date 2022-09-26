package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

@TeleOp(name = "PortTest")
public class PortTestOp extends LinearOpMode {
    public Robot robot;
    public PortTest test;

    private ArrayList<DcMotorSimple> motors;
    private ArrayList<Servo> servos;

    public void runOpMode() {
        motors = new ArrayList<>(hardwareMap.getAll(DcMotorSimple.class));
        servos = new ArrayList<>(hardwareMap.getAll(Servo.class));

        test = new PortTest(motors, servos);

        waitForStart();

        while (opModeIsActive()) {
            test.update(gamepad1, gamepad2);
            test.write();
        }
    }
}
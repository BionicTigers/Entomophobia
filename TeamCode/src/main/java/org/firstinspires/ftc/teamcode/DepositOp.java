package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "DepositOp")
public class DepositOp extends LinearOpMode {

    public Deposit deposit;
    @Override
    public void runOpMode() {

        deposit = new Deposit(hardwareMap.get(Servo.class, "depositServo"));

        waitForStart();

        while (opModeIsActive()) {
            deposit.update(gamepad1, gamepad2);
            deposit.write();
        }
    }
}
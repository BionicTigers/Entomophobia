package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "DepositOp")
public class DepositOp extends LinearOpMode {


    @Override
    public void runOpMode() {

         Deposit deposit = new Deposit(hardwareMap.get(Servo.class, "0"));

        waitForStart();

        while (opModeIsActive()) {
            deposit.update(gamepad1, gamepad2);
            deposit.write();
        }
    }
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class LiftOutputOp extends LinearOpMode {

    public Lift lift;
    public Deposit deposit;

    @Override
    public void runOpMode() throws InterruptedException {

        lift = new Lift (hardwareMap.get(DcMotorEx.class, "lift"));
        deposit = new Deposit(hardwareMap.get(Servo.class, "deposit"));

        waitForStart();

        while (opModeIsActive()) {
            lift.update(gamepad1, gamepad2);
            lift.write();
            deposit.update(gamepad1, gamepad2);
            deposit.write();
        }
    }
}

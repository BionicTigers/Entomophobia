package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "LiftOp")
public class LiftOp extends LinearOpMode {

    @Override
    public void runOpMode() {

        Lift lift = new Lift(hardwareMap.get(DcMotorEx.class, "rightLiftMotor"), hardwareMap.get(DcMotorEx.class, "leftLiftMotor"));

        waitForStart();

        while (opModeIsActive()) {
            lift.update(gamepad1, gamepad2);
            lift.write();
        }
    }
}


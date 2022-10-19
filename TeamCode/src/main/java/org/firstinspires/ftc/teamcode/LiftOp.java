package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "LiftOp")
public class LiftOp extends LinearOpMode {
    public Telemetry telemetry;

    public void runOpMode() {

        Lift lift = new Lift(hardwareMap.get(DcMotorEx.class, "liftMotor"));
        waitForStart();

        while (opModeIsActive()) {
            lift.update(gamepad1, gamepad2);
            lift.write();
            telemetry.update();
        }
    }
}


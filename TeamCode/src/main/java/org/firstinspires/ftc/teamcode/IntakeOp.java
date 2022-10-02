package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class IntakeOp extends LinearOpMode {

    @Override
    public void runOpMode() {


        Intake intake = new Intake(hardwareMap.get(DcMotorEx.class, "IntakeMotor"));

        waitForStart();

        while (opModeIsActive()) {
            intake.update(gamepad1, gamepad2);
            intake.write();
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Prototype")
public class PrototypeOp extends LinearOpMode {

    @Override
    public void runOpMode() {


        Prototype p = new Prototype(hardwareMap.get(DcMotorEx.class, "Motor"), hardwareMap.get(Servo.class, "Servo"));

        waitForStart();

        while (opModeIsActive()) {
            p.update(gamepad1, gamepad2);
            p.write();
        }
    }
}

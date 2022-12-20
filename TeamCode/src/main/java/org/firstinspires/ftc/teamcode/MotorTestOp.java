package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "MotorTest")
public class MotorTestOp extends LinearOpMode {
    MotorTest mt;
    public void runOpMode() throws InterruptedException {
        mt = new MotorTest(hardwareMap.get(DcMotorEx.class, "motor1"), hardwareMap.get(DcMotorEx.class, "motor2"));
        waitForStart();
        while(opModeIsActive()){
            mt.update(gamepad1, gamepad2);
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "EncoderTest")
public class EncoderTestOp extends LinearOpMode {
    public EncoderTest encoderTest;

    public void runOpMode() throws InterruptedException {
        encoderTest = new EncoderTest(hardwareMap.get(DcMotorEx.class, "motor"), telemetry);
        waitForStart();
        while(opModeIsActive()){
            encoderTest.update(gamepad1, gamepad2);
            telemetry.update();
        }
    }
}

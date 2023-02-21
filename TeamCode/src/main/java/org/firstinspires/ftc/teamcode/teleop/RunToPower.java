package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.ControlHub;
import org.firstinspires.ftc.teamcode.util.IndependentPID;

@TeleOp
public class RunToPower extends LinearOpMode {
    IndependentPID independentPid;
    DcMotorEx motor;

    ControlHub hub;

    @Override
    public void runOpMode() throws InterruptedException {
        independentPid = new IndependentPID(2, 0, 1, -1000, 1000);
        motor = hardwareMap.get(DcMotorEx.class, "frontRight");
        hub = new ControlHub(hardwareMap, (LynxDcMotorController) hardwareMap.get("Expansion Hub 2"));

        waitForStart();

        while (opModeIsActive()) {
            hub.refreshBulkData();

            double encoderPosition = hub.getEncoderTicks(2);
            double output = independentPid.calculate(700, encoderPosition);
            telemetry.addData("Current Position", encoderPosition);
            telemetry.addData("Output", output);
            telemetry.update();
        }
    }
}

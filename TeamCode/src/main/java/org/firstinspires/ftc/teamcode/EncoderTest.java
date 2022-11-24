package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class EncoderTest extends Mechanism{
    public DcMotorEx motor;
    public Telemetry telemetry;

    public EncoderTest(DcMotorEx m, Telemetry T){
        telemetry = T;
        motor = m;
        motors.add(motor);
        motors.get(0).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.get(0).setTargetPosition(0);
        motors.get(0).setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void update(Gamepad gp1, Gamepad gp2) {
        telemetry.addData("Position: ", motors.get(0).getCurrentPosition());
    }

    public void write() {

    }
}

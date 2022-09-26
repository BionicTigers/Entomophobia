package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

/*
   Motor that runs to position
   Servos to 0-1
 */

public class Prototype extends Mechanism{
    private boolean held = false;
    private boolean active = false;

    private DcMotorEx motor;
    private Servo servo;

    public Prototype(DcMotorEx m, Servo s) {
        motor = m;
        servo = s;

        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void update(Gamepad gp1, Gamepad gp2) {
        if (gp1.right_bumper && !held) {
            held = true;
            active = true;
        }
        else if (!gp1.right_bumper && held) {
            held = false;
            active = false;
        }
    }

    @Override
    public void write() {
        if (active) {
            motor.setPower(1);
            motor.setTargetPosition(500);
            motor.setPower(0);

            servo.setPosition(0.4);
        } else {
            motor.setPower(1);
            motor.setTargetPosition(0);
            motor.setPower(0);

            servo.setPosition(0);
        }
    }
}

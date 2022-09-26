package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

public class Intake extends Mechanism {
    private DcMotorEx motor;
    private boolean active = false;

    public Intake(DcMotorEx m) {
        motor = m;
    }

    @Override
    public void update(Gamepad gp1, Gamepad gp2) {
        active = gp1.right_trigger > 0.3;
    }

    @Override
    public void write() {
        if (active) {
            motor.setPower(1);
        } else {
            motor.setPower(0);
        }
    }
}

/*
    Pseudocode:
    When a button is held
    Turn on/off intake (Motor or Servo)
*/
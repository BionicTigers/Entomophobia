package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.util.Mechanism;

import java.util.concurrent.TimeUnit;

public class Arm extends Mechanism {
    public Telemetry telemetry;

    public CRServo left;
    public CRServo right;

    public Arm (CRServo l, CRServo r, Telemetry T) {
        super();
        //Declares the left motor
        crServos.add(l);
        left = l;
        //Declares the right motor
        crServos.add(r);
        right = r;

        //Flips the right servo so it rotates the right way
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        //Declares telemetry
        telemetry = T;
    }

    @Override
    public void update(Gamepad gp1, Gamepad gp2) {
        //Defaults to a 0 power move
        move(0);
        //Moves the arm upward slowly
        if (gp2.right_bumper) {
            move(0.4);
        }
        //Moves the arm downward slowly
        if (gp2.left_bumper) {
            move(-0.4);
        }
        //Moves the arm upward quickly
        if (gp2.dpad_up || gp2.dpad_left || gp2.dpad_down) {
            move(1);
        }
        //Moves the arm downward quickly
        if (gp2.dpad_right) {
            move(-1);
        }
    }

    @Override
    public void write() {
        //📝
    }

    //Moves the lift forward
    public void move(double mod) {
        //Sets both servo powers to whatever the mod is
        left.setPower(mod);
        right.setPower(mod);
    }
}

package org.firstinspires.ftc.teamcode.LastYearClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Mechanism;

public class Spinner extends Mechanism {


    //Controller buttons
    public boolean deployed;
    public boolean aIsPressed;
    public boolean spinning;
    public boolean spinningBack;
    public int x = 0;
    public int startSpot = 0;

    //Used to declare new instances of Spinner
    public Spinner(DcMotorEx spinner) {
        super();
        motors.add(spinner);
        motors.get(0).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motors.get(0).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.get(0).setTargetPosition(0);
        motors.get(0).setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void update(Gamepad gp1, Gamepad gp2) {
         if (gp2.right_trigger > .2) {
            autoSpin();
            aIsPressed = true;
        }

    }

    @Override
    public void write() {
        if (spinning && motors.get(0).getCurrentPosition() <= startSpot - (1300*4/3)) {
            motors.get(0).setVelocity(2500);
        }

    }

    public void autoSpin() {
        startSpot = motors.get(0).getCurrentPosition();
        motors.get(0).setTargetPosition(motors.get(0).getCurrentPosition() - 2900);
        motors.get(0).setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motors.get(0).setVelocity(1500);
        spinning = true;

    }

}

/*
    Pseudocode:
    (Potentially) Move automatically to the corner
    DIRECTIVE: Spin
    Ducky Profit
    Stop so duck resupply can occur
    Repeat last few steps
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * The lift class has 2 motors that lift a double reverse four bar lift.
 * @author Jack Gerber
 */
public class Lift extends Mechanism {
    public Telemetry telemetry;

    /**
     * Adds motors to the left and right variables
     * @param l imported left motor
     */
    public Lift (DcMotorEx l) {
        super();
        motors.add(l);
        motors.get(0).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motors.get(0).setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motors.get(0).setTargetPosition(0);
        motors.get(0).setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    /**
     * Takes inputs from controllers and converts them into robot movement
     * @param gp1 the drivetrain gamepad
     * @param gp2 the lift gamepad
     */
    public void update(Gamepad gp1, Gamepad gp2) {
        if (gp2.right_stick_y >= 0.25) {
            motors.get(0).setTargetPosition(50);
        }
        else if (gp2.right_stick_y <= -0.25) {
            motors.get(0).setTargetPosition(100);
        }
        else {
            motors.get(0).setTargetPosition(0);
        }
        telemetry.addData("Current position: ", motors.get(0).getCurrentPosition());
    }

    @Override
    public void write() {
        motors.get(0).setVelocity(1000);
    }
}
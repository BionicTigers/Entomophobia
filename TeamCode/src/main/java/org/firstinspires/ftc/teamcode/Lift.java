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
    public int height = 0;
    public int trim = 0;
    /**
     * Adds motors to the left and right variables
     * @param l imported left motor
     */
    public Lift (DcMotorEx l, Telemetry T) {
        super();
        motors.add(l);
        motors.get(0).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motors.get(0).setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motors.get(0).setTargetPosition(0);
        motors.get(0).setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        telemetry = T;
        height = 0;
    }

    /**
     * Takes inputs from controllers and converts them into robot movement
     * @param gp1 the drivetrain gamepad
     * @param gp2 the lift gamepad
     */
    public void update(Gamepad gp1, Gamepad gp2) {
//        if (gp2.right_stick_y >= 0.25) {
//            height = height + 10;
//        }
//        else if (gp2.right_stick_y <= -0.25) {
//            height = height - 10;
//        } else {
//            height = height;
//        }
//        if (height > 0) {
//            height = 0;
//        }

        if (gp2.left_stick_y <= -0.3) {
            height = -2150;
        } else if (gp2.left_stick_y >= 0.3) {
            height = 0;
            trim = 0;
        }

        if (gp2.right_stick_y >= 0.3) {
            trim = trim + 10;
        }
        if (gp2.right_stick_y <= -0.3) {
            trim = trim - 10;
        }
    }

//    //FOR SHOWCASE NIGHT USE ONLY
//    public void update(Gamepad gp1, Gamepad gp2) {
//        if (gp1.right_trigger >= 0.25) {
//            height = height + 10;
//        }
//        else if (gp1.left_trigger >= 0.25) {
//            height = height - 10;
//        }
//        telemetry.addData("Current position: ", motors.get(0).getCurrentPosition());
//    }

    @Override
    public void write() {
        //Sets the height of the lift to height + trim
        motors.get(0).setTargetPosition(height + trim);
        //Makes the lift motor move
        motors.get(0).setVelocity(1000);

        //Provides telemetry for the motor's current position and the trim value
        telemetry.addData("Current position: ", motors.get(0).getCurrentPosition());
        telemetry.addData("Trim", trim);
    }
}
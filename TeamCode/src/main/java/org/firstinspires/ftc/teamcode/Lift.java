package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * The lift class has 2 motors that lift a double reverse four bar lift.
 * @author Jack Gerber
 */
public class Lift extends Mechanism {
    public DcMotorEx left;
    public DcMotorEx right;

    /**
     * Adds motors to the left and right variables
     * @param l imported left motor
     * @param r imported right motor
     */
    public Lift (DcMotorEx l, DcMotorEx r) {
        super();
        left = l;
        motors.add(left);
        right = r;
        motors.add(right);
    }

    /**
     * Takes inputs from controllers and converts them into robot movement
     * @param gp1 the drivetrain gamepad
     * @param gp2 the lift gamepad
     */
    public void update(Gamepad gp1, Gamepad gp2) {
        if (gp2.right_stick_y >= 0.25) {
            left.setPower(50);
            right.setPower(50);
        }
        else if (gp2.right_stick_y <= 0.25) {
            left.setPower(-50);
            right.setPower(-50);
        }
        else {
            left.setPower(0);
            right.setPower(0);
        }
    }

    @Override
    public void write() {

    }
}
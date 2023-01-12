package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Mechanism;

/**
 * The lift class has 2 motors that lift a double reverse four bar lift.
 * @author Jack Gerber
 */
public class Lift extends Mechanism {
    //Declares fields
    private  DigitalChannel topLimitSwitch;
    private  DigitalChannel bottomLimitSwitch;

    public Telemetry telemetry;

    public int height = 0;
    public int trim = 0;

    public DcMotorEx left;
    public DcMotorEx right;

    /**
     * Adds motors to the left and right variables
     * @param l imported left motor
     * @param r imported right motor
     * @param top imported top limit switch
     * @param bottom imported bottom limit switch
     * @param T imported telemetry
     */
    public Lift (DcMotorEx l, DcMotorEx r, DigitalChannel top, DigitalChannel bottom, Telemetry T) {
        super();

        //Sets the fields to parameter values
        left = l;
        right = r;
        telemetry = T;

        //Adds to motors ArrayList
        motors.add(left);
        motors.add(right);

        //Prepares the left motor
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        left.setTargetPosition(0);
        left.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        //Prepares the right motor
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        right.setTargetPosition(0);
        right.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        //Sets the height starting position
        height = 0;

        //Declares limit switches
        topLimitSwitch = top;
        bottomLimitSwitch = bottom;
        sensors.add(topLimitSwitch);
        sensors.add(bottomLimitSwitch);
        topLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        bottomLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
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

        if (gp2.left_stick_y <= -0.3 && !bottomLimitSwitch.getState() && !topLimitSwitch.getState()) {
            height = -1500;
        } else if (gp2.left_stick_y >= 0.3 && !bottomLimitSwitch.getState() && !topLimitSwitch.getState()) {
            height = 0;
            trim = 0;
        }

        if (gp2.right_stick_y >= 0.3 && !bottomLimitSwitch.getState() && !topLimitSwitch.getState()) {
            trim = trim + 10;
        }
        if (gp2.right_stick_y <= -0.3 && !bottomLimitSwitch.getState() && !topLimitSwitch.getState()) {
            trim = trim - 10;
        }

//        //Moves the lift up when claw is closed so that cones can move over ground junctions
//        if(gp2.left_trigger > 0.3) {
//            Deadline hold = new Deadline(300, TimeUnit.MILLISECONDS);
//            if(hold.hasExpired()) {
//                height = -200;
//            }
//        }
    }

    /**
     * Writes the motor powers to the motors
     */
    @Override
    public void write() {
        right.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        left.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //Sets the height of the lift to height + trim
        right.setTargetPosition(height + trim);
        left.setTargetPosition(height + trim);
        //Makes the lift motor move
        right.setVelocity(1000);
        left.setVelocity(1000);

        //Provides telemetry for the motor's current position and the trim value
        telemetry.addData("Right Current position: ", right.getCurrentPosition());
        telemetry.addData("Left Current position: ", left.getCurrentPosition());
        telemetry.addData("Trim", trim);
        telemetry.addData("Height Value", height);
    }
}
package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.Mechanism;

/*
 * The lift class has 2 motors that lift a double reverse four bar lift.
 * @author Jack Gerber
 */
public class Lift extends Mechanism {
    //Declares fields
    private DigitalChannel limitSwitch;

    public Telemetry telemetry;

    public int height = 0;
    public int trim = 0;

    public DcMotorEx left;
    public DcMotorEx right;

    public boolean currentlyPressed = false;
    /*
     * Creates an enum for lift positions
     */
    public enum LiftState {
        HIGH, MEDIUM, LOW, INTAKE, STORING
    }
    /*
     * The current position of the lift
     */
    private LiftState liftState = LiftState.STORING;

    /*
     * Adds motors to the left and right variables
     * @param l imported left motor
     * @param r imported right motor
     //* @param top imported top limit switch
     //* @param bottom imported bottom limit switch
     * @param T imported telemetry
     */
    public Lift (DcMotorEx l, DcMotorEx r, DigitalChannel lift, Telemetry T) {
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

        //Declares limit switch
        limitSwitch = lift;
        sensors.add(limitSwitch);
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    /*
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
            height = -1250;
        } else if (gp2.left_stick_y >= 0.3) {
            height = 0;
            trim = 0;
        }

        if (gp2.right_stick_y >= 0.3) {
            trim = trim + 20;
        }
        if (gp2.right_stick_y <= -0.3) {
            trim = trim - 20;
        }

//        //Moves the lift up when claw is closed so that cones can move over ground junctions
//        if(gp2.left_trigger > 0.3) {
//            Deadline hold = new Deadline(300, TimeUnit.MILLISECONDS);
//            if(hold.hasExpired()) {
//                height = -200;
//            }
//        }

        //Allows for easier intake of cones in a stack
        if(gp2.dpad_up) {
            height += 10;
        }
        else if(gp2.dpad_down) {
            height -= 10;
        }

    }
    /*
     * Sets the lift to a given height
     * @param targetState the height to set the lift to
     */
    public void moveLift(LiftState targetState) {
        //Moves the lift to the desired height
        if (targetState == LiftState.INTAKE) {
            height = -1500;
            liftState = LiftState.INTAKE;
        } else if (targetState == LiftState.STORING) {
            height = -1000;
            liftState = LiftState.STORING;
        } else if (targetState == LiftState.LOW) {
            height = -500;
            liftState = LiftState.LOW;
        } else if (targetState == LiftState.MEDIUM) {
            height = -250;
            liftState = LiftState.MEDIUM;
        } else if (targetState == LiftState.HIGH) {
            height = 0;
            liftState = LiftState.HIGH;
        }

        left.setTargetPosition(height + trim);
        right.setTargetPosition(height + trim);
        left.setVelocity(1000);
        right.setVelocity(1000);
    }

    /*
     * Writes the motor powers to the motors
     */
    @Override
    public void write() {
        right.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        left.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        if (left.getCurrent(CurrentUnit.MILLIAMPS) > 3000 || right.getCurrent(CurrentUnit.MILLIAMPS) > 3000) {
//            height = -1100;
//        }
        //Sets the height of the lift to height + trim
        right.setTargetPosition(height + trim);
        left.setTargetPosition(height + trim);
        //Makes the lift motor move
        right.setVelocity(1000);
        left.setVelocity(1000);

        //Uses a limit switch to prevent the motor from trying to go too far
        if(!sensors.get(0).getState() && !currentlyPressed){
            trim = 0;
            motors.get(0).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors.get(1).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors.get(0).setTargetPosition(0);
            motors.get(1).setTargetPosition(0);
            motors.get(0).setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motors.get(1).setMode(DcMotor.RunMode.RUN_TO_POSITION);
            currentlyPressed = true;
        }

        if (sensors.get(0).getState() && currentlyPressed) {
            currentlyPressed = false;
        }

        //Provides telemetry for the motor's current position and the trim value
        telemetry.addData("Right Current position: ", right.getCurrentPosition());
        telemetry.addData("Left Current position: ", left.getCurrentPosition());
        telemetry.addData("Trim", trim);
        telemetry.addData("Height Value", height);
    }
}
package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.IndependentPID;
import org.firstinspires.ftc.teamcode.util.Mechanism;
import org.firstinspires.ftc.teamcode.util.PID;

public class OldLift extends Mechanism {
    //Declares fields
    private DigitalChannel limitSwitch;

    public Telemetry telemetry;

    public IndependentPID pid;

    //The height that we set the encoders to, negative is up for some reason, positive is also up because it loops around
    public int height = 0;
    //The amount to trim the height up and down by to make finer movements
    public int trim = 0;

    //Booleans used to make switch buttons, ones that track once per press instead of constantly
    public boolean bumpedUp = false;
    public boolean bumpedDown = false;


    //Declares the three motors (Bonus fact: The actual physical motors are called top, weast, and down respectively)
    public DcMotorEx top;
    public DcMotorEx middle;
    public DcMotorEx bottom;

    public boolean currentlyPressed = false;
    public boolean slowMode;

    public boolean goingUp;

    public enum position {GROUND, LOW, MEDIUM, HIGH, STACK}


    public OldLift(DcMotorEx t, DcMotorEx m, DcMotorEx b, DigitalChannel lift, Telemetry T) {
        super();

        //Sets the fields to parameter values
        top = t;
        middle = m;
        bottom = b;
        telemetry = T;

        //Adds the motors to the motors ArrayList
        motors.add(top);
        motors.add(middle);
        motors.add(bottom);

        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        top.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        middle.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        bottom.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        //Sets the height starting position
        height = 0;

        //Declares limit switch
        limitSwitch = lift;
        sensors.add(limitSwitch);
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        //Experimental PID stuff
        pid = new IndependentPID(1, 0, 0, 0, -1400);
    }

    public void update(Gamepad gp1, Gamepad gp2) {

        if (gp2.right_stick_y >= 0.3) {
            if (slowMode) {
                trim = trim + 5;
            } else {
                trim = trim + 10;
            }
        }
        if (gp2.right_stick_y <= -0.3) {
            if (slowMode) {
                trim = trim - 5;
            } else {
                trim = trim - 10;
            }
        }

        if (gp2.right_trigger > 0.3) {
            slowMode = true;
        } else {
            slowMode = false;
        }

        //A quick bump up to allow for easier grabbing of the cone stack
        if (gp2.left_stick_button && !bumpedUp) {
            trim += 60;
            bumpedUp = true;
        }
        if (!gp2.left_stick_button && bumpedUp) {
            bumpedUp = false;
        }

        //A quick bump down to allow for easier grabbing of the cone stack
        if (gp2.right_stick_button && !bumpedDown) {
            trim -= 60;
            bumpedDown = true;
        }
        if (!gp2.right_stick_button && bumpedDown) {
            bumpedDown = false;
        }

        if (gp2.dpad_up) {
            height = -800;
        }
        if (gp2.dpad_left) {
            height = -380;
        }
        if (gp2.dpad_down) {
            height = -120;
        }
        if (gp2.dpad_right) {
            height = 0;
            trim = 0;
        }
    }

    private boolean over = false;

    /*
     * Writes the motor powers to the motors
     */
    @Override
    public void write() {
        top.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        middle.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        bottom.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        if (top.getCurrent(CurrentUnit.MILLIAMPS) > 7500 || middle.getCurrent(CurrentUnit.MILLIAMPS) > 7500 || bottom.getCurrent(CurrentUnit.MILLIAMPS) > 7500 && !over) {
            trim += 10;
            over = true;
        } else if (over && top.getCurrent(CurrentUnit.MILLIAMPS) < 7500 && middle.getCurrent(CurrentUnit.MILLIAMPS) < 7500 && bottom.getCurrent(CurrentUnit.MILLIAMPS) < 7500) {
            over = false;
        }

        //Sets the height of the lift to PIDed height + trim
        double output = pid.calculate(height + trim, middle.getCurrentPosition());

        //Makes the lift motor move
        for (DcMotorEx motor : motors) {
            if (!limitSwitch.getState() && height == 0 && trim == 0) {
                motor.setPower(0);
            } else {
                motor.setPower(output);
            }
        }

        //PID set powers
//        top.setPower(-pid.calculate(height + trim, middle.getCurrentPosition()));
//        middle.setPower(pid.calculate(height + trim, middle.getCurrentPosition()));
//        bottom.setPower(-pid.calculate(height + trim, middle.getCurrentPosition()));

//        if (middle.getTargetPosition() > middle.getCurrentPosition()) {
//            goingUp = true;
//        } else {
//            goingUp = false;
//        }
//
//        if (goingUp) {
//            top.setPower(-middle.getVelocity() / 2900);
//            bottom.setPower(-middle.getVelocity() / 2900);
//        } else {
//            top.setPower(middle.getVelocity() / 2900);
//            bottom.setPower(middle.getVelocity() / 2900);
//        }


        //Uses a limit switch to prevent the motor from trying to go too far
        if (!limitSwitch.getState() && !currentlyPressed) {
            top.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            middle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            height = 0;
            trim = 0;
            top.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            middle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bottom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            currentlyPressed = true;
        }

        if (limitSwitch.getState() && currentlyPressed) {
            currentlyPressed = false;
        }

        //Provides telemetry for the motor's current position and the trim value
//        telemetry.addData("Right Current position: ", middle.getCurrentPosition());
//        telemetry.addData("Left Current position: ", top.getCurrentPosition());
        telemetry.addData("Trim", trim);
        telemetry.addData("Height Value", height);
        telemetry.addData("Limit Switch Pressed", limitSwitch.getState());
//        telemetry.addData("Double velocity", (double) middle.getVelocity());
//        telemetry.addData("Velocity", middle.getVelocity());
//        telemetry.addData("Ticks", middle.getCurrentPosition());
//        telemetry.addData("Top power", top.getPower());
//        telemetry.addData("Weast power", middle.getPower());
//        telemetry.addData("Down power", bottom.getPower());
    }

    public void lift(position pos) {
        //Changes the height field
        switch(pos){
            case HIGH:
                height = -1400;
                break;
            case MEDIUM:
                height = -380;
                break;
            case LOW:
            case GROUND:
                height = -120;
                break;
        }

        //Sets the target position
        top.setTargetPosition(height);
        middle.setTargetPosition(-(height));
        bottom.setTargetPosition(height);

        //Moves the motors
        if (!limitSwitch.getState() && height == 0 && trim == 0) {
            top.setPower(0);
            middle.setPower(0);
            bottom.setPower(0);
        } else if (middle.getTargetPosition() == 0) {
            top.setPower(0.15);
            middle.setPower(0.15);
            bottom.setPower(0.15);
        } else {
            top.setPower(1);
            middle.setPower(1);
            bottom.setPower(1);
        }
    }
}
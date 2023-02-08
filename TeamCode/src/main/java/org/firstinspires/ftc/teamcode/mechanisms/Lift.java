package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.Mechanism;

public class Lift extends Mechanism {
    //Declares fields
    private DigitalChannel limitSwitch;

    public Telemetry telemetry;

    public int height = 0;
    public int trim = 0;

    public boolean trimmedUp = false;
    public boolean trimmedDown = false;

    public DcMotorEx top;
    public DcMotorEx middle;
    public DcMotorEx bottom;

    public boolean currentlyPressed = false;
    public boolean slowMode;

    public boolean goingUp;


    public Lift (DcMotorEx t, DcMotorEx m, DcMotorEx b, DigitalChannel lift, Telemetry T) {
        super();

        //Sets the fields to parameter values
        top = t;
        middle = m;
        bottom = b;
        telemetry = T;

        //Adds to motors ArrayList
        motors.add(top);
        motors.add(middle);
        motors.add(bottom);

        //Prepares the top motor
        top.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        top.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        top.setTargetPosition(0);
        top.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        //Prepares the middle motor
        middle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        middle.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        middle.setTargetPosition(0);
        middle.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        //Prepares the bottom motor
        bottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bottom.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bottom.setTargetPosition(0);
        bottom.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        //Sets the height starting position
        height = 0;

        //Declares limit switch
        limitSwitch = lift;
        sensors.add(limitSwitch);
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
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

        if (gp2.left_stick_button && !trimmedUp) {
            trim += 60;
            trimmedUp = true;
        }
        if (!gp2.left_stick_button && trimmedUp) {
            trimmedUp = false;
        }

        if (gp2.right_stick_button && !trimmedDown) {
            trim -= 60;
            trimmedDown = true;
        }
        if (!gp2.right_stick_button && trimmedDown) {
            trimmedDown = false;
        }

        if (gp2.dpad_up) {
            height = -1400;
        }
        if (gp2.dpad_left) {
            height = -1000;
        }
        if (gp2.dpad_down) {
            height = -600;
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
        top.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        middle.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        bottom.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        if (top.getCurrent(CurrentUnit.MILLIAMPS) > 7500 || middle.getCurrent(CurrentUnit.MILLIAMPS) > 7500 || bottom.getCurrent(CurrentUnit.MILLIAMPS) > 7500 && !over) {
            trim += 10;
            over = true;
        } else if (over && top.getCurrent(CurrentUnit.MILLIAMPS) < 7500 && middle.getCurrent(CurrentUnit.MILLIAMPS) < 7500 && bottom.getCurrent(CurrentUnit.MILLIAMPS) < 7500) {
            over = false;
        }

        //Sets the height of the lift to height + trim
        top.setTargetPosition(height + trim);
        middle.setTargetPosition(-(height + trim));
        bottom.setTargetPosition(height + trim);

        //Makes the lift motor move
        if (!sensors.get(0).getState() && middle.getTargetPosition() == 0) {
            top.setPower(0);
            middle.setPower(0);
            bottom.setPower(0);
        } else if (middle.getTargetPosition() == 0) {
            top.setPower(0.1);
            middle.setPower(0.1);
            bottom.setPower(0.1);
        } else {
            top.setPower(1);
            middle.setPower(1);
            bottom.setPower(1);
        }

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
        if (!sensors.get(0).getState() && !currentlyPressed){
            trim = 0;
            top.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            middle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            top.setTargetPosition(0);
            middle.setTargetPosition(0);
            bottom.setTargetPosition(0);
            top.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            middle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bottom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            currentlyPressed = true;
        }

        if (sensors.get(0).getState() && currentlyPressed) {
            currentlyPressed = false;
        }

        //Provides telemetry for the motor's current position and the trim value
//        telemetry.addData("Right Current position: ", middle.getCurrentPosition());
//        telemetry.addData("Left Current position: ", top.getCurrentPosition());
        telemetry.addData("Trim", trim);
        telemetry.addData("Height Value", height);
//        telemetry.addData("Double velocity", (double) middle.getVelocity());
//        telemetry.addData("Velocity", middle.getVelocity());
//        telemetry.addData("Ticks", middle.getCurrentPosition());
//        telemetry.addData("Top power", top.getPower());
//        telemetry.addData("Weast power", middle.getPower());
//        telemetry.addData("Down power", bottom.getPower());
    }
}
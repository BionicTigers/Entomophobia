package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Mechanism;

public class Lift extends Mechanism {
    //Declares fields
    private DigitalChannel limitSwitch;

    public Telemetry telemetry;

    public int height = 0;
    public int trim = 0;

    public DcMotorEx top;
    public DcMotorEx middle;
    public DcMotorEx bottom;

    public boolean currentlyPressed = false;
    public boolean slowMode;


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
        middle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        middle.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        middle.setTargetPosition(0);
        middle.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        //Sets the height starting position
        height = 0;

        //Declares limit switch
        limitSwitch = lift;
        sensors.add(limitSwitch);
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    public void update(Gamepad gp1, Gamepad gp2) {

        if (gp2.left_stick_y <= -0.3) {
            height = -1400;
        } else if (gp2.left_stick_y >= 0.3) {
            height = 0;
            trim = 0;
        }

        if (gp2.right_stick_y >= 0.3) {
            if (slowMode) {
                trim = trim + 20;
            } else {
                trim = trim + 40;
            }
        }
        if (gp2.right_stick_y <= -0.3) {
            if (slowMode) {
                trim = trim - 20;
            } else {
                trim = trim - 40;
            }
        }

        if (gp2.right_trigger > 0.3) {
            slowMode = true;
        } else {
            slowMode = false;
        }
    }

    /*
     * Writes the motor powers to the motors
     */
    @Override
    public void write() {
        top.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        middle.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        bottom.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        if (left.getCurrent(CurrentUnit.MILLIAMPS) > 3000 || right.getCurrent(CurrentUnit.MILLIAMPS) > 3000) {
//            height = -1100;
//        }
        //Sets the height of the lift to height + trim
        top.setTargetPosition(height + trim);
        middle.setTargetPosition(-(height + trim));
        bottom.setTargetPosition(height + trim);
        //Makes the lift motor move
        if (!sensors.get(0).getState() && middle.getTargetPosition() == 0) {
            top.setVelocity(0);
            middle.setVelocity(0);
            bottom.setVelocity(0);
        } else {
            top.setVelocity(1000);
            middle.setVelocity(1000);
            bottom.setVelocity(1000);
        }

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
    }
}
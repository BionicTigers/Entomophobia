package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Mechanism;

public class Lift extends Mechanism {

    public DcMotorEx top, middle, bottom;

    public DigitalChannel limitSwitch;

    public Telemetry telemetry;
    //TargetHeight is the encoder position we want to go to, and trim is how much to trim from there
    public int targetHeight = 0;
    public int trim = 0;

    //Booleans used to make switch buttons, ones that track once per press instead of constantly
    public boolean bumpedUp = false;
    public boolean bumpedDown = false;

    public boolean killPower = false;
    public boolean limitPressed = false;

    private int cycles = 0;

    public enum Position {GROUND, LOW, MEDIUM, HIGH, STACK}

    public Lift(DcMotorEx t, DcMotorEx m, DcMotorEx b, DigitalChannel limit, Telemetry telem) {
        super();

        //Declares the motors
        top = t;
        middle = m;
        bottom = b;
        //Adds the motors to the array list
        motors.add(top);
        motors.add(middle);
        motors.add(bottom);
        //Resets the encoders on all the motors and sets their zero power behavior to float
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            motor.setTargetPosition(0);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        //This is temporary stuff, might make motors sad and fight too much
//        top.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        middle.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        bottom.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        //Declares the limit switch
        limitSwitch = limit;
        //Adds the limit switch to the array list and sets it as an input
        sensors.add(limitSwitch);
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        //Declares the telemetry
        telemetry = telem;
    }

    public void update(Gamepad gp1, Gamepad gp2) {
        //Sets targetHeight to the high height
        if (gp2.dpad_up) {
            targetHeight = -800;
            killPower = false;
        }
        //Sets targetHeight to the middle height
        if (gp2.dpad_left) {
            targetHeight = -380;
            killPower = false;
        }
        //Sets targetHeight to the low height
        if (gp2.dpad_down) {
            targetHeight = 0;
            killPower = false;
        }
        //Sets targetHeight to the lowest point
        if (gp2.dpad_right) {
            targetHeight = 50;
        }

        if (trim != 0) {
            killPower = false;
        }

        //Increase the trim
        if (gp2.right_stick_y >= 0.3) {
            trim += 10;
        }
        //Decreases the trim
        if (gp2.right_stick_y <= -0.3) {
            trim -= 10;
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
    }
    //🍦
    public void write() {
        //Sets the target position for each motor, middle being reversed

        if (!killPower) {
            for (DcMotorEx motor : motors) {
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            top.setTargetPosition(targetHeight + trim);
            middle.setTargetPosition(-(targetHeight + trim));
            bottom.setTargetPosition(targetHeight + trim);
        }

        cycles++;

        if (cycles > 3) {
            limitPressed = limitSwitch.getState();
            cycles = 0;
        }

        if (!limitPressed) {
            killPower = true;
        }

        if (!limitPressed && killPower) {
            for (DcMotorEx motor : motors) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            trim = 0;
            targetHeight = 0;
        }

        //If the limit switch is pressed and we're trying to go down
        if (killPower) {
            //Sets powers to 0 to prevent motor strain
            for (DcMotorEx motor : motors) {
                motor.setPower(0);
            }
        } else if (targetHeight == 50) {
            //Sets powers to 0.2 when going down to prevent crashing
            for (DcMotorEx motor : motors) {
                motor.setPower(0.2);
            }
        } else {
            //Sets powers to 1 so the motors move
            for (DcMotorEx motor : motors) {
                motor.setPower(1);
            }
        }

        //Telemetry information
        telemetry.addData("Trim", trim);
        telemetry.addData("Height Value", targetHeight);
        telemetry.addData("Limit Switch Pressed", limitPressed);
    }

    public void lift(Lift.Position pos) {
        //Changes the height field
        switch(pos){
            case HIGH:
                targetHeight = -1400;
                break;
            case MEDIUM:
                targetHeight = -380;
                break;
            case LOW:
            case GROUND:
                targetHeight = -120;
                break;
        }

        //Sets the target position
        top.setTargetPosition(targetHeight);
        middle.setTargetPosition(-(targetHeight));
        bottom.setTargetPosition(targetHeight);

        //Moves the motors
        if (!limitSwitch.getState() && targetHeight == 0) {
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

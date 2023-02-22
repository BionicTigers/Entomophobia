package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Mechanism;

public class BensLift extends Mechanism {

    public DcMotorEx top, middle, bottom;

    public DigitalChannel limitSwitch;

    public Telemetry telemetry;
    //SetHeight is the encoder position we want to go to, and trim is how much to trim from there
    public int setHeight = 0;
    public int trim = 0;

    public BensLift (DcMotorEx t, DcMotorEx m, DcMotorEx b, DigitalChannel limit, Telemetry telem) {
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
        }
        //Declares the limit switch
        limitSwitch = limit;
        //Adds the limit switch to the array list and sets it as an input
        sensors.add(limitSwitch);
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        //Declares the telemetry
        telemetry = telem;
    }

    public void update(Gamepad gp1, Gamepad gp2) {

    }

    public void write() {

    }
}

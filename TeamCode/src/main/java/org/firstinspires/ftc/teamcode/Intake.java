package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

public class Intake extends Mechanism {
    Deadline stop = new Deadline(500, TimeUnit.MILLISECONDS);

    //Both of these variables control the direction of the intake
    public boolean goingIn;
    public boolean goingOut;
    public boolean deposit;
    public boolean override;
    public DistanceSensor distance;

    public DcMotorEx intake;

    //Used for intake method
    //ColorSensor color = hardwareMap.get(ColorSensor.class, "color");

    /*
     * Creates, declares, and assigns a motor to the motors array list
     */
    public Intake(DcMotorEx intake, Servo intake2, DistanceSensor d) {
        super();
        this.intake = intake;
        motors.add(intake);
        motors.get(0).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors.get(0).setDirection(DcMotorSimple.Direction.FORWARD);
        servos.add(intake2);
        distance = d;
    }
    public Intake(DcMotorEx intake, Servo intake2) {
        super();
        this.intake = intake;
        motors.add(intake);
        motors.get(0).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        servos.add(intake2);
    }

    //Intakes until the color sensor detects freight
    public void intake() {

    }
    //Intakes for a given time in seconds
    public void intake(int duration) {
        Deadline time = new Deadline(duration, TimeUnit.SECONDS);
        while(!time.hasExpired()){
            intake.setVelocity(60); //Add number
            intake.setPower(60); //Add number
        }
    }

    //Deposits freight while color sensor says it's there
    public void deposit() {
        //Deposit freight
    }
    //Deposits for a given time in seconds
    public void deposit(int duration) {
        Deadline time = new Deadline(duration, TimeUnit.SECONDS);
        while(!time.hasExpired()){
 //Add number
            intake.setPower(50); //Add number
        }
    }

    /*
     * Controls the robot during TeleOp and sends input to run
     */
    public void update(Gamepad gp1, Gamepad gp2) {
        goingIn = gp1.right_trigger >= .3;
        goingOut = gp1.left_trigger >= .3;
        deposit = gp1.left_bumper;

        if(override && gp1.right_trigger <= .3 && distance.getDistance(DistanceUnit.CM)>= .8){
            override = false;
        } else if(gp1.a || distance.getDistance(DistanceUnit.CM)< .8){override = true;}
    }

     //Controls the intake
    public void write() {
        run(goingIn, goingOut);
        if(deposit) motors.get(0).setPower(-.5);
        if(deposit){
            servos.get(0).setPosition(0.1);
        } else if(goingIn && !override){
            servos.get(0).setPosition(0.1);
        }
        else {
            servos.get(0).setPosition(0.6);
        }
    }

     //Controls the intake during TeleOp using input from update
    public void run(boolean in, boolean out) {
        if (in) {
            motors.get(0).setPower(1);
        } else if (out) {
            motors.get(0).setPower(-1);
        } else if (deposit) {
            motors.get(0).setPower(0.1);
        } else {
            motors.get(0).setPower(0);
        }
    }
}

/*
    Pseudocode:
    When a button is pressed
    Turn on/off intake (Motor or Servo)
*/
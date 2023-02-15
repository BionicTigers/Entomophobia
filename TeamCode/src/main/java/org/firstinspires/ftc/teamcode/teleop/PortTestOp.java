package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.OldRobot;

import java.util.ArrayList;
import java.util.HashMap;

@TeleOp(name = "PortTest")
public class PortTestOp extends LinearOpMode {
    public OldRobot robot;

    private ArrayList<DcMotorSimple> motors;
    private ArrayList<Servo> servos;

    private boolean activate = false;
    private boolean indexChanged = false;
    private boolean speedChanged = false;
    public int index = 0;
    public float speed = 0;
    private ArrayList<Object> objects = new ArrayList<>();

    private HashMap<Integer, Double> POWERS = new HashMap<>();

    public void update(Gamepad gp1, Gamepad gp2) {
        //Detect for index changes
        if (gp1.left_bumper && !indexChanged) {
            index = Math.max(0, index-1);
            indexChanged = true;
        } else if (gp1.right_bumper && !indexChanged) {
            index = Math.min(objects.size()-1, index+1);
            indexChanged = true;
        }

        //Basically just a trigger for a button
        if (!gp1.left_bumper && !gp1.right_bumper && indexChanged) {
            indexChanged = false;
        }

        //Detect for speed changes
        if (gp1.dpad_left && !speedChanged) {
            speed = Math.max(0f, speed - 0.1f);
            speedChanged = true;
        } else if (gp1.dpad_right && !speedChanged) {
            speed = Math.min(1.0f, speed + 0.1f);

            speedChanged = true;
        }

        //Basically just a trigger for a button
        if (!gp1.dpad_left && !gp1.dpad_right && speedChanged) {
            speedChanged = false;
        }

        //Don't hold down x and change the index.
        activate = gp1.x;

        telemetry.addData("Index: ", index);
        telemetry.addData("Speed: ", speed);
    }

    public void write() {
        Object thingToActivate = objects.get(index);

        if (activate) {
            if (thingToActivate instanceof DcMotorSimple) {
                //Little Confusing - Explanation: If POWERS contains index then get the value, else return the set speed
                ((DcMotorSimple) thingToActivate).setPower(POWERS.containsKey(index) ? POWERS.get(index) : speed);
            } else if (thingToActivate instanceof Servo) {
                ((Servo) thingToActivate).setPosition(POWERS.containsKey(index) ? POWERS.get(index) : speed);
            }
        } else {
            if (thingToActivate instanceof DcMotorSimple) {
                ((DcMotorSimple) thingToActivate).setPower(0);
            } else if (thingToActivate instanceof Servo) {
                ((Servo) thingToActivate).setPosition(0);
            }
        }
    }

    public void runOpMode() {
        motors = new ArrayList<>(hardwareMap.getAll(DcMotorSimple.class));
        servos = new ArrayList<>(hardwareMap.getAll(Servo.class));

        objects.addAll(motors);
        objects.addAll(servos);

        waitForStart();

        while (opModeIsActive()) {
            update(gamepad1, gamepad2);
            write();
            telemetry.update();
        }
    }
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.ArrayList;

//"Object" is any class
//In the future it would be better to change to it's own

public class PortTest extends Mechanism {
    private boolean activate = false;
    private boolean indexChanged = false;
    private boolean speedChanged = false;
    public int index = 0;
    public float speed = 0;
    public Telemetry telemetry;
    private ArrayList<Object> objects = new ArrayList<>();

    private HashMap<Integer, Double> POWERS = new HashMap<>();

    public PortTest(Telemetry T, ArrayList<DcMotorSimple> m, ArrayList<Servo> s)
    {
        telemetry = T;

        //Merge the Motor and Servo list
        objects.addAll(m);
        objects.addAll(s);

        //Comment this out if you are not testing end positions
        //POWERS.put(0, 0.3);
    }

    @Override
    public void update(Gamepad gp1, Gamepad gp2) {
        //Detect for index changes
        if (gp1.left_bumper && !indexChanged) {
            index = Math.max(0, index-1);
            telemetry.addLine("Index: " + index);
            indexChanged = true;
        } else if (gp1.right_bumper && !indexChanged) {
            index = Math.min(objects.size()-1, index+1);
            telemetry.addLine("Index: " + index);
            indexChanged = true;
        }

        //Basically just a trigger for a button
        if (!gp1.left_bumper && !gp1.right_bumper && indexChanged) {
            indexChanged = false;
        }

        //Detect for speed changes
        if (gp1.dpad_left && !speedChanged) {
            speed = Math.max(0f, speed - 0.1f);
            telemetry.addLine("Speed: " + speed);
            speedChanged = true;
        } else if (gp1.dpad_right && !speedChanged) {
            speed = Math.min(1.0f, speed + 0.1f);
            telemetry.addLine("Speed: " + speed);

            speedChanged = true;
        }

        //Basically just a trigger for a button
        if (!gp1.dpad_left && !gp1.dpad_right && speedChanged) {
            speedChanged = false;
        }

        //Don't hold down x and change the index.
        activate = gp1.x;
        telemetry.update();
    }

    @Override
    public void write() {
        Object thingToActivate = objects.get(index);
        telemetry.addLine("Help: " + thingToActivate.toString());

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
}
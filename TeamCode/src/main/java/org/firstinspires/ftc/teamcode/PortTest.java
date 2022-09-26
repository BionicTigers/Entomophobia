package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.ArrayList;

public class PortTest extends Mechanism {
    private boolean activate = false;
    private boolean indexChanged = false;
    private boolean speedChanged = false;
    private int index = 0;
    private float speed = 0;
    public Telemetry telemetry;
    private ArrayList<Object> objects = new ArrayList<>();

    private HashMap<Integer, Double> POWERS = new HashMap<>();

    public PortTest(ArrayList<DcMotorSimple> m, ArrayList<Servo> s)
    {
        objects.addAll(m);
        objects.addAll(s);

        //Comment this out if you are not testing end positions
        POWERS.put(0, 0.3);
    }

    @Override
    public void update(Gamepad gp1, Gamepad gp2) {
        //Detect for index changes
        if (gp1.left_bumper && !indexChanged) {
            index = Math.max(0, index-1);
            telemetry.addData("Index", index);
            indexChanged = true;
        } else if (gp1.right_bumper && !indexChanged) {
            index = Math.min(objects.size(), index+1);
            telemetry.addData("Index", index);
            indexChanged = true;
        }

        //Basically just a trigger for a button
        if (!gp1.left_bumper && indexChanged) {
            indexChanged = false;
        } else if (!gp1.right_bumper && indexChanged) {
            indexChanged = false;
        }

        //Detect for speed changes
        if (gp1.dpad_left && !speedChanged) {
            speed = Math.max(0f, speed - 0.1f);
            telemetry.addData("Speed", speed);
            speedChanged = true;
        } else if (gp1.dpad_right && !speedChanged) {
            speed = Math.max(1f, speed + 0.1f);
            telemetry.addData("Speed", speed);
            speedChanged = true;
        }

        //Basically just a trigger for a button
        if (!gp1.dpad_left && speedChanged) {
            speedChanged = false;
        } else if (!gp1.dpad_right && speedChanged) {
            speedChanged = false;
        }

        activate = gp1.x;
        telemetry.update();
    }

    @Override
    public void write() {
        Object thingToActivate = objects.get(index);

        if (activate) {
            if (thingToActivate instanceof DcMotorSimple) {
                ((DcMotorSimple) thingToActivate).setPower(speed);
            } else if (thingToActivate instanceof Servo) {
                //Little Confusing - Explanation: If POWERS contains index then get the value, else return the set speed
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